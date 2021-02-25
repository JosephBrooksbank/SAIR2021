import math
import numpy
import random
import nav_msgs.msg
import rospy
from sklearn.cluster import AgglomerativeClustering
import geometry_msgs.msg
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA


class Map:
    """ Base Map Class"""
    ROBOT_WIDTH = 5

    def __init__(self, **kwargs):

        if "data" in kwargs:
            data = kwargs["data"]
            assert isinstance(data, nav_msgs.msg.OccupancyGrid)
            self.width = data.info.width
            self.height = data.info.height
            self.resolution = data.info.resolution
            self.origin = data.info.origin
            # raw map data from sensors
            self.map_2d = numpy.reshape(data.data, (self.width, self.height))

        elif "map_data" in kwargs:
            map_data = kwargs["map_data"]
            assert isinstance(map_data, Map)
            self.width = map_data.width
            self.height = map_data.height
            self.resolution = map_data.resolution
            self.origin = map_data.origin

            self.map_2d = numpy.copy(map_data.map_2d)

        elif "size" in kwargs:
            size = kwargs["size"]
            self.width = size.width
            self.height = size.height
            self.resolution = size.resolution
            self.origin = size.origin

            self.map_2d = numpy.zeros((self.width, self.height))

    def fill_square(self, center, radius, value):
        """ Fills a square of the value given at the center and radius given, in the 2D array given. """
        width = self.map_2d.shape[0]
        height = self.map_2d.shape[1]
        for i in range(center[0] - radius, center[0] + radius):
            if i < 0:
                i = 0
            if i > width - 1:
                i = width - 1
            for j in range(center[1] - radius, center[1] + radius):
                if j < 0:
                    j = 0
                if j > height - 1:
                    j = height - 1
                self.map_2d[i, j] = value

    @staticmethod
    def generate_neighbors(map_data, i, j):
        # type: (Map, int, int) -> (int, int, int, int, int)
        """ Get the value of neighboring values in a the current map

        Arguments:
            map_data: the map to get values from
            i,j: the coordinates whos neighbors should be given

        returns: center, left, right, up, down, as integer values corresponding to that position
        """

        center = map_data.map_2d[i, j]
        if i == 0:
            left = map_data.map_2d[i, j]
        else:
            left = map_data.map_2d[i - 1, j]
        if i == map_data.width - 1:
            right = map_data.map_2d[i, j]
        else:
            right = map_data.map_2d[i + 1, j]
        if j == 0:
            up = map_data.map_2d[i, j]
        else:
            up = map_data.map_2d[i, j - 1]
        if j == map_data.height - 1:
            down = map_data.map_2d[i, j]
        else:
            down = map_data.map_2d[i, j + 1]

        return center, left, right, up, down


class FrontierMap(Map):
    """ Class for Frontier Maps, including generation of map and occupancy grid"""

    def __init__(self, map_data):
        # type: (Map) -> None
        assert isinstance(map_data, Map)
        Map.__init__(self, size=map_data)

        # Shallow copy of map to generate obstacles
        obstacle_map_2d = Map(map_data=map_data)

        for i in range(self.width):
            for j in range(self.height):
                if map_data.map_2d[i, j] == 100:
                    obstacle_map_2d.fill_square([i, j], Map.ROBOT_WIDTH, 100)

            # for every item in map, if unknown and next to a known, it is a frontier
        for i in range(self.width):
            for j in range(self.height):
                center, left, right, up, down = Map.generate_neighbors(obstacle_map_2d, i, j)

                if center == -1:
                    if left == 0 or right == 0 or up == 0 or down == 0:
                        self.map_2d[i, j] = 100
                    else:
                        self.map_2d[i, j] = 0

    def to_occupancy_grid(self):
        # type: () -> nav_msgs.msg.OccupancyGrid
        occupy_grid = nav_msgs.msg.OccupancyGrid()
        occupy_grid.header.frame_id = "map"
        occupy_grid.header.stamp = rospy.Time.now()

        occupy_grid.info.width = self.width
        occupy_grid.info.height = self.height
        occupy_grid.info.resolution = self.resolution
        occupy_grid.info.origin = self.origin
        occupy_grid.info.map_load_time = rospy.Time.now()
        occupy_grid.data = self.map_2d.flatten()
        return occupy_grid


class ClusterMap(Map):
    """ Class for Cluster Maps; including generation of map and conversion to visual markers"""

    def __init__(self, map_data):
        # type: (FrontierMap) -> None
        assert isinstance(map_data, FrontierMap)
        self.center_points = []
        self.mark_id = 1

        # Initially, map is all zeros
        Map.__init__(self, size=map_data)

        # Creating an array of frontier locations as [x,y] pairs for cluster algorithm
        point_array = []  # type: [int,int]
        for i in range(self.width):
            for j in range(self.height):
                if map_data.map_2d[i, j] != 0:
                    point_array.append([i, j])

        # through experimentation, this was the best way to cluster
        cluster_values = AgglomerativeClustering(n_clusters=4, ).fit_predict(point_array)
        cluster_groups = {}

        # There is a cluster labeled "0" by default, which is also my empty space. This caused
        # issues for an embarrassingly long time.
        for i in range(len(cluster_values)):
            cluster_values[i] += 1

        # Re building a map with the index of the cluster as the value at that position
        for i in range(len(point_array)):
            cur_value = cluster_values[i]

            # the clustering algorithm maintains order of the array, so I can get which coordinates
            # the cluster index was originally part of
            coords = point_array[i]

            # Sorting into groups based on cluster index, for centroid creation
            if cur_value not in cluster_groups:
                cluster_groups[cur_value] = [coords]
            else:
                cluster_groups[cur_value].append(coords)

            self.map_2d[coords[0], coords[1]] = cur_value

        # Saving center points of all clusters in map coordinates
        for clusterID in cluster_groups:
            x_c = y_c = count = 0
            for coord_pair in cluster_groups[clusterID]:
                x_c += coord_pair[1]
                y_c += coord_pair[0]
                count += 1
            x_c = x_c / count
            y_c = y_c / count

            xy = self.grid_to_map_coords([x_c, y_c])
            self.center_points.append(xy)

    def make_sphere_marker(self, xy, radius, color):
        # type: ([int,int], float, ColorRGBA) -> Marker
        """ Function to generate a spherical marker

            ARGUMENTS:
                x, y: coordinates of point in grid space
                radius: float
                color: ColorRGBA
                mark_id: unique id
                off_x, off_y: Origin of map coordinates in world frame """
        marker = Marker()

        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'frontier'
        marker.id = self.mark_id
        self.mark_id += 1
        marker.type = 2
        marker.action = 0
        marker.pose = geometry_msgs.msg.Pose()
        marker.pose.orientation.w = 1
        marker.pose.position.x = xy[0]
        marker.pose.position.y = xy[1]
        marker.scale = geometry_msgs.msg.Vector3(radius, radius, radius)
        marker.color = color
        marker.lifetime = rospy.rostime.Duration(0)
        marker.frame_locked = True
        return marker

    def to_markers(self):
        # type: () -> MarkerArray
        """places markers in map space based on clusters in grid space, with centroids visible."""

        colors = {}
        markers = []

        # unique identifier for each sphere marker

        for i in range(self.width):
            for j in range(self.height):
                cluster_value = self.map_2d[i, j]
                if cluster_value != 0:
                    if cluster_value in colors:
                        color = colors[cluster_value]
                    else:
                        # each cluster should have a consistent, unique color
                        temp_color = ColorRGBA(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1), 1)
                        colors[cluster_value] = temp_color
                        color = temp_color

                    xy = self.grid_to_map_coords([j, i])
                    markers.append(self.make_sphere_marker(xy, 0.1, color))

        for center_point in self.center_points:
            markers.append(
                self.make_sphere_marker(center_point, 0.3, ColorRGBA(1, 0, 0, 1)))

        return MarkerArray(markers)

    def grid_to_map_coords(self, xy):
        x = self.resolution * xy[0] + self.origin.position.x
        y = self.resolution * xy[1] + self.origin.position.y
        return [x, y]

    def closest_centroid(self, robotXY):
        # type: ([int,int]) -> [int,int]
        lowest_distance = float('inf')
        closest_centroid = None  # type: [int,int]
        # if there are no centroids left, return none
        if len(self.center_points) == 0:
            return None
        for xy in self.center_points:
            dist = math.hypot(xy[0] - robotXY[0], xy[1] - robotXY[1])
            if dist < lowest_distance:
                lowest_distance = dist
                closest_centroid = xy
        rospy.loginfo("closest centroid is: " + str(closest_centroid[0]) + "," + str(closest_centroid[1]))
        return closest_centroid
