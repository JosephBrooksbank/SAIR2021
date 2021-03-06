#!/usr/bin/env python2
import random
import rospy
import actionlib
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs.msg
import nav_msgs.msg
from nav_msgs.msg import OccupancyGrid
import numpy
from sklearn.cluster import AgglomerativeClustering
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

# Globals
cur_map = []
new_map = []
map_post = None
marker_post = None
# Default res of 0.05, updated in map callback
resolution = 0.05


# HELPER METHODS

def generate_goal(x, y, w):
    """ Generate a goal position and orientation inside a MoveBaseGoal() Object """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w
    return goal


def generate_neighbors(cur_map_2d, i, j):
    """ Get the value of neighboring values in a 2D Array.

    Arguments:
        cur_map_2d: the map to get values from
        i,j: the coordinates whos neighbors should be given

    returns: center, left, right, up, down, as integer values corresponding to that position
    """
    width = cur_map_2d.shape[0]
    height = cur_map_2d.shape[1]

    center = cur_map_2d[i, j]
    if i == 0:
        left = cur_map_2d[i, j]
    else:
        left = cur_map_2d[i - 1, j]
    if i == width - 1:
        right = cur_map_2d[i, j]
    else:
        right = cur_map_2d[i + 1, j]
    if j == 0:
        up = cur_map_2d[i, j]
    else:
        up = cur_map_2d[i, j - 1]
    if j == height - 1:
        down = cur_map_2d[i, j]
    else:
        down = cur_map_2d[i, j + 1]

    return center, left, right, up, down


def fill_square(cur_map_2d, center, radius, value):
    """ Fills a square of the value given at the center and radius given, in the 2D array given. """
    width = cur_map_2d.shape[0]
    height = cur_map_2d.shape[1]
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
            cur_map_2d[i, j] = value


def make_sphere(x, y, radius, color, mark_id, off_x, off_y):
    """ Function to generate a spherical marker

        ARGUMENTS:
            x, y: coordinates of point in grid space
            radius: float
            color: ColorRGBA
            mark_id: unique id
            off_x, off_y: Origin of map coordinates in world frame """
    global resolution
    marker = Marker()

    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = 'map'
    marker.ns = 'frontier'
    marker.id = mark_id
    marker.type = 2
    marker.action = 0
    marker.pose = geometry_msgs.msg.Pose()

    marker.pose.position.x = resolution * x + off_x
    marker.pose.position.y = resolution * y + off_y
    marker.scale = geometry_msgs.msg.Vector3(radius, radius, radius)
    marker.color = color
    marker.lifetime = rospy.rostime.Duration(0)
    marker.frame_locked = True
    return marker


# MAP GENERATORS FOR DIFFERENT FRAMES

def generate_obstacle_map(cur_map_2d):
    """ Creates a map with space around all known obstacles on map, based on robot width"""
    width = cur_map_2d.shape[0]
    height = cur_map_2d.shape[1]
    grown_map = numpy.copy(cur_map_2d)

    # https://www.robotis.us/turtlebot-3-waffle-pi/ puts the waffle pi at 281x306mm,
    # looking manually at the resolution of the map data compared to the visible dimensions of the robot,
    # I would say that the robot is ~5 squares.
    robot_width = 5
    for i in range(width):
        for j in range(height):
            if cur_map_2d[i, j] == 100:
                fill_square(grown_map, [i, j], robot_width, 100)
    return grown_map


def generate_frontier_map(cur_map_2d):
    """ Generates a frontier map, based on the relationship between known and unknown space """
    width = cur_map_2d.shape[0]
    height = cur_map_2d.shape[1]
    frontier_map = numpy.zeros(shape=(width, height))

    # for every item in map, if unknown and next to a known, it is a frontier
    for i in range(width):
        for j in range(height):
            center, left, right, up, down = generate_neighbors(cur_map_2d, i, j)

            if center == -1:
                if left == 0 or right == 0 or up == 0 or down == 0:
                    frontier_map[i, j] = 100
                else:
                    frontier_map[i, j] = 0
    return frontier_map


def generate_cluster_map(frontier_map):
    """Takes a frontier map, and runs a clustering algorithm on it to determine which frontiers are connected."""
    width = frontier_map.shape[0]
    height = frontier_map.shape[1]
    # Empty map to store cluster data in-- 0 means no data, any positive integer is the index of the cluster.
    clustering_map = numpy.zeros(shape=(width, height))

    # Creating an array of frontier locations as [x,y] pairs for cluster algorithm
    point_array = []
    for i in range(width):
        for j in range(height):
            if frontier_map[i, j] != 0:
                point_array.append([i, j])

    # through experimentation, this was the best way to cluster
    cluster_values = AgglomerativeClustering(n_clusters=4, ).fit_predict(point_array)

    # There is a cluster labeled "0" by default, which is also my empty space. This caused
    # issues for an embarrassingly long time.
    for i in range(len(cluster_values)):
        cluster_values[i] += 1

    # For centroid calculation, the [x,y] pairs of each point, grouped by cluster index, must be stored.
    # TODO generate middle points here? Not sure
    cluster_groups = {}

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

        clustering_map[coords[0], coords[1]] = cur_value

    return clustering_map, cluster_groups


def place_markers(cluster_map, origin, cluster_groups):
    """places markers in map space based on clusters in grid space, with centroids visible."""

    # Clearing canvas
    delete_message = Marker()
    delete_message.action = 3  # Action 3: Delete All
    marker_post.publish([delete_message])

    width = cluster_map.shape[0]
    height = cluster_map.shape[1]

    colors = {}
    markers = []
    global marker_post
    global resolution

    # unique identifier for each sphere marker
    mark_id = 1
    for i in range(width):
        for j in range(height):
            cluster_value = cluster_map[i, j]
            if cluster_value != 0:
                if cluster_value in colors:
                    color = colors[cluster_value]
                else:
                    # each cluster should have a consistent, unique color
                    temp_color = ColorRGBA(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1), 1)
                    colors[cluster_value] = temp_color
                    color = temp_color

                markers.append(make_sphere(j, i, 0.1, color, mark_id, origin.position.x, origin.position.y))
                mark_id += 1

    # Calculating centroid based on algorithm from class
    for clusterID in cluster_groups:
        x_c = y_c = count = 0
        for coord_pair in cluster_groups[clusterID]:
            x_c += coord_pair[0]
            y_c += coord_pair[1]
            count += 1
        x_c = x_c / count
        y_c = y_c / count

        markers.append(make_sphere(y_c, x_c, 0.3, ColorRGBA(1, 0, 0, 1), mark_id, origin.position.x, origin.position.y))
        mark_id += 1

    marker_post.publish(MarkerArray(markers))
    rospy.loginfo("published markers")


def map_to_frontier_map_callback(data):
    """ Callback function for the /map topic subscriber, to generate frontier maps"""
    global new_map
    global cur_map
    global resolution
    global map_post
    new_map = data.data

    # Checking for updates
    if new_map != cur_map:
        cur_map = new_map
        width = data.info.width
        height = data.info.height
        resolution = data.info.resolution
        # Reshape to 2D Array
        cur_map_2d = numpy.reshape(cur_map, (width, height))
        # Add cspace around obstacles
        map_with_obstacles = generate_obstacle_map(cur_map_2d)
        # Generate frontiers
        frontier_map = generate_frontier_map(map_with_obstacles)
        # Generating map with clusters labeled
        cluster_map, cluster_groups = generate_cluster_map(frontier_map)

        # Posting frontiers to topic
        occu_grid = OccupancyGrid()
        occu_grid.header.frame_id = "map"
        occu_grid.header.stamp = rospy.Time.now()

        occu_grid.info.width = width
        occu_grid.info.height = height
        occu_grid.info.resolution = data.info.resolution
        occu_grid.info.origin = data.info.origin
        occu_grid.info.map_load_time = rospy.Time.now()

        occu_grid.data = frontier_map.flatten()
        map_post.publish(occu_grid)

        # Publishing markers based on cluster map
        place_markers(cluster_map, data.info.origin, cluster_groups)


if __name__ == '__main__':
    rospy.init_node('project_2', anonymous=True)

    # Publishing bot position
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    post = rospy.Publisher('turtle_position', geometry_msgs.msg.TransformStamped, queue_size=10)
    rate = rospy.Rate(10.0)

    # Simple Action Client for moving to position
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Subscriber to map topic
    global map_post
    map_post = rospy.Publisher('/frontiers_map', nav_msgs.msg.OccupancyGrid, queue_size=10)
    rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, map_to_frontier_map_callback)

    global marker_post
    marker_post = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=10)

    # FROM PART 1: Not currently in use
    # first_goal = generate_goal(-1.0, 1.0, 1.0)
    # client.send_goal(first_goal)
    # client.wait_for_result()
    # second_goal = generate_goal(-1.0, -1.0, 1.0)
    # client.send_goal(second_goal)
    # client.wait_for_result()

    while not rospy.is_shutdown():
        # noinspection PyBroadException
        try:
            trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())

        except:
            rate.sleep()
            continue

        post.publish(trans)
        rate.sleep()
