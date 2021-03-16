import geometry_msgs.msg
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class DrawingTools:

    def __init__(self):
        self.mark_id = 1

    def make_sphere_marker(self, xy, radius, color, namespace):
        # type: ([int,int], float, ColorRGBA, str) -> Marker
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
        marker.ns = namespace
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
