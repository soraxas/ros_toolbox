import rospy
from visualization_msgs.msg import Marker, MarkerArray

pts_id = 0


def create_marker_array(
        pts, radius=0.05, color_msg=None, frame_id="world", duration=1000
):
    """Given [n x 3] 3D points, return a marker array with unique ids."""
    global pts_id

    markers = MarkerArray()
    for pt in pts:
        marker = Marker(
            type=Marker.SPHERE,
            # ns='velodyne',
            action=Marker.ADD,
        )
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()

        marker.id = pts_id
        pts_id += 1

        marker.scale.x = marker.scale.y = marker.scale.z = radius
        marker.lifetime = rospy.Duration.from_sec(duration)
        if color_msg is None:
            marker.color.a = 1.0
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
        else:
            marker.color = color_msg

        marker.pose.position.x = pt[0]
        marker.pose.position.y = pt[1]
        marker.pose.position.z = pt[2]
        marker.pose.orientation.w = 1.0

        markers.markers.append(marker)
    return markers
