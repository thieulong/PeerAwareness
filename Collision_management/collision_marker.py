from visualisation_msgs.msg import Marker
from rclpy.time import Time   

class VisualiseCollision():

    collsion_pub = self.create_publisher(Marker, "visualization_marker", 1)
    collison = Marker()
    collision.header.frame_id = "/map"
    collision.header.stamp = node.get_clock().now()
    collision.ns = "basic_shapes"
    collision.id = 0


    collision.type = visualization_msgs.msg.marker.SPHERE

    collision.action = visualization_msgs.msg.marker.ADD

    collision.pose.position.x = 0
    collision.pose.position.y = 0
    collision.pose.position.z = 0
    collision.pose.orientation.x = 0.0
    collision.pose.orientation.y = 0.0
    collision.pose.orientation.z = 0.0
    collision.pose.orientation.w = 1.0

    collision.scale.x = 1.0
    collision.scale.y = 1.0
    collision.scale.z = 1.0

    collision.color.r = 0.0f
    collision.color.g = 1.0f
    collision.color.b = 0.0f
    collision.color.a = 1.0f 

    collision_pub.publish(collision)

