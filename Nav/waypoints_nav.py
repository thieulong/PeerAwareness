import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from rclpy.service import Service
from std_srvs.srv import SetBool
import math
import tf2_ros
from tf_transformations import euler_from_quaternion

class WaypointsNavigation(Node):
    def __init__(self):
        super().__init__('waypoints_navigation')

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.update_odometry_pose, 10)
        self.plan_subscriber = self.create_subscription(Path, '/plan', self.plan_callback, 10)

        self.stop_signal_service = self.create_service(SetBool, '/stop_signal', self.stop_signal_callback)

        self.rate = self.create_rate(10)

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.linear_tolerance = 0.2
        self.angular_tolerance = 0.035

        self.stop_signal = False  
        self.waypoints = []  
        self.current_waypoint_idx = 0  

    def update_odometry_pose(self, msg):
        """Update odometry position from /odom topic"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.odom_x = position.x
        self.odom_y = position.y

        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, self.odom_yaw = euler_from_quaternion(quaternion)

    def plan_callback(self, msg):
        """Callback when a new plan (path) is received"""
        self.get_logger().info(f"Received {len(msg.poses)} waypoints from /plan topic.")

        filtered_waypoints = self.filter_waypoints(msg.poses)
        self.get_logger().info(f"Filtered waypoints count: {len(filtered_waypoints)}")

        self.waypoints = filtered_waypoints
        self.current_waypoint_idx = 0
        self.navigate_to_next_waypoint()

    def stop_signal_callback(self, request, response):
        """Stop or resume the robot's movement"""
        self.stop_signal = request.data
        if self.stop_signal:
            self.get_logger().info("Received stop signal, stopping robot.")
            self.stop_robot()
        else:
            self.get_logger().info("Received resume signal, resuming navigation.")
            self.navigate_to_next_waypoint()

        response.success = True
        return response

    def navigate_to_next_waypoint(self):
        """Navigate the robot to the next waypoint"""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        waypoint = self.waypoints[self.current_waypoint_idx]
        goal_x = waypoint.pose.position.x
        goal_y = waypoint.pose.position.y

        goal_angle = math.atan2(goal_y - self.odom_y, goal_x - self.odom_x)
        angle_error = self.normalize_angle(goal_angle - self.odom_yaw)

        while abs(angle_error) >= self.angular_tolerance:
            if self.stop_signal:
                self.stop_robot()
                return
            velocity_msg = Twist()
            angular_speed = 0.5 * angle_error
            velocity_msg.angular.z = angular_speed
            self.velocity_publisher.publish(velocity_msg)
            self.update_odometry_pose()
            angle_error = self.normalize_angle(goal_angle - self.odom_yaw)
            self.rate.sleep()

        velocity_msg = Twist()
        velocity_msg.angular.z = 0
        self.velocity_publisher.publish(velocity_msg)

        while math.sqrt((goal_x - self.odom_x) ** 2 + (goal_y - self.odom_y) ** 2) >= self.linear_tolerance:
            if self.stop_signal:
                self.stop_robot()
                return
            velocity_msg = Twist()
            velocity_msg.linear.x = 0.15 
            self.velocity_publisher.publish(velocity_msg)
            self.update_odometry_pose()
            self.rate.sleep()

        velocity_msg = Twist()
        velocity_msg.linear.x = 0
        self.velocity_publisher.publish(velocity_msg)

        self.current_waypoint_idx += 1
        self.navigate_to_next_waypoint()

    def stop_robot(self):
        """Stop the robot immediately."""
        velocity_msg = Twist()
        self.velocity_publisher.publish(velocity_msg)

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def filter_waypoints(self, waypoints):
        """Filter waypoints based on angle difference"""
        filtered_waypoints = []
        if len(waypoints) < 2:
            return waypoints

        filtered_waypoints.append(waypoints[0])
        for i in range(1, len(waypoints) - 1):
            prev_wp = waypoints[i - 1]
            curr_wp = waypoints[i]
            next_wp = waypoints[i + 1]

            prev_to_curr_angle = math.atan2(curr_wp.pose.position.y - prev_wp.pose.position.y,
                                            curr_wp.pose.position.x - prev_wp.pose.position.x)
            curr_to_next_angle = math.atan2(next_wp.pose.position.y - curr_wp.pose.position.y,
                                            next_wp.pose.position.x - curr_wp.pose.position.x)

            angle_diff = abs(curr_to_next_angle - prev_to_curr_angle)

            if angle_diff > 0.1:  
                filtered_waypoints.append(curr_wp)

        filtered_waypoints.append(waypoints[-1])  
        return filtered_waypoints


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
