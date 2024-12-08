import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from rclpy.service import Service
from std_srvs.srv import SetBool
import math
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class StretchNavigation(Node):
    def __init__(self):
        super().__init__('waypoints_navigation')

        # Create publisher to cmd_vel only
        self.velocity_publisher = self.create_publisher(Twist, '/stretch/cmd_vel', 10)

        # Create subscribers
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.update_odometry_pose, 10)
        self.plan_subscriber = self.create_subscription(Path, '/plan', self.plan_callback, 10)

        # Create new service for stop signal
        self.stop_signal_service = self.create_service(SetBool, '/stop_signal', self.stop_signal_callback)

        self.rate = self.create_rate(10)

        # Initialize variables
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.global_x = 0.0
        self.global_y = 0.0
        self.global_yaw = 0.0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.linear_tolerance = 0.5
        self.angular_tolerance = 0.035

        self.stop_signal = False  # Flag for stop signal
        self.waypoints = []  # List to hold the waypoints
        self.current_waypoint_idx = 0  # Index of the current waypoint being navigated to

    def update_odometry_pose(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.odom_x = position.x
        self.odom_y = position.y

        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, self.odom_yaw = euler_from_quaternion(quaternion)

        self.update_global_pose()

    def update_global_pose(self):
        odom_pose = PoseStamped()
        odom_pose.header.frame_id = "odom"
        odom_pose.pose = Pose(
            position=Point(self.odom_x, self.odom_y, 0.0),
            orientation=Quaternion(*quaternion_from_euler(0, 0, self.odom_yaw))
        )

        try:
            global_pose = self.tf_buffer.transform(odom_pose, "map")
            self.global_x = global_pose.pose.position.x
            self.global_y = global_pose.pose.position.y
            _, _, self.global_yaw = euler_from_quaternion(
                (global_pose.pose.orientation.x, global_pose.pose.orientation.y,
                 global_pose.pose.orientation.z, global_pose.pose.orientation.w)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("TF lookup failed")

    def plan_callback(self, msg):
        # Extract and filter waypoints
        self.waypoints = self.filter_waypoints(msg.poses)
        self.current_waypoint_idx = 0
        self.navigate_to_next_waypoint()

    def stop_signal_callback(self, request, response):
        # Handle the stop signal
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
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        waypoint = self.waypoints[self.current_waypoint_idx]
        goal_x = waypoint.pose.position.x
        goal_y = waypoint.pose.position.y

        goal_angle = math.atan2(goal_y - self.global_y, goal_x - self.global_x)
        angle_error = self.normalize_angle(goal_angle - self.global_yaw)

        # Turn towards the goal angle
        while abs(angle_error) >= self.angular_tolerance:
            if self.stop_signal:
                self.stop_robot()
                return
            velocity_msg = Twist()
            angular_speed = 0.5 * angle_error
            velocity_msg.angular.z = angular_speed
            self.velocity_publisher.publish(velocity_msg)
            self.update_global_pose()
            angle_error = self.normalize_angle(goal_angle - self.global_yaw)
            self.rate.sleep()

        # Stop turning and move towards the goal position
        velocity_msg = Twist()
        velocity_msg.angular.z = 0
        self.velocity_publisher.publish(velocity_msg)

        while math.sqrt((goal_x - self.global_x) ** 2 + (goal_y - self.global_y) ** 2) >= self.linear_tolerance:
            if self.stop_signal:
                self.stop_robot()
                return
            velocity_msg = Twist()
            velocity_msg.linear.x = 0.15  # Constant linear speed
            self.velocity_publisher.publish(velocity_msg)
            self.update_global_pose()
            self.rate.sleep()

        # Stop after reaching the waypoint
        velocity_msg = Twist()
        velocity_msg.linear.x = 0
        self.velocity_publisher.publish(velocity_msg)

        # Move to the next waypoint
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
        """Filter out unnecessary intermediate waypoints."""
        filtered_waypoints = []
        if len(waypoints) < 2:
            return waypoints

        filtered_waypoints.append(waypoints[0])  # Always keep the first waypoint
        for i in range(1, len(waypoints) - 1):
            prev_wp = waypoints[i - 1]
            curr_wp = waypoints[i]
            next_wp = waypoints[i + 1]

            # Calculate the distance between the points
            distance_curr_next = math.sqrt((next_wp.pose.position.x - curr_wp.pose.position.x) ** 2 +
                                          (next_wp.pose.position.y - curr_wp.pose.position.y) ** 2)
            distance_prev_curr = math.sqrt((curr_wp.pose.position.x - prev_wp.pose.position.x) ** 2 +
                                           (curr_wp.pose.position.y - prev_wp.pose.position.y) ** 2)

            # Calculate the angle difference between consecutive waypoints
            angle_diff = math.atan2(curr_wp.pose.position.y - prev_wp.pose.position.y,
                                    curr_wp.pose.position.x - prev_wp.pose.position.x)

            # If distance is small and angle difference is small, skip the intermediate point
            if distance_curr_next < 0.5 and abs(angle_diff) < 0.1:  # Small enough to skip
                continue

            filtered_waypoints.append(curr_wp)

        filtered_waypoints.append(waypoints[-1])  # Always keep the last waypoint
        return filtered_waypoints


def main(args=None):
    rclpy.init(args=args)
    node = StretchNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
