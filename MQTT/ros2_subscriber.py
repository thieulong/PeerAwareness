kimport paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# MQTT settings
BROKER = "mqtt.eclipseprojects.io"  # Replace with your broker address
PORT = 1883  # Default MQTT port
TOPIC = "robot/control"  # Replace with your desired MQTT topic

class MQTTToROS2(Node):
    def __init__(self):
        super().__init__("mqtt_to_ros2")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.movement_active = True  # Start with movement active

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to MQTT broker
        self.mqtt_client.connect(BROKER, PORT, 60)
        self.mqtt_client.loop_start()

        # Start the robot moving at 0.5 m/s
        self.get_logger().info("Starting robot with 0.5 m/s speed.")
        self.publish_movement(0.5)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker!")
            client.subscribe(TOPIC)
            self.get_logger().info(f"Subscribed to topic: {TOPIC}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, return code {rc}")

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode().strip().lower()
        self.get_logger().info(f"Received MQTT message: '{command}'")

        if command == "stop":
            self.get_logger().info("Stopping the robot...")
            self.movement_active = False
            self.publish_movement(0.0)  # Send stop signal

        elif command == "go":
            self.get_logger().info("Resuming the robot's movement at 0.5 m/s...")
            self.movement_active = True
            self.publish_movement(0.5)  # Resume movement

    def publish_movement(self, speed):
        """
        Continuously publish the given speed as a Twist message
        while the current mode (movement_active) is true.
        """
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = 0.0

        # Publish continuously while the state is active
        while rclpy.ok() and ((self.movement_active and speed > 0) or (not self.movement_active and speed == 0)):
            self.publisher.publish(twist_msg)
            self.get_logger().info(f"Publishing: linear.x={speed}, angular.z=0.0")
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow callback handling

def main(args=None):
    rclpy.init(args=args)
    node = MQTTToROS2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.mqtt_client.loop_stop()
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

