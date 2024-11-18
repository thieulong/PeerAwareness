import paho.mqtt.client as mqtt
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
        self.stop_publishing = False  # Indicates if we are in stop mode

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect(BROKER, PORT, 60)
        self.mqtt_client.loop_start()

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
            self.get_logger().info("Publishing stop signal to /cmd_vel...")
            self.stop_publishing = True
            self.publish_stop_signal()

        elif command == "go":
            self.get_logger().info("Removing stop signal, robot can resume...")
            self.stop_publishing = False

    def publish_stop_signal(self):
        # Publish a zero velocity command when stop is received
        while self.stop_publishing:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher.publish(stop_msg)
            self.get_logger().info("Stop signal published to /cmd_vel.")
            rclpy.spin_once(self, timeout_sec=1.0)  # Allow other callbacks to run

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
