import paho.mqtt.client as mqtt
import time

# MQTT settings
BROKER = "mqtt.eclipseprojects.io"  # Replace with your broker's address
PORT = 1883  # Default MQTT port
TOPIC = "test/topic"  # Replace with your desired topic

# Callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC)  # Subscribe to the topic
    else:
        print(f"Failed to connect, return code {rc}")

# Callback for when a message is received from the server.
def on_message(client, userdata, msg):
    print(f"Received message: {msg.payload.decode()} on topic: {msg.topic}")

# Publisher function
def publish(client):
    while True:
        message = input("Enter a message to publish (or 'exit' to quit): ")
        if message.lower() == "exit":
            break
        client.publish(TOPIC, message)
        print(f"Published: {message} to topic: {TOPIC}")

# Main function
if __name__ == "__main__":
    # Subscriber client setup
    subscriber_client = mqtt.Client()
    subscriber_client.on_connect = on_connect
    subscriber_client.on_message = on_message

    # Connect the subscriber client to the broker
    subscriber_client.connect(BROKER, PORT, 60)

    # Run the subscriber in a separate thread
    subscriber_client.loop_start()

    # Publisher client setup
    publisher_client = mqtt.Client()
    publisher_client.connect(BROKER, PORT, 60)

    # Start publishing messages
    try:
        publish(publisher_client)
    except KeyboardInterrupt:
        print("\nExiting...")

    # Stop the subscriber loop and disconnect
    subscriber_client.loop_stop()
    subscriber_client.disconnect()
    publisher_client.disconnect()

