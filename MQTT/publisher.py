import paho.mqtt.client as mqtt

# MQTT settings
BROKER = "mqtt.eclipseprojects.io"  # Replace with your broker address
PORT = 1883  # Default MQTT port
TOPIC = "test/topic"  # Replace with your desired topic

# Publisher function
def publish():
    client = mqtt.Client()
    client.connect(BROKER, PORT, 60)

    while True:
        message = input("Enter a message to publish (or 'exit' to quit): ")
        if message.lower() == "exit":
            break
        client.publish(TOPIC, message)
        print(f"Published: '{message}' to topic '{TOPIC}'")
    
    client.disconnect()
    print("Disconnected from the broker.")

if __name__ == "__main__":
    try:
        publish()
    except KeyboardInterrupt:
        print("\nExiting...")


