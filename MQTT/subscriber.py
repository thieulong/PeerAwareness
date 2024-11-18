import paho.mqtt.client as mqtt

# MQTT settings
BROKER = "mqtt.eclipseprojects.io"  # Replace with your broker address
PORT = 1883  # Default MQTT port
TOPIC = "test/topic"  # Replace with your desired topic

# Callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC)
        print(f"Subscribed to topic: {TOPIC}")
    else:
        print(f"Failed to connect, return code {rc}")

# Callback for when a message is received from the server.
def on_message(client, userdata, msg):
    print(f"Received message: {msg.payload.decode()} on topic: {msg.topic}")

def subscribe():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(BROKER, PORT, 60)

    try:
        print("Listening for messages...")
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nExiting...")
        client.disconnect()
        print("Disconnected from the broker.")

if __name__ == "__main__":
    subscribe()

