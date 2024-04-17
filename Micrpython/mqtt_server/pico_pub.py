from machine import Pin
from utime import sleep
from umqtt.simple import MQTTClient
pin = Pin("LED", Pin.OUT)
import network

###varibles#'###
topic = b"test/topic"
message = b"Hello from Pico W!"

### functions###
def wifi_connect():
    # Set up the WiFi connection
    ssid = 'GL-MT300N-V2-0c2'
    password = 'goodlife'

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)

    # Wait for the connection to be established
    while not wlan.isconnected():
        print(".")
        sleep(300)
        pass

    print('Connected to', ssid)
    print('Network config:', wlan.ifconfig())

def mqtt_connect():
    # Connect to MQTT broker
    client = MQTTClient("pico_w", "192.168.8.234")  # Pi's IP address
    client.connect()
    return client

def mqtt_disconnect(client):
    client.disconnect()

def mqtt_publish(client,topic,message):
    # Publish a message
    client.publish(topic, message)

###init code###
wifi_connect()
client = mqtt_connect()

## main code
print("LED starts flashing...")
while True:
    try:
        pin.toggle()
        mqtt_publish(client,topic,message)
        sleep(0.1) # sleep 1sec
    except KeyboardInterrupt:
        break

###off code###
mqtt_disconnect(client)
pin.off()
print("Finished.")