from machine import Pin
from utime import sleep
from umqtt.simple import MQTTClient
pin = Pin("LED", Pin.OUT)
import network
from Motors.Stepper import StepperMotor
from Motors.DCmotor import DCmotor


###varibles#'###
topic = b"stepp/status"
message = b"message"

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
        sleep(1)
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

def mqtt_subscribe(client,topic,message):
    # Publish a message
    client.publish(topic, message)

### global varibles 
topic_dict = {
    1.0: "stepp/",
    1.1: "stepp/inst",
    1.2: "stepp/status",


    2.0: "angle/",
    2.1: "angle/inst",
    2.2: "angle/status",
    2.3: "angle/mpu",

    3.0: "linear/",
    3.1: "linear/inst",
    3.2: "linear/status",
    3.3: "linear/encoder",

    4.0: "hall/reading"
}

motor_status ={
    "stepp/status": 0,
    "angle/status": 0,
    "linear/status": 0
}

motor_inst ={
    "stepp/inst": 0,
    "angle/inst": 0,
    "linear/inst": 0
}
data_dict = {
    "angle/mpu": 0,
    "hall/reading": 0
}

angles = {
    "des_angle" : 0,
    "command": 0,
    "cur_angle" : 0
}

### angle motor instructions
def angle_reach():
    global angles
    global motor_status
    err = ((angles["des_angle"]^2)^0.5 - (angles["cur_angle"]^2)^0.5)
    if err > 0.5 or err < -0.5:
        motor_status["angle/inst"] = 0
        angle_motor.stop()
        # angle achieved
    
    integral += err
    derivative = err - last_error
    last_error = err

    # PID control
    output = (1.0 * err) + (1.0 * integral) + (1.0 * derivative)

    # Limit the output to a valid duty cycle range
    output = max(0, min(100, output))
    angle_motor.move(output)



## motor initilisations
# numbers prepresnt the pins
stepper = StepperMotor(15,14,16,17)
stepper.stop()
angle_motor = DCmotor(27,26)
linear_motor = DCmotor(27,26)

def motor_control(topic,input):
    global motor_status
    global angles
    if topic == "stepp/inst":
        motor_status["stepp/inst"] = 1
        stepper.move(int(input))
        motor_status["stepp/inst"] = 0
    elif topic == "angle/inst":
        motor_status["angle/inst"] = 1
        angles["des_angle"] = input
    else:
        print(topic)
        print("motor not run")




###init code###
wifi_connect()
client = mqtt_connect()



# Define the callback function for receiving messages
def sub_cb(topic, msg):
    global motor_status
    print(f"Received message: {msg.decode()} on topic {topic.decode()}")
    ## a handle manager for when the pi requests infromation about a motors status on/off
    if msg.decode() == "req" and topic.decode() in motor_status.keys():
        mqtt_publish(client,topic.decode(),str(motor_status[topic.decode()]))
    if topic.decode() in motor_inst.keys():
        motor_control(topic.decode(),msg.decode())
    if topic.decode() in data_dict.keys():
        # need to crewate a global varibe-l called current angle
        print("datastor()")

    


# Subscribe to the topic
client.set_callback(sub_cb)



## main code
print("LED starts flashing...")
while True:
    try:
        pin.toggle()
        for value in topic_dict.values():
            client.subscribe(value)
        #mqtt_publish(client,topic,message)
        if motor_inst["stepp/inst"] == 1:
            angle_reach()
        sleep(0.2) # sleep 1sec
    except KeyboardInterrupt:
        angle_motor.stop()
        break

###off code###
mqtt_disconnect(client)
pin.off()
print("Finished.")