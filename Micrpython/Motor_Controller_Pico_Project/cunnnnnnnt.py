from machine import Pin
from utime import sleep
import utime
from umqtt.simple import MQTTClient
pin = Pin("LED", Pin.OUT)
import network
from Motors.Stepper import StepperMotor
from Motors.DCmotor import DCmotor


# Set the GPIO pin for the limit switch
limit_switch_pin = 2

# Set the pin as an input with pull-up resistor
limit_switch = Pin(limit_switch_pin, Pin.IN, Pin.PULL_UP)



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
    client = MQTTClient("pico_w_m", "192.168.8.234")  # Pi's IP address
    
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

    4.2: "data/data",
    4.3: "data/inst",
    4.4: "stepp/pos"
}
Standard_movement = {
    "active": False,
    "position": 0,
    "angle_pos": False,
    "order": 0
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
    "angle/mpu": 0
}

angles = {
    "des_angle" : 1,
    "command": 0,
    "cur_angle" : 1
}
pid_controller = {
    "integral" : 0.0,
    "derivative" : 0.0,
    "last_error" : 0.0,
    "largest_error" : 0.0,
    "k": 5.0,
    "c": 1,
    "i": 0.000,
    "d": 2.5

}

### angle motor instructions
def angle_reach():
    global angles
    global pid_controller
    global motor_status
    global angle_motor
    err = -float(angles["des_angle"]) + float(angles["cur_angle"])
    
    print("error : "+ str(err))
    print("cur_angle : "+ str(angles["cur_angle"]))
    
    motor_status["angle/status"] = 1
    
    
    pid_controller["integral"] += err
    pid_controller["derivative"] = err - pid_controller["last_error"]
    pid_controller["last_error"] = err
    
    # PID control
    output = (pid_controller["k"] * err) + (pid_controller["i"] * pid_controller["integral"]) + (pid_controller["d"] * pid_controller["derivative"])
    if (output**2)**0.5 > pid_controller["largest_error"]:
        pid_controller["largest_error"] = (output**2)**0.5
    output = 75*(output/pid_controller["largest_error"])
    if output < 0:
        output = output -25
    elif output > 0:
        output = output + 25
    # Limit the output to a valid duty cycle range
    output = max(-100, min(100, output))
    #output = -50
    print("output : " + str(output))
    # - is up 
    # + is down

    #output = max(-100, min(100, output))
    angle_motor.move(output)
    if -1.50 < err < 1.50 :
        pid_controller["c"] = pid_controller["c"] + 1
        angle_motor.move(0)
        if pid_controller["c"] > 1:
            motor_status["angle/status"] = 0
            angle_motor.stop()
            mqtt_publish(client,"angle/status","0")
            mqtt_publish(client,"angle/status", "stopped")
        # angle achieved
    else : pid_controller["c"] = 0
### angle motor instructions
def angle_reach2():
    global angles
    global pid_controller
    global motor_status
    global angle_motor
    err = -float(angles["des_angle"]) + float(angles["cur_angle"])
    
    print("error : "+ str(err))
    print("cur_angle : "+ str(angles["cur_angle"]))
    
    motor_status["angle/status"] = 1
    
    
    pid_controller["integral"] += err
    pid_controller["derivative"] = err - pid_controller["last_error"]
    pid_controller["last_error"] = err
    
    # PID control
    output = (pid_controller["k"] * err) + (pid_controller["i"] * pid_controller["integral"]) + (pid_controller["d"] * pid_controller["derivative"])
    if (output**2)**0.5 > pid_controller["largest_error"]:
        pid_controller["largest_error"] = (output**2)**0.5
    output = 75*(output/pid_controller["largest_error"])
    if output < 0:
        output = output -25
    elif output > 0:
        output = output + 25
    # Limit the output to a valid duty cycle range
    output = max(-100, min(100, output))
    #output = -50
    print("output : " + str(output))
    # - is up 
    # + is down

    #output = max(-100, min(100, output))
    angle_motor.move(output)
    if -1.50 < err < 1.50 :
        pid_controller["c"] = pid_controller["c"] + 1
        angle_motor.move(0)
        if pid_controller["c"] > 1:
            motor_status["angle/status"] = 0
            angle_motor.stop()
            mqtt_publish(client,"angle/status","0")
            mqtt_publish(client,"angle/status", "stopped")
        # angle achieved
    else : pid_controller["c"] = 0


## motor initilisations
# numbers prepresnt the pins
stepper = StepperMotor(15,14,16,17)
stepper.stop()
angle_motor = DCmotor(27,26)
linear_motor = DCmotor(27,26)
# Define a function to detect the pulse
bounce_lock = True
bounce_time = utime.ticks_ms()
def detect_pulse(pin):
    global bounce_lock
    global stepper
    global Standard_movement
    global bounce_time
    if utime.ticks_ms() > bounce_time:
        print("Pulse detected!")
        motor_status["stepp/status"] = 0
        stepper.stop()
        mqtt_publish(client,"stepp/status", "stopped")
        bounce_time = utime.ticks_ms() + 500
        if Standard_movement["active"] == True:
            Standard_movement["position"] = Standard_movement["position"] + 1
            Custom_opp()

# Set up the interrupt for the limit switch pin
limit_switch.irq(trigger=Pin.IRQ_RISING, handler=detect_pulse)

def angle_2():
    global motor_status
    while motor_status["angle/status"] ==1:
        print("check1")
        client.check_msg()
        angle_reach2()
        client.check_msg()
        utime.sleep(0.1)

def motor_control(topic,input):
    global motor_status
    global angles
    global bounce_lock
    if topic == "stepp/inst":
        bounce_lock = True
        motor_status["stepp/inst"] = 1
        mqtt_publish(client,"stepp/status", "moving")
        stepper.move(int(input))
        motor_status["stepp/status"] = 0
        mqtt_publish(client,"stepp/status", "stopped")
    elif topic == "angle/inst": 
        motor_status["angle/status"] = 1
        mqtt_publish(client,"angle/status", "moving")
        angles["des_angle"] = int(input)
    else:
        print("topic : " + topic)
        print("motor not run")
        




###init code###
wifi_connect()
client = mqtt_connect()







def Custom_opp():
    global Standard_movement
    Standard_movement["active"] = True
    # true for upper pos
    # false for lower pos
    if Standard_movement["position"] == 0:
        #mqtt_publish(client,"data/inst", "start_rec")
        motor_control("stepp/inst","-1400")
        mqtt_publish(client,"stepp/pos", "0")
        print("step1")
    elif Standard_movement["position"] == 1:
        print("int step")
    elif Standard_movement["position"] == 2:
        motor_status["stepp/position"] = 0
        mqtt_publish(client,"stepp/pos", str(motor_status["stepp/position"]))
        utime.sleep(0.1)
        motor_control("stepp/inst","213")
        motor_status["stepp/position"] = 213
    elif Standard_movement["position"] == 3:
        utime.sleep(0.1)
        motor_status["angle/status"] = 1
        angles["des_angle"] = -25
        angle_2()
        #motor_control("angle/inst","-25")
        print("step3")
    elif  Standard_movement["position"] == 4:
        utime.sleep(0.1)
        angles["des_angle"] = -25
        motor_status["angle/status"] = 1
        angle_2()
        #motor_control("angle/inst","-25")
        Standard_movement["angle_pos"] = False
        print("step4")
    elif 39> Standard_movement["position"] > 4:
        utime.sleep(0.05)
        if Standard_movement["order"] == 0 and Standard_movement["angle_pos"] == False:
            angles["des_angle"] = 0
            motor_status["angle/status"] = 1
            angle_2()
            #motor_control("angle/inst","0")
            Standard_movement["position"] = Standard_movement["position"] - 1
            Standard_movement["order"] = Standard_movement["order"] + 1
            Standard_movement["angle_pos"] = True
            print("true")
        elif Standard_movement["order"] == 0 and Standard_movement["angle_pos"] == True:
            angles["des_angle"] = -25
            motor_status["angle/status"] = 1
            angle_2()
            #motor_control("angle/inst","-25")
            Standard_movement["position"] = Standard_movement["position"] - 1
            Standard_movement["order"] = Standard_movement["order"] + 1
            Standard_movement["angle_pos"] = False
            print("false")
        elif Standard_movement["order"] == 1:
            motor_control("stepp/inst","10")
            motor_status["stepp/position"] = motor_status["stepp/position"] + 10
            mqtt_publish(client,"stepp/pos", str(motor_status["stepp/position"]))
            Standard_movement["order"] = 0
    else:
        utime.sleep(0.1)
        ## need to convert my data_packet into a csv file
        mqtt_publish(client,"data/inst", "stop_rec")
        Standard_movement["position"] = 0
        Standard_movement["active"] = False









# Define the callback function for receiving messages
def sub_cb(topic, msg):
    global motor_status
    global angles
    #print(msg.decode())
    ## a handle manager for when the pi requests infromation about a motors status on/off
    if topic.decode() == "angle/mpu" :
        #print(f"Received message: {msg.decode()} on topic {topic.decode()}")
        # need to crewate a global varibe-l called current angle
        angles["cur_angle"] = float(msg.decode())
        print("current angle : "+ str(angles["cur_angle"]))
    else:
        if msg.decode() == "req" and topic.decode() in motor_status.keys() and motor_status["angle/status"] == 0:
            #print(f"Received message: {msg.decode()} on topic {topic.decode()}")
            mqtt_publish(client,topic.decode(),str(motor_status[topic.decode()]))
        if topic.decode() in motor_inst.keys() and motor_status["angle/status"] == 0:
            print(f"Received message: {msg.decode()} on topic {topic.decode()}")
            motor_control(topic.decode(),msg.decode())
        if topic.decode() == "data/inst" and msg.decode() == "start_rec" and motor_status["angle/status"] == 0:
            print("started")
            Custom_opp()



# Subscribe to the topic
client.set_callback(sub_cb)
for value in topic_dict.values():
    client.subscribe(value)


## main code
print("LED starts flashing...")
while True:
    if Standard_movement["active"] == True and motor_status["angle/status"] == 0:
        Standard_movement["position"] = Standard_movement["position"] + 1
        print(str(Standard_movement["position"]))
        Custom_opp()
    else:
        client.check_msg()
        if motor_status["angle/status"] == 1:
            angle_reach()
    try:
        pin.toggle()
        client.check_msg()
        #mqtt_publish(client,topic,message)
        sleep(0.005) # sleep 1sec
    except KeyboardInterrupt:
        angle_motor.stop()
        break

###off code###
mqtt_disconnect(client)
pin.off()
print("Finished.")