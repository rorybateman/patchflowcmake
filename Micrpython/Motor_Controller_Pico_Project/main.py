import time
from umqtt.simple import MQTTClient
from machine import Pin
import sys
import utime
import network
from Motors.DCmotor import DCmotor
from Motors.Stepper import StepperMotor
pin = Pin("LED", Pin.OUT)

"""Defining the callback pin"""
# Set the GPIO pin for the limit switch
limit_switch_pin = 1

# Set the pin as an input with pull-up resistor
limit_switch = Pin(limit_switch_pin, Pin.IN, Pin.PULL_UP)
bounce_time = time.ticks_ms()

def detect_pulse(pin):
    global stepper
    global Standard_movement
    global bounce_time
    if utime.ticks_ms() > bounce_time:
        motor_status["stepp/status"] = 0
        stepper.stop()
        bounce_time = utime.ticks_ms() + 500
        if Standard_movement["active"] == True:
            Standard_movement["position"] = Standard_movement["position"] + 1

limit_switch.irq(trigger=Pin.IRQ_FALLING, handler=detect_pulse)
angle_motor = DCmotor(27,26)
stepper = StepperMotor(15,14,16,17)
stepper.stop()
angle_motor.stop()
# Test reception e.g. with:
# mosquitto_sub -t foo_topic
angles = {
    "des_angle" : -25,
    "command": 0,
    "cur_angle" : 1
}

pid_controller = {
    "integral" : 0.0,
    "derivative" : 0.0,
    "last_error" : 0.0,
    "largest_error" : 0.01,
    "k": 5.0,
    "c": 1,
    "i": 0.000,
    "d": 2.5
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
# Publish test messages e.g. with:
# mosquitto_pub -t foo_topic -m hello
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
        time.sleep(1)
        pass

    print('Connected to', ssid)
    print('Network config:', wlan.ifconfig())


# Define the callback function for receiving messages
def sub_cb(topic, msg):
    global motor_status
    global angles
    global c
    #print(msg)
    ## a handle manager for when the pi requests infromation about a motors status on/off
    if topic == b"angle/mpu" and motor_status["angle/status"] == 1:
        #print(f"Received message: {msg.decode()} on topic {topic.decode()}")
        # need to crewate a global varibe-l called current angle
        angles["cur_angle"] = float(msg.decode())
        print("current angle : "+ str(angles["cur_angle"]))
    if topic == b"genral/inst" and msg == b"start":
        c.publish("data/inst","start_rec")
        c.publish("stepp/pos",str(Standard_movement["position"]))
        Standard_movement["position"] = 0
        Standard_movement["active"] = True

def angle_reach():
    global angles
    global pid_controller
    global motor_status
    global angle_motor
    motor_status["angle/status"] = 1
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
    output = 60*(output/pid_controller["largest_error"])
    #should be 75 rather than 30
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
            print("angle_reached")
        # angle achieved
    else : pid_controller["c"] = 0
### angle motor instructions


def Custom_opp():
    global Standard_movement
    global bounce_time
    Standard_movement["active"] = True
    # true for upper pos
    # false for lower pos
    if Standard_movement["position"] == 0:
        motor_status["stepp/status"] = 1
        stepper.move(-1400)
        motor_status["stepp/status"] = 0
        print("step1")
        bounce_time = utime.ticks_ms() + 30000
    elif Standard_movement["position"] == 1:
        print("int step")
    elif Standard_movement["position"] == 2:
        motor_status["stepp/position"] = 0
        motor_status["stepp/status"] = 1
        stepper.move(213)
        motor_status["stepp/status"] = 0
        print("step1")
        motor_status["stepp/position"] = 213
        c.publish("stepp/pos",str(motor_status["stepp/position"]))
    elif Standard_movement["position"] == 3:
        motor_status["angle/status"] = 1
        angles["des_angle"] = -25
        print("step3")
    elif  Standard_movement["position"] == 4:
        motor_status["angle/status"] = 1
        angles["des_angle"] = 0
        print("step4")
        Standard_movement["angle_pos"] = True
    elif 74> Standard_movement["position"] > 4:
        if Standard_movement["order"] == 0 and Standard_movement["angle_pos"] == False:
            motor_status["angle/status"] = 1
            angles["des_angle"] = 0
            print("step4")
            Standard_movement["position"] = Standard_movement["position"] - 1
            Standard_movement["order"] = Standard_movement["order"] + 1
            Standard_movement["angle_pos"] = True
            print("true")
        elif Standard_movement["order"] == 0 and Standard_movement["angle_pos"] == True:
            motor_status["angle/status"] = 1
            angles["des_angle"] = -25
            print("step4")
            Standard_movement["position"] = Standard_movement["position"] - 1
            Standard_movement["order"] = Standard_movement["order"] + 1
            Standard_movement["angle_pos"] = False
            print("false")
        elif Standard_movement["order"] == 1:
            motor_status["stepp/status"] = 1
            stepper.move(3)
            motor_status["stepp/status"] = 0
            motor_status["stepp/position"] = motor_status["stepp/position"] + 3
            Standard_movement["order"] = 0
            c.publish("stepp/pos",str(motor_status["stepp/position"]))
    else:
        Standard_movement["position"] = 0
        Standard_movement["active"] = False
        c.publish("data/inst","stop_rec")







wifi_connect()
print("run while")



server="192.168.8.234"
c = MQTTClient("umqtt_client", server)
c.set_callback(sub_cb)
c.connect()
print("connected")
c.subscribe(b"angle/mpu")
c.subscribe(b"angle/mpu")
c.subscribe(b"genral/inst")
print("subbed")
"""
if __name__ == "__main__":
    main()
"""
while True:
    if motor_status["angle/status"] == 1:
        c.wait_msg()
        angle_reach()
    elif Standard_movement["active"] == True:
        print(str(Standard_movement["position"]))
        Custom_opp()
        Standard_movement["position"] = Standard_movement["position"] + 1
    if True:
        # Blocking wait for message
        c.wait_msg()
        pin.toggle()
        #print("loop")
    else:
        # Non-blocking wait for message
        c.check_msg()
        # Then need to sleep to avoid 100% CPU usage (in a real
        # app other useful actions would be performed instead)
        time.sleep(1)

c.disconnect()

"""
"""
## main code
print("LED starts flashing...")
print("Finished.")


### just need to be able to activat/decativate it with mqtt messsage