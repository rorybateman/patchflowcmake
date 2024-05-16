import machine
from machine import Pin

from utime import sleep
import time
import utime
import math
import network

from umqtt.simple import MQTTClient
from mqtt_server.Kalman import KalmanAngle
from machine import Pin, I2C
from mqtt_server.mpu6050 import MPU6050
from Flow import HallSensorCounter
pin = Pin("LED", Pin.OUT)


# Set up the I2C interface
i2c = machine.I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
#i2c = machine.I2C(1, sda=machine.Pin(14), scl=machine.Pin(15))

# Set up the MPU6050 class 
mpu = MPU6050(i2c)

# wake up the MPU6050 from sleep
mpu.wake()
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
        #print(".")
        sleep(1)
        pass

    #print('Connected to', ssid)
    #print('Network config:', wlan.ifconfig())

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


data_dict = {
    "angle/mpu": 0,
    "hall/reading": 0
}

angles = {
    "des_angle" : 0,
    "command": 0,
    "cur_angle" : 0
}




###init code###
wifi_connect()
client = mqtt_connect()
mqqt_timer = time.ticks_ms()

sensor = HallSensorCounter(pin=28)
#time since beginning of time

time_start = time.time()
##mqtt functions##
def send_protocal(cur_time,time,message):
    global sensor
    global time_start
    if cur_time > time:
        pin.toggle()
        #i = i + 1
        #message = message + "  i:" + str(i)
        
        mqtt_publish(client,"angle/mpu",message)
        
        mqtt_publish(client,"data/data", message +","+ str(sensor.pulse_count) +","+ str(time_start + cur_time))
        
        #mqtt_publish(client,"time/time",str(time_start + cur_time))
        
        sensor.pulse_count = 0
        time = time + 50
        #
        return time
    else:
        return time


### initiallising the angleometer###
RestrictPitch = True	#Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0
kalmanX = KalmanAngle()
kalmanY = KalmanAngle()


time.sleep(1)
#Read Accelerometer raw value
accel = mpu.read_accel_data()
accX = accel[0]
accY = accel[1]
accZ = accel[2]


if (RestrictPitch):
    roll = math.atan2(accY,accZ) * radToDeg
    pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
else:
    roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
    pitch = math.atan2(-accX,accZ) * radToDeg

kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll;
gyroYAngle = pitch;
compAngleX = roll;
compAngleY = pitch;
def m():
    global marker
    marker = marker + 1
timer = time.time()
flag = 0
marker = 0
### Main code looop###
while True:
    if(flag >100): #Problem with the connection
        #print("There is a problem with the connection")
        #pico_pub.mqtt_publish(client,"angle/status","There is a problem with the connection")
        flag=0
        continue
    try:
        #Read Accelerometer raw value
        accel = mpu.read_accel_data()
        accX = accel[1]
        accY = accel[0]
        accZ = accel[2]

        #Read Gyroscope raw value
        gyro = mpu.read_gyro_data()
        gyroX = gyro[1]
        gyroY = gyro[0]
        gyroZ = gyro[2]

        dt = time.time() - timer
        timer = time.time()
        
        if (RestrictPitch):
            roll = math.atan2(accY,accZ) * radToDeg
            pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
        else:
            roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
            pitch = math.atan2(-accX,accZ) * radToDeg

        gyroXRate = gyroX/131
        gyroYRate = gyroY/131
        
        if (RestrictPitch):

            if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
                kalmanX.setAngle(roll)
                complAngleX = roll
                kalAngleX   = roll
                gyroXAngle  = roll

            else:
                kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)


            if(abs(kalAngleX)>90):
                gyroYRate  = -gyroYRate
                kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)

        else:

            if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
                kalmanY.setAngle(pitch)
                complAngleY = pitch
                kalAngleY   = pitch
                gyroYAngle  = pitch
                
                
            else:
                kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)
                

            if(abs(kalAngleY)>90):
                gyroXRate  = -gyroXRate
                kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)
        #print(kalAngleY)
		#angle = (rate of change of angle) * change in time
        gyroXAngle = gyroXRate * dt
        gyroYAngle = gyroYAngle * dt
        
		#compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
        compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
        compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch
        
        if ((gyroXAngle < -180) or (gyroXAngle > 180)):
            gyroXAngle = kalAngleX
        if ((gyroYAngle < -180) or (gyroYAngle > 180)):
            gyroYAngle = kalAngleY

        #print("Angle X: " + str(kalAngleX)+"   " +"Angle Y: " + str(kalAngleY))
        #print(str(roll)+"  "+str(gyroXAngle)+"  "+str(compAngleX)+"  "+str(kalAngleX)+"  "+str(pitch)+"  "+str(gyroYAngle)+"  "+str(compAngleY)+"  "+str(kalAngleY))
        
        message = str(kalAngleX)
        
        cur_timer = time.ticks_ms()
        
        #send message if timer has elapsed
        mqqt_timer = send_protocal(cur_timer,mqqt_timer,message)
        
        print(message)
        time.sleep(0.005)
		
    except Exception as exc:
        print(str(marker))
        marker = 0
        flag +=1