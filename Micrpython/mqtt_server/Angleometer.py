
import machine
from machine import Pin
from utime import sleep
from umqtt.simple import MQTTClient
import mpu6050
import time

from machine import Pin, I2C
import utime

# Set up the I2C interface
i2c = machine.I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
#i2c = machine.I2C(1, sda=machine.Pin(14), scl=machine.Pin(15))

# Set up the MPU6050 class 
mpu = mpu6050.MPU6050(i2c)

# wake up the MPU6050 from sleep
mpu.wake()

# continuously print the data
while True:
    gyro = mpu.read_gyro_data()
    accel = mpu.read_accel_data()
    print("Gyro: " + str(gyro) + ", Accel: " + str(accel))
    time.sleep(0.1)