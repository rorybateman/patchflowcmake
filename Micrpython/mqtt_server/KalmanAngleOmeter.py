import machine
from machine import Pin

from utime import sleep
import time
import utime
import math

from Kalman import KalmanAngle
from machine import Pin, I2C
import mpu6050



# Set up the I2C interface
i2c = machine.I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
#i2c = machine.I2C(1, sda=machine.Pin(14), scl=machine.Pin(15))

# Set up the MPU6050 class 
mpu = mpu6050.MPU6050(i2c)

# wake up the MPU6050 from sleep
mpu.wake()

# continuously print the data
gyro = mpu.read_gyro_data()
accel = mpu.read_accel_data()
print("Gyro: " + str(gyro) + ", Accel: " + str(accel))
time.sleep(0.1)


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

#print(accX,accY,accZ)
#print(math.sqrt((accY**2)+(accZ**2)))
if (RestrictPitch):
    roll = math.atan2(accY,accZ) * radToDeg
    pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
else:
    roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
    pitch = math.atan2(-accX,accZ) * radToDeg

print(roll)
kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll;
gyroYAngle = pitch;
compAngleX = roll;
compAngleY = pitch;

timer = time.time()
flag = 0
while True:
    if(flag >100): #Problem with the connection
        print("There is a problem with the connection")
        flag=0
        continue
    try:
        #Read Accelerometer raw value
        accel = mpu.read_accel_data()
        accX = accel[0]
        accY = accel[1]
        accZ = accel[2]

        #Read Gyroscope raw value
        gyro = mpu.read_gyro_data()
        gyroX = gyro[0]
        gyroY = gyro[1]
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

        print("Angle X: " + str(kalAngleX)+"   " +"Angle Y: " + str(kalAngleY))
        time.sleep(0.005)
		
    except Exception as exc:
        flag += 1