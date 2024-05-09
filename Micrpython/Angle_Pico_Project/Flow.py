import machine 
import time



class HallSensorCounter:
    def __init__(self, pin):
        self.pin = machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP)
        self.pulse_count = 0
        # Add event detection for the  pin
        self.pin.irq(trigger=machine.Pin.IRQ_RISING, handler=self.hall_sensor_callback)

    def hall_sensor_callback(self,channel):
        self.pulse_count += 1
        print("Pulse detected! Total count: {}".format(self.pulse_count))


"""
# Example usage
sensor = HallSensorCounter(pin=28)

mqqt_timer = time.ticks_ms()
def send_protocal(cur_time,time):
    global sensor
    if cur_time > time:
        time = time + 5000
        sensor.pulse_count = 0
        return time
    else:
        return time

while True:
    cur_timer = time.ticks_ms()
    mqqt_timer = send_protocal(cur_timer,mqqt_timer)
    print(str(sensor.pulse_count))
    """