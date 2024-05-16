from machine import Pin
import time

# Set the GPIO pin for the limit switch
limit_switch_pin = 1

# Set the pin as an input with pull-up resistor
limit_switch = Pin(limit_switch_pin, Pin.IN, Pin.PULL_UP)



bounce_lock = True
def detect_pulse(pin):
    global bounce_lock
    global stepper
    print("Pulse!")
    bounce_lock = True
    if bounce_lock == True:
        print("Pulse detected!")
        bounce_lock = False
# Set up the interrupt for the limit switch pin
limit_switch.irq(trigger=Pin.IRQ_FALLING, handler=detect_pulse)

while True:
    time.sleep(0.1)


"""

class HallSensorCounter:
    def __init__(self, pin):
        self.pin = machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP)
        self.pulse_count = 0
        # Add event detection for the  pin
        self.pin.irq(trigger=machine.Pin.IRQ_RISING, handler=self.hall_sensor_callback)
        self.activate = False

    def hall_sensor_callback(self,channel):
        self.pulse_count += 1
        self.activate = True
        print("Pulse detected! Total count: {}".format(self.pulse_count))



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