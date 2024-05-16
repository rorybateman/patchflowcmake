from machine import Pin
import time

# Set the GPIO pin for the limit switch
limit_switch_pin = 2

# Set the pin as an input with pull-up resistor
limit_switch = Pin(limit_switch_pin, Pin.IN, Pin.PULL_UP)

# Define a function to detect the pulse
def detect_pulse(pin):
    print("Pulse detected!")

# Set up the interrupt for the limit switch pin
limit_switch.irq(trigger=Pin.IRQ_RISING, handler=detect_pulse)

try:
    print("Waiting for pulse...")
    while True:
        # Keep the program running
        time.sleep(1)

except KeyboardInterrupt:
    # Clean up the GPIO pins
    limit_switch.irq(handler=None)
    print("Exiting program.")