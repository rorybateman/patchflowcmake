from time import sleep
import machine

class MotorEncoder:
    """
    A class to interface with a motor encoder in MicroPython.
    """
    def __init__(self, pin_a, pin_b, cpr=360):
        """
        Initialize the MotorEncoder object.
        
        Args:
            pin_a (int): The GPIO pin connected to the encoder channel A.
            pin_b (int): The GPIO pin connected to the encoder channel B.
            cpr (int): The number of counts per revolution of the encoder. Default is 360.
        """
        self.pin_a = machine.Pin(pin_a, machine.Pin.IN, machine.Pin.PULL_UP)
        self.pin_b = machine.Pin(pin_b, machine.Pin.IN, machine.Pin.PULL_UP)
        self.cpr = cpr
        self.count = 0
        self.pin_a.irq(trigger=machine.Pin.IRQ_RISING, handler=self.encoder_isr)
        self.pin_b.irq(trigger=machine.Pin.IRQ_RISING, handler=self.encoder_isr)

    def encoder_isr(self, pin):
        """
        Interrupt service routine for the encoder.
        Increments or decrements the count based on the state of the encoder channels.
        """
        if pin == self.pin_a:
            if self.pin_b.value() == 0:
                self.count += 1
            else:
                self.count -= 1
        else:
            if self.pin_a.value() == 0:
                self.count -= 1
            else:
                self.count += 1

    def get_position(self):
        """
        Returns the current position of the encoder in counts.
        """
        return self.count

    def get_revolutions(self):
        """
        Returns the current position of the encoder in revolutions.
        """
        return self.count / self.cpr

"""

encoder = MotorEncoder(1,2)
while True:
    print(str(encoder.get_position()))
    print(str(encoder.get_revolutions()))


"""