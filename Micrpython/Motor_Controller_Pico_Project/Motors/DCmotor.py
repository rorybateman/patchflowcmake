from time import sleep
import machine


class DCmotor:
    def __init__(self, cpin,acpin):
        self.pin1 = machine.PWM(machine.Pin(cpin))
        self.pin2 = machine.PWM(machine.Pin(acpin))
        self.pin1.freq(1000)
        self.pin2.freq(1000)

    def move_clockwise(self, duty):
        self.pin1.duty_u16(int(duty*65535/100))
        self.pin2.duty_u16(0)

    def move_counterclockwise(self, duty):
        self.pin1.duty_u16(0)
        self.pin2.duty_u16(int(duty*65535/100))
    
    def move(self,duty):
        if duty < 0 :
            self.move_counterclockwise(duty*-1)
        elif duty > 0 :
            self.move_clockwise(duty)
        else:
            self.stop()
    def stop(self):
        self.pin1.duty_u16(0)
        self.pin2.duty_u16(0)



"""
#motor = DCmotor(27,26)

print("clockwise?")
#motor.move_clockwise(100)
sleep(2)
print("stop")
motor.stop()
sleep(2)
print("clockwise?")
motor.move_counterclockwise(100)
sleep(2)
#print("move_counterclockwise?")
#motor.move_clockwise(100)
#sleep(2)
print("stop")
motor.stop()


"""
