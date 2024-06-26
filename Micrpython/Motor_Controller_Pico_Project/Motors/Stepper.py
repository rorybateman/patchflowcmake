from machine import Pin
import time

# need to add an interupt sequence to stop the for loop maybe 
class StepperMotor:
    def __init__(self, in1, in2, in3, in4):
        self.pin1 = Pin(in1, mode=Pin.OUT)
        self.pin2 = Pin(in2, mode=Pin.OUT)
        self.pin3 = Pin(in3, mode=Pin.OUT)
        self.pin4 = Pin(in4, mode=Pin.OUT)
        self.lock = False
        self.pins = [
            self.pin1, # in1
            self.pin2, # in2
            self.pin3, # in3
            self.pin4  # in4
        ]
        self.full_step_sequence = [
            [1,0,0,0],
            [1,1,0,0],
            [0,1,0,0],
            [0,1,1,0],
            [0,0,1,0],
            [0,0,1,1],
            [0,0,0,1],
            [1,0,0,1]
        ]
        self.full_step_sequence_counter = [
            [1,0,0,1],
            [0,0,0,1],
            [0,0,1,1],
            [0,0,1,0],
            [0,1,1,0],
            [0,1,0,0],
            [1,1,0,0],
            [1,0,0,0]
        ]
    def stop(self):
        self.pin1.low()
        self.pin2.low()
        self.pin3.low()
        self.pin4.low()
        self.lock = True
    def move_clockwise(self, steps):
        for _ in range(steps): # and self.x == True
            if self.lock == True:
                _ = _ + 2* steps
            elif self.lock == False:
                for step in self.full_step_sequence:
                    for i in range(len(self.pins)):
                        self.pins[i].value(step[i])
                        time.sleep(0.001)
                i = 0
        _ = 0

    def move_counterclockwise(self, steps):
        for _ in range(steps):
            if self.lock == True:
                _ = _ + 2* steps
            elif self.lock == False:
                for step in self.full_step_sequence_counter:
                   for i in range(len(self.pins)):
                        self.pins[i].value(step[i])
                        time.sleep(0.001)
                i = 0
        _ = 0
    def move(self,input):
        self.lock = False
        if input < 0:
            self.move_counterclockwise(-1*input)
        else:
            self.move_clockwise(input)



###example usage:###

#stepper = StepperMotor(15,14,16,17)
#stepper.move(-100)
#stepper.move(20)


#stepper.stop()