from machine import Pin
import time


class StepperMotor:
    def __init__(self, in1, in2, in3, in4):
        self.pin1 = Pin(in1, mode=Pin.OUT)
        self.pin2 = Pin(in2, mode=Pin.OUT)
        self.pin3 = Pin(in3, mode=Pin.OUT)
        self.pin4 = Pin(in4, mode=Pin.OUT)
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
    def move_clockwise(self, steps):
        for _ in range(steps):
            for step in self.full_step_sequence:
                for i in range(len(self.pins)):
                    self.pins[i].value(step[i])
                    time.sleep(0.001)
            i = 0
        _ = 0

    def move_counterclockwise(self, steps):
        for _ in range(steps):
            for step in self.full_step_sequence_counter:
                for i in range(len(self.pins)):
                    self.pins[i].value(step[i])
                    time.sleep(0.001)
            i = 0
        _ = 0
    def move(self,input):
        if input < 0:
            self.move_counterclockwise(-1*input)
        else:
            self.move_clockwise(input)
    def stop(self):
        self.pin1.low()
        self.pin2.low()
        self.pin3.low()
        self.pin4.low()



###example usage:###

#stepper = StepperMotor(15,14,16,17)
#stepper.move(-20)
#stepper.move(20)

#stepper.stop()