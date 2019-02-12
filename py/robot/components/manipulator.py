from ctre import WPI_TalonSRX
from magicbot import tunable
from enum import IntEnum
from wpilib import DoubleSolenoid


class ClawState(IntEnum):
    RETRACTED = 0
    EXTENDED = 1


class Manipulator:

    belt_motor = WPI_TalonSRX
    roller_motor = WPI_TalonSRX
    solenoid = DoubleSolenoid

    def setup(self):
        self.state = ClawState.EXTENDED

    def switch(self):
        if self.state == ClawState.EXTENDED:
            self.state = ClawState.RETRACTED
        elif self.state == ClawState.RETRACTED:
            self.state = ClawState.EXTENDED

    def extend(self):
        self.state = ClawState.EXTENDED

    def retract(self):
        self.state = ClawState.RETRACTED

    def run_belt(self, speed):
        self.belt_motor.set(speed)

    def run_roller(self, speed):
        self.roller_motor.set(speed)

    def stop(self):
        self.belt_motor.set(0)
        self.roller_motor.set(0)

    def get_state(self):
        return {
            'claw_state': self.state
        }

    def put_state(self, state):
        self.state = state['claw_state']

    def execute(self):
        if self.state == ClawState.RETRACTED:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
        elif self.state == ClawState.EXTENDED:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
