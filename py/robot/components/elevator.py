from ctre import WPI_TalonSRX
from magicbot import tunable
from wpilib import Solenoid


class Elevator:

    left_motor = WPI_TalonSRX
    right_motor = WPI_TalonSRX
    wrist_motor = WPI_TalonSRX

    def execute(self):
        pass