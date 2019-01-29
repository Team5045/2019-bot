from ctre import WPI_TalonSRX
from magicbot import tunable
from wpilib import Solenoid


class Manipulator:

    motor = WPI_TalonSRX
    solenoid = Solenoid

    def execute(self):
        pass