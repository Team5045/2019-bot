from ctre import WPI_TalonSRX
from magicbot import tunable
from wpilib import DoubleSolenoid


class Manipulator:

    left_motor = WPI_TalonSRX
    right_motor = WPI_TalonSRX
    solenoid = DoubleSolenoid

    def execute(self):
        pass