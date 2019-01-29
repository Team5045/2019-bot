from ctre import WPI_TalonSRX
from magicbot import tunable
from wpilib import Solenoid


class Elevator:

    motor = WPI_TalonSRX
    solenoid = DoubleSolenoid

    def execute(self):
        pass