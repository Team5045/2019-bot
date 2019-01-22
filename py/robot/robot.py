import magicbot
import wpilib
import ctre
import math
import navx
from magicbot import tunable
from components import drivetrain

CONTROLLER_LEFT = wpilib.XboxController.Hand.kLeft
CONTROLLER_RIGHT = wpilib.XboxController.Hand.kRight


class SpartaBot(magicbot.MagicRobot):

    drivetrain = drivetrain.Drivetrain

    def createObjects(self):
        self.drive_controller = wpilib.XboxController(0)
        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(2)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(3)
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(4)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(5)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(0)
        
        self.navx = navx.AHRS.create_spi()

    def teleopInit(self):
        self.drivetrain.shift_high_gear()
        
    def teleopPeriodic(self):
        a = self.drive_controller.getY(CONTROLLER_LEFT)
        b = self.drive_controller.getY(CONTROLLER_RIGHT)
        self.drivetrain_left_motor_master.set(a)
        self.drivetrain_right_motor_master.set(b)


if __name__ == '__main__':
    wpilib.run(SpartaBot)
