import magicbot
import wpilib
import ctre
import math
from robotpy_ext.common_drivers import navx
from magicbot import tunable
from components import drivetrain

CONTROLLER_LEFT = wpilib.XboxController.Hand.kLeft
CONTROLLER_RIGHT = wpilib.XboxController.Hand.kRight


class SpartaBot(magicbot.MagicRobot):

    bot = bot.Bot
    drivetrain = drivetrain.Drivetrain

    def createObjects(self):
        self.drive_controller = wpilib.XboxController(1)
        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(4)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(7)
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(5)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(6)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(0)
        
        self.navx = navx.AHRS.create_spi()

    def teleopInit(self):
        self.drivetrain.shift_high_gear()
        
    def teleopPeriodic(self):
        angle = self.drive_controller.getX(CONTROLLER_RIGHT)
        self.drivetrain.angle_corrected_differential_drive(
            self.drive_controller.getY(CONTROLLER_LEFT), angle)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == '__main__':
    wpilib.run(SpartaBot)
