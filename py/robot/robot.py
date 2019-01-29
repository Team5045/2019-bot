import wpilib
import ctre
import magicbot
import navx
from magicbot import tunable
from components import drivetrain, irsensor, elevator, manipulator
from constats import DIST_PIN, ANGLE_PIN, CONFIG_PIN
import math

CONTROLLER_LEFT = wpilib.XboxController.Hand.kLeft
CONTROLLER_RIGHT = wpilib.XboxController.Hand.kRight


class SpartaBot(magicbot.MagicRobot):

    drivetrain = drivetrain.Drivetrain
    irsensor = irsensor.IRSensor
    elevator = elevator.Elevator
    manipulator = manipulator.Manipulator

    def createObjects(self):
        self.drive_controller = wpilib.XboxController(0)
        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(2)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(3)
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(4)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(5)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(0)

        self.compressor = wpilib.Compressor()
        self.navx = navx.AHRS.create_spi()

        self.irsensor_dist_signal = wpilib.AnalogInput(DIST_PIN)
        self.irsensor_angle_signal = wpilib.AnalogInput(ANGLE_PIN)
        self.irsensor_configurator = wpilib.AnalogOutput(CONFIG_PIN)

        self.elevator_motor = ctre.WPI_TalonSRX(0)
        self.elevator_solenoid = wpilib.DoubleSolenoid(1,2)

        self.manipulator_motor = ctre.WPI_TalonSRX(7)
        self.manipulator_solenoid = wpilib.DoubleSolenoid(3,4)

    def teleopInit(self):
        self.drivetrain.reset_angle_correction()
        self.drivetrain.shift_high_gear()

    def teleopPeriodic(self):
        if self.drive_controller.getBumperReleased(CONTROLLER_LEFT):
            self.drivetrain.shift_toggle()
        angle = self.drive_controller.getX(CONTROLLER_RIGHT)
        self.drivetrain.angle_corrected_differential_drive(
            self.drive_controller.getY(CONTROLLER_LEFT), angle)


if __name__ == '__main__':
    wpilib.run(SpartaBot)
