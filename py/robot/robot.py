import wpilib
import ctre
import magicbot
import navx
from magicbot import tunable
from components import drivetrain, irsensor, targeting, field, manipulator, elevator, lift
from common import rumbler
from controllers import angle_controller, position_controller, trajectory_controller

CONTROLLER_LEFT = wpilib.XboxController.Hand.kLeft
CONTROLLER_RIGHT = wpilib.XboxController.Hand.kRight

AUTO_PLACE = False
MANIPULATOR_RANGE = 3.0

class SpartaBot(magicbot.MagicRobot):

    drivetrain = drivetrain.Drivetrain
    irsensor = irsensor.IRSensor
    targeting = targeting.Targeting
    field = field.Field
    manipulator = manipulator.Manipulator
    elevator = elevator.Elevator
    lift = lift.Lift
    position_controller = position_controller.PositionController
    angle_controller = angle_controller.AngleController
    trajectory_controller = trajectory_controller.TrajectoryController

    def createObjects(self):
        self.drive_controller = wpilib.XboxController(0)
        self.compressor = wpilib.Compressor()

        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(2)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(3)
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(4)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(5)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(2)
        self.navx = navx.AHRS.create_spi()

        self.irsensor_serial = wpilib.SerialPort(
            9600, wpilib.SerialPort.Port.kUSB)

        self.manipulator_belt_motor = ctre.WPI_TalonSRX(7)
        self.manipulator_roller_motor = ctre.WPI_TalonSRX(8)
        self.manipulator_solenoid = wpilib.DoubleSolenoid(0, 1)

        self.elevator_motor = ctre.WPI_TalonSRX(0)
        self.elevator_slave_motor = ctre.WPI_TalonSRX(1)
        self.elevator_wrist_motor = ctre.WPI_TalonSRX(6)
        self.elevator_reverse_limit = wpilib.DigitalInput(0)
        self.elevator_solenoid = wpilib.DoubleSolenoid(0, 1)

        self.lift_solenoid = wpilib.DoubleSolenoid(3, 4)

        self.ultrasonic = wpilib.Ultrasonic(0,0,wpilib.Ultrasonic.kInches)

    def autonomousInit(self):
        self.teleopInit()

    def autonomousPeriodic(self):
        self.teleopPeriodic()

    def teleopInit(self):
        self.drivetrain.reset_angle_correction()
        self.drivetrain.shift_high_gear()

    def teleopPeriodic(self):
        if self.drive_controller.getBumperReleased(CONTROLLER_LEFT):
            self.drivetrain.shift_toggle()
        if self.drive_controller.getBumperReleased(CONTROLLER_RIGHT):
            self.manipulator.switch()
        angle = self.drive_controller.getX(CONTROLLER_RIGHT)
        self.drivetrain.angle_corrected_differential_drive(
            self.drive_controller.getY(CONTROLLER_LEFT), angle)
        if AUTO_PLACE:
            if self.irsensor.isOriented() or self.targeting.isOriented():
                # do elevator
                if self.ultrasonic.getRangeInches()>MANIPULATOR_RANGE:
                    self.drivetrain.drive(0.2)
                elif self.ultrasonic.getRangeInches()<MANIPULATOR_RANGE:
                    self.manipulator.retract()            

if __name__ == '__main__':
    wpilib.run(SpartaBot)
