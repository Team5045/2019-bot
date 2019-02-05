import wpilib
import ctre 
import magicbot
import navx
from magicbot import tunable
from components import drivetrain, irsensor#, elevator, manipulator, lift

CONTROLLER_LEFT = wpilib.XboxController.Hand.kLeft
CONTROLLER_RIGHT = wpilib.XboxController.Hand.kRight

class SpartaBot(magicbot.MagicRobot):

    drivetrain = drivetrain.Drivetrain
    irsensor = irsensor.IRSensor
    #elevator = elevator.Elevator
    #manipulator = manipulator.Manipulator
    #lift = lift.Lift

    def createObjects(self):
        self.drive_controller = wpilib.XboxController(0)
        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(2)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(3)
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(4)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(5)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(2)
        self.compressor = wpilib.Compressor()
        self.navx = navx.AHRS.create_spi()

        self.irsensor_serial = wpilib.SerialPort(9600, wpilib.SerialPort.Port.kUSB) 

        #self.elevator_left_motor = ctre.WPI_TalonSRX(0)
        #self.elevator_right_motor = ctre.WPI_TalonSRX(1)
        #self.elevator_wrist_motor = ctre.WPI_TalonSRX(6)

        #self.manipulator_left_motor = ctre.WPI_TalonSRX(7)
        #self.manipulator_right_motor = ctre.WPI_TalonSRX(8)
        #self.manipulator_solenoid = wpilib.DoubleSolenoid(1, 2)

        #self.lift_solenoid = wpilib.DoubleSolenoid(3, 4)
        
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
