import wpilib
import ctre
import magicbot
import navx
from magicbot import tunable
from components import drivetrain, irsensor, targeting, field, manipulator, elevator, lift, wrist
from common import rumbler
#from controllers import angle_controller, position_controller, trajectory_controller

CONTROLLER_LEFT = wpilib.XboxController.Hand.kLeft
CONTROLLER_RIGHT = wpilib.XboxController.Hand.kRight

class SpartaBot(magicbot.MagicRobot):

    drivetrain = drivetrain.Drivetrain
    targeting = targeting.Targeting
    field = field.Field
    manipulator = manipulator.Manipulator
    elevator = elevator.Elevator
    wrist = wrist.Wrist
    
    #irsensor = irsensor.IRSensor
    #position_controller = position_controller.PositionController
    #angle_controller = angle_controller.AngleController    
    #trajectory_controller = trajectory_controller.TrajectoryController

    def createObjects(self):
        self.drive_controller = wpilib.XboxController(0)
        self.compressor = wpilib.Compressor()

        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(3)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(2)
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(4)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(5)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(2)
        self.navx = navx.AHRS.create_spi()

        #self.irsensor_i2c = wpilib.I2C(wpilib.I2C.Port.kOnboard, 8)
        #self.irsensor_serial = wpilib.SerialPort(
        #    9600, wpilib.SerialPort.Port.kUSB)

        self.manipulator_belt_motor = ctre.WPI_TalonSRX(0)
        self.manipulator_roller_motor = ctre.WPI_TalonSRX(6)
        self.manipulator_solenoid = wpilib.DoubleSolenoid(1,3)
        self.manipulator_shift = wpilib.DoubleSolenoid(0,4)

        self.elevator_motor = ctre.WPI_TalonSRX(1)
        self.elevator_slave_motor = ctre.WPI_TalonSRX(7)
        self.elevator_reverse_limit = wpilib.DigitalInput(0)

        self.wrist_motor = ctre.WPI_TalonSRX(8)

    def autonomousInit(self):
        self.teleopInit() 

    def autonomousPeriodic(self):
        self.teleopPeriodic() 

    def teleopInit(self):
        self.drivetrain.reset_angle_correction()
        self.wrist.reset_position()
        self.manipulator.retract()
        self.carrying = False
        
        self.elevator_buffer = [100,2900,12000,20000]
        self.elevator_buffer_index = 0

    def teleopPeriodic(self):
        angle = self.drive_controller.getX(CONTROLLER_RIGHT)
        self.drivetrain.angle_corrected_differential_drive(
            self.drive_controller.getY(CONTROLLER_LEFT), angle)

        if self.drive_controller.getBumperReleased(CONTROLLER_LEFT):
            self.manipulator.switch()
            self.wrist.carrying = not self.wrist.carrying
        if self.drive_controller.getBumperReleased(CONTROLLER_RIGHT):
            self.manipulator.shift_pad()

        if self.drive_controller.getAButtonReleased():
            self.wrist.move_to(2050)
        elif self.drive_controller.getBButtonReleased():
            self.wrist.move_to(50)

        if self.drive_controller.getXButtonReleased():
            if self.elevator_buffer_index<len(self.elevator_buffer):
                self.elevator_buffer_index += 1
                self.elevator.move_to(self.elevator_buffer[self.elevator_buffer_index])
        elif self.drive_controller.getYButtonReleased():
            if self.elevator_buffer_index>0:
                self.elevator_buffer_index -= 1
                self.elevator.move_to(self.elevator_buffer[self.elevator_buffer_index])

    
if __name__ == '__main__':
    wpilib.run(SpartaBot)

