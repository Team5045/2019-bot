import wpilib
import ctre
import magicbot
import navx
from components import drivetrain, targeting, field, manipulator, elevator, lift, wrist
from controllers import alignment_controller

CONTROLLER_LEFT = wpilib.XboxController.Hand.kLeft
CONTROLLER_RIGHT = wpilib.XboxController.Hand.kRight


class SpartaBot(magicbot.MagicRobot):

    drivetrain = drivetrain.Drivetrain
    targeting = targeting.Targeting
    field = field.Field
    manipulator = manipulator.Manipulator
    elevator = elevator.Elevator
    wrist = wrist.Wrist
    lift = lift.Lift

    alignment_controller = alignment_controller.AlignmentController

    use_teleop_in_autonomous = True

    def createObjects(self):
        self.drive_controller = wpilib.XboxController(0)
        self.compressor = wpilib.Compressor()

        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(3)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(2)
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(5)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(4)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(2)
        self.navx = navx.AHRS.create_spi()

        self.manipulator_roller_motor = ctre.WPI_TalonSRX(6)
        self.manipulator_solenoid = wpilib.DoubleSolenoid(1, 3)
        self.manipulator_shift = wpilib.DoubleSolenoid(0, 4)

        self.elevator_motor = ctre.WPI_TalonSRX(1)
        self.elevator_slave_motor = ctre.WPI_TalonSRX(7)
        self.elevator_reverse_limit = wpilib.DigitalInput(0)

        self.wrist_motor = ctre.WPI_TalonSRX(8)

        self.lift_solenoid = wpilib.DoubleSolenoid(5,6)

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        angle = self.drive_controller.getX(CONTROLLER_RIGHT)
        self.drivetrain.angle_corrected_differential_drive(
            self.drive_controller.getY(CONTROLLER_LEFT), angle)

        if self.drive_controller.getBumperReleased(CONTROLLER_LEFT):
            self.manipulator.switch()
        if self.drive_controller.getBumperReleased(CONTROLLER_RIGHT) and self.drive_controller.getStickButtonReleased(CONTROLLER_RIGHT):
            self.lift.switch()

        if self.drive_controller.getTriggerAxis(CONTROLLER_LEFT)>0.5:
            self.manipulator.run_roller(-0.5)
        elif self.drive_controller.getTriggerAxis(CONTROLLER_RIGHT)>0.5:
            self.manipulator.run_roller(0.5)
        else:
            self.manipulator.run_roller(0)

        if self.drive_controller.getAButtonReleased():
            self.wrist.toggle_forward()

        if self.drive_controller.getXButtonReleased():
            self.elevator.toggle()
        elif self.drive_controller.getYButtonReleased():
            self.elevator.toggle(False)

        if self.drive_controller.getStickButtonReleased(CONTROLLER_LEFT):
            self.drivetrain.shift_toggle()
        if self.drive_controller.getStickButtonPressed(CONTROLLER_RIGHT):
            self.alignment_controller.move_to(0)
        else:
            self.alignment_controller.stop()
        
        print(self.wrist.get_position())

if __name__ == '__main__':
    wpilib.run(SpartaBot)
