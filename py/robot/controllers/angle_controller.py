from magicbot import tunable
from robotpy_ext.common_drivers import navx
import hal

from components.drivetrain import Drivetrain
from .base_pid_controller import BasePIDComponent


class AngleController(BasePIDComponent):
    """
    When enabled, controls the angle of the robot.
    """

    drivetrain = Drivetrain

    kP = tunable(0.01 if hal.HALIsSimulation() else 0.1)
    kI = tunable(0)
    kD = tunable(0)
    kF = tunable(0)
    kToleranceDegrees = tunable(0.5 if hal.HALIsSimulation() else 0.75)
    kIzone = tunable(0)

    navx = navx.AHRS

    def __init__(self):
        super().__init__(self.get_angle, 'angle_controller')

        self.last_angle = 0
        self.angle_reset_factor = 0

        if hal.HALIsSimulation():
            self.set_abs_output_range(0.08, 0.25)
        else:
            # self.set_abs_output_range(0.13, 0.25)
            self.set_abs_output_range(0.13, 0.4)

        if hasattr(self, 'pid'):
            self.pid.setInputRange(-180.0, 180.0)
            self.pid.setOutputRange(-1.0, 1.0)
            self.pid.setContinuous(True)

        self.report = 0

    def get_angle(self):
        """
        Return the robot's current heading.
        """
        # print('current_angle', self.navx.getYaw())
        try:
            self.last_angle = self.navx.getYaw()
            return self.last_angle - self.angle_reset_factor
        except Exception as e:
            print('!!! gyro error, falling back', e)
            return self.last_angle - self.angle_reset_factor

    def align_to(self, angle):
        """
        Move the robot and turns it to a specified absolute direction.
        :param angle: Angle value from 0 to 359
        """
        self.setpoint = angle

    def is_aligned(self):
        """
        Return True if robot is pointing at specified angle.
        Note: Always returns False when move_at_angle is not being called.
        """
        if not self.enabled:
            return False

        return self.is_aligned_to(self.setpoint)

    def is_aligned_to(self, setpoint):
        """
        :param setpoint: Setpoint value to check alignment against
        :return: True if aligned False if not
        """
        angle = self.get_angle()

        # compensate for wraparound (code from PIDController)
        error = setpoint - angle
        if abs(error) > 180.0:
            if error > 0:
                error -= 360.0
            else:
                error += 360.0

        return abs(error) < self.kToleranceDegrees

    def reset_angle(self):
        self.drivetrain.shift_low_gear()
        self.angle_reset_factor += self.get_angle()
        # if hal.HALIsSimulation():
        #     self.angle_reset_factor += self.get_angle()
        # else:
        #     self.navx.reset()
        print('angle_controller#reset_angle', self.get_angle())

    def compute_error(self, setpoint, pid_input):
        """
        Compute the error between the setpoint and pid_input.
        :return: A value in degrees of error. (360 to -360)
        """
        error = pid_input - setpoint

        if abs(error) > 180.0:  # Used to find the closest path to the setpoint
            if error > 0:
                error -= 360.0
            else:
                error += 360.0

        return error

    def execute(self):
        super().execute()

        # rate will never be None when the component is not enabled
        if self.rate is not None:
            if self.is_aligned():
                self.stop()
            else:
                self.drivetrain.turn(-self.rate, force=True)

    def stop(self):
        self.drivetrain.differential_drive(0)

    def on_disable(self):
        self.stop()
