from magicbot import tunable
from wpilib import PIDController
import hal

from components.manipulator import Manipulator
from .base_pid_controller import BasePIDComponent

class WristController(BasePIDComponent):

    manipulator = Manipulator

    def __init__(self):
        super().__init__(self.get_encoder_position, 'wrist_controller')
        self.set_abs_output_range(0.18, 0.7)

        self._angle_offset = 0
        self.angle_pid_controller = PIDController(
            Kp=self.manipulator.kP, Ki=self.manipulator.kI, Kd=self.manipulator.kD, Kf=self.manipulator.kF,
            source=self.get_angle,
            output=self.pidWriteAngle)
        self.angle_pid_controller.setInputRange(0, 2500)
        self.angle_pid_controller.setContinuous(False)
        self.angle_pid_controller.setOutputRange(-self.manipulator.kFreeSpeed,
                                                 self.manipulator.kFreeSpeed)

    def get_angle(self):
        return self.manipulator.get_encoder_position()

    def reset_angle(self):
        self.manipulator.reset_position()
        self.angle_pid_controller.setSetpoint(0)

    def move_to(self, position):
        self.setpoint = position
        self.angle_pid_controller.enable()

    def is_at_location(self):
        return self.enabled and \
            abs(self.get_encoder_position() - self.setpoint) < self.kToleranceInches

    def pidWrite(self, output):
        self.rate = -output

    def pidWriteAngle(self, angle):
        self.angle = angle

    def execute(self):
        super().execute()

        if self.rate is not None:
            if self.is_at_location():
                self.stop()
            else:
                self.manipulator.drive(self.rate)

    def stop(self):
        self.angle_pid_controller.disable()

    def on_disable(self):
        self.stop()
