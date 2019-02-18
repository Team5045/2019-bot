from ctre import WPI_TalonSRX
from magicbot import tunable
from constants import TALON_TIMEOUT
from wpilib import PIDController
import math

ARM_WEIGHT = 18
ARM_DISPLACEMENT = 0.15
GEAR_RATIO = 180

class Wrist:

    motor = WPI_TalonSRX
    kP = tunable(0.1)
    kI = tunable(0.1)
    kD = tunable(0.3)
    kF = tunable(0.0)

    kFreeSpeed = tunable(0.25)

    setpoint = tunable(0)
    value = tunable(0)
    error = tunable(0)


    def setup(self):
        self.pending_drive = None
        self.carrying = False
        self.front = None

        self.motor.setSensorPhase(False)
        self.motor.setInverted(True)
        self.motor.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.motor.selectProfileSlot(0, 0)
        
        self.motor.config_kP(0, self.kP, 0)
        self.motor.config_kI(0, self.kI, 0)
        self.motor.config_kD(0, self.kD, 0)
        self.motor.config_kF(0, self.kF, 0)

        self.pid_controller = PIDController(
            Kp=self.kP, Ki=self.kI, Kd=self.kD, Kf=self.kF,
            source=self.get_position,
            output=self.pidWrite)
        self.pid_controller.setInputRange(-400, 3500)
        self.pid_controller.setContinuous(False)
        self.pid_controller.setOutputRange(-0.25,0.25)
        self.pid_controller.setPercentTolerance(2)

    def reset_angle(self):
        self.reset_position()
        self.setpoint = 0

    def move_to(self, position, carrying=False):
        self.setpoint = position
        self.pid_controller.enable()
        if carrying:
            self.carrying = carrying

    def pidWrite(self, output):
        self.rate = -output

    def is_encoder_connected(self):
        return self.motor.getPulseWidthRiseToRiseUs() != 0

    def get_position(self):
        return self.motor.getSelectedSensorPosition(0)
    
    def reset_position(self):
        self.motor.setQuadraturePosition(0, TALON_TIMEOUT)

    def stop(self):
        self.pid_controller.disable()
        self.setpoint = 0
        self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0)

    def stop_at_front(self):
        self.pid_controller.disable()
        self.setpoint = 0
        self.front = True

    def lower_freely(self):
        self.pending_drive = self.kFreeSpeed

    def raise_freely(self):
        self.pending_drive = -self.kFreeSpeed

    def drive(self, speed):
        self.pending_drive = speed

    def execute(self):
        self.pid_controller.setSetpoint(self.setpoint)
        if self.front:
            if self.carrying:
                self.motor.set(WPI_TalonSRX.ControlMode.Current, .475)
            else:
                self.motor.set(WPI_TalonSRX.ControlMode.Current, .15)
        if self.rate is not None and self.pid_controller.isEnabled():
            if self.pid_controller.onTarget():
                if self.setpoint==2000:
                    self.stop_at_front()
                else:
                    self.front = False
                    self.stop()
            else:
                self.front = False
                self.drive(self.rate)
        if self.pending_drive and self.is_encoder_connected():
            self.front = False
            self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                           self.pending_drive)
            self.pending_drive = None
 
    def on_disable(self):
        self.stop()