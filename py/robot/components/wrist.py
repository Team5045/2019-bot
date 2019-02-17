from ctre import WPI_TalonSRX
from magicbot import tunable
from constants import TALON_TIMEOUT
from wpilib import PIDController

POSITION_TOLERANCE = 50
class Wrist:

    motor = WPI_TalonSRX
    kP = tunable(0.1)
    kI = tunable(0.1)
    kD = tunable(0.3)
    kF = tunable(0.0)
    kFreeSpeed = tunable(0.3)
    kToleranceInches = tunable(0.2)
    kIzone = tunable(0)

    setpoint = tunable(0)
    value = tunable(0)
    error = tunable(0)


    def setup(self):
        self.pending_drive = None

        self.motor.setSensorPhase(True)
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
        self.pid_controller.setPercentTolerance(5)

    def reset_angle(self):
        self.reset_position()
        self.pid_controller.setSetpoint(0)

    def move_to(self, position):
        self.pid_controller.setSetpoint(position)
        self.pid_controller.enable()

    def pidWrite(self, output):
        self.rate = -output

    def is_encoder_connected(self):
        return self.motor.getPulseWidthRiseToRiseUs() != 0

    def get_position(self):
        return self.motor.getSelectedSensorPosition(0)

    def is_at_position(self, position):
        return abs(self.get_position() - position) <= \
            POSITION_TOLERANCE
    
    def reset_position(self):
        self.motor.setQuadraturePosition(0, TALON_TIMEOUT)

    def stop(self):
        self.motor.set(0)
        self.pid_controller.disable()

    def lower_freely(self):
        self.pending_drive = self.kFreeSpeed

    def raise_freely(self):
        self.pending_drive = -self.kFreeSpeed

    def drive(self, speed):
        self.pending_drive = speed

    def execute(self):
        if self.rate is not None:
            if self.pid_controller.onTarget():
                self.stop()
            else:
                self.drive(self.rate)
        if self.pending_drive and self.is_encoder_connected():
            self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                           self.pending_drive)
            self.pending_drive = None
 
    def on_disable(self):
        self.stop()