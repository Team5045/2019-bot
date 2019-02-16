from ctre import WPI_TalonSRX
from magicbot import tunable
from enum import IntEnum
from wpilib import DoubleSolenoid
from constants import TALON_TIMEOUT

class ClawState(IntEnum):
    RETRACTED = 0
    EXTENDED = 1

class ShiftState(IntEnum):
    RETRACTED = 0
    EXTENDED = 1

POSITION_TOLERANCE = 40

class Manipulator:

    belt_motor = WPI_TalonSRX
    roller_motor = WPI_TalonSRX
    wrist_motor = WPI_TalonSRX
    solenoid = DoubleSolenoid
    shift = DoubleSolenoid
    
    kP = tunable(0.3)
    kI = tunable(0.0)
    kD = tunable(0.0)
    kF = tunable(0.0)
    kFreeSpeed = tunable(0.3)

    setpoint = tunable(0)
    value = tunable(0)
    error = tunable(0)

    def setup(self):
        self.state = ClawState.RETRACTED
        self.shift_state = ShiftState.RETRACTED
        self.pending_position = None
        self.belt_motor.setSensorPhase(True)
        self.roller_motor.setInverted(True)
        self.roller_motor.setSensorPhase(True)

        self.wrist_motor.setSensorPhase(False)
        self.wrist_motor.setInverted(False)
        self.wrist_motor.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.wrist_motor.selectProfileSlot(0, 0)
        
        self.speed = 0.0

        self.wrist_motor.config_kP(0, self.kP, 0)
        self.wrist_motor.config_kI(0, self.kI, 0)
        self.wrist_motor.config_kD(0, self.kD, 0)
        self.wrist_motor.config_kF(0, self.kF, 0)

    def switch(self):
        if self.state == ClawState.EXTENDED:
            self.state = ClawState.RETRACTED
        elif self.state == ClawState.RETRACTED:
            self.state = ClawState.EXTENDED

    def extend(self):
        self.state = ClawState.EXTENDED

    def retract(self):
        self.state = ClawState.RETRACTED

    def run_belt(self, speed):
        self.belt_motor.set(speed)

    def run_roller(self, speed):
        self.roller_motor.set(speed)
    
    def is_encoder_connected(self):
        return self.wrist_motor.getPulseWidthRiseToRiseUs() != 0

    def get_encoder_position(self):
        return self.wrist_motor.getSelectedSensorPosition(0)

    def is_at_position(self, position):
        return abs(self.get_encoder_position() - position) <= \
            POSITION_TOLERANCE
    
    def reset_position(self):
        self.wrist_motor.setQuadraturePosition(0, TALON_TIMEOUT)

    def shift_pad(self):
        if self.shift == ShiftState.EXTENDED:
            self.shift = ShiftState.RETRACTED
        elif self.shift == ShiftState.RETRACTED:
            self.shift = ShiftState.EXTENDED

    def stop(self):
        self.belt_motor.set(0)
        self.roller_motor.set(0)

    def raise_freely(self):
        self.pending_drive = self.kFreeSpeed

    def lower_freely(self):
        self.pending_drive = -self.kFreeSpeed

    def drive(self, speed):
        self.pending_drive = speed

    def get_state(self):
        return {
            'claw_state': self.state,
            'shift_state': self.shift_state
        }

    def put_state(self, state):
        self.state = state['claw_state']
        self.shift_state = state['shift_state']

    def execute(self):
        if self.state == ClawState.RETRACTED:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
        elif self.state == ClawState.EXTENDED:
            self.solenoid.set(DoubleSolenoid.Value.kForward)

        if self.shift_state == ShiftState.RETRACTED:
            self.shift.set(DoubleSolenoid.Value.kReverse)
        elif self.shift_state == ShiftState.EXTENDED:
            self.shift.set(DoubleSolenoid.Value.kForward)

        self.run_belt(self.speed)
        self.run_roller(self.speed)

        if self.pending_drive and self.is_encoder_connected():
            self.wrist_motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                           self.pending_drive)
            self.pending_drive = None