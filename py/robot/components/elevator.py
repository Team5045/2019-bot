import math
from magicbot import tunable
from ctre import WPI_TalonSRX
from enum import IntEnum
from wpilib import DoubleSolenoid, DigitalInput

from constants import TALON_TIMEOUT


class ElevatorState(IntEnum):
    LOCKED = 0
    RELEASED = 1


class ElevatorPosition(IntEnum):
    GROUND = 0
    ROCKET1 = 1000
    ROCKET2 = 2000
    ROCKET3 = 3000

class Elevator:

    USE_MOTIONMAGIC = True
    USE_LIMIT_SWITCH = True

    motor = WPI_TalonSRX
    slave_motor = WPI_TalonSRX
    wrist_motor = WPI_TalonSRX
    solenoid = DoubleSolenoid
    reverse_limit = DigitalInput

    kFreeSpeed = tunable(0.1)
    kZeroingSpeed = tunable(0.1)
    kP = tunable(0.3)
    kI = tunable(0.0)
    kD = tunable(0.0)
    kF = tunable(0.0)

    kCruiseVelocity = 30000
    kAcceleration = 12000

    setpoint = tunable(0)
    value = tunable(0)
    error = tunable(0)

    def setup(self):
        self.state = ElevatorState.LOCKED
        self.pending_position = None
        self.pending_drive = None
        self._temp_hold = None

        self.has_zeroed = False
        self.needs_brake = False
        self.braking_direction = None

        self.motor.setInverted(True)
        self.motor.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.motor.selectProfileSlot(0, 0)
        self.motor.setSensorPhase(True)

        self.motor.config_kP(0, self.kP, 0)
        self.motor.config_kI(0, self.kI, 0)
        self.motor.config_kD(0, self.kD, 0)
        self.motor.config_kF(0, self.kF, 0)

        try:
            self.motor.configMotionCruiseVelocity(self.kCruiseVelocity, 0)
            self.motor.configMotionAcceleration(self.kAcceleration, 0)
        except NotImplementedError:
            # Simulator - no motion profiling support
            self.USE_MOTIONMAGIC = False

    def is_encoder_connected(self):
        return self.motor.getPulseWidthRiseToRiseUs() != 0

    def get_encoder_position(self):
        return self.motor.getSelectedSensorPosition(0)

    def is_at_ground(self):
        return self.get_encoder_position() <= GROUND_CUTOFF

    def is_at_position(self, position):
        return abs(self.get_encoder_position() - position) <= \
            POSITION_TOLERANCE

    def lock(self):
        self.state = ElevatorState.LOCKED

    def release_lock(self):
        self.state = ElevatorState.RELEASED

    def lower_to_ground(self):
        self.pending_position = ElevatorPosition.GROUND

    def move_incremental(self, amount):
        '''
        Move `amount` inches.
        '''
        self.pending_position = self.get_encoder_position() + \
            amount / DISTANCE_PER_REV * UNITS_PER_REV

    def raise_freely(self):
        self.pending_drive = self.kFreeSpeed

    def lower_freely(self):
        self.pending_drive = -self.kFreeSpeed

    def execute(self):
        # For debugging
        # print('elevator', 'drive', self.pending_drive,
        #       'lim', self.reverse_limit.get(),
        #       'pending_pos', self.pending_position,
        #       'setpoint', self.setpoint,
        #       'val', self.value,
        #       'err', self.error,
        #       'curr_pos', self.motor.getQuadraturePosition(),
        #       'curr_velo', self.motor.getQuadratureVelocity())

        # Brake - apply the brake either when we reach peak of movement
        # (for upwards motion), and thus ds/dt = v = 0, or else immediately
        # if we're traveling downwards (since no e.z. way to sense gravity vs
        # intertial movement).
        if self.needs_brake:
            if self.pending_drive:
                self.needs_brake = False
            else:
                velocity = self.motor.getQuadratureVelocity()
                if velocity == 0 or \
                        self.braking_direction == -1 or \
                        velocity / abs(velocity) != self.braking_direction:
                    self.pending_position = self.motor.getQuadraturePosition()
                    self.needs_brake = False
                    self.braking_direction = None

        # Zero the encoder when hits bottom of elevator - limit switch NC
        # Also ensure we don't drive past the bottom limit.
        if self.USE_LIMIT_SWITCH and not self.reverse_limit.get():
            self.motor.setQuadraturePosition(0, TALON_TIMEOUT)
            self.has_zeroed = True
            if self.pending_drive and self.pending_drive <= 0:
                self.pending_drive = None

        # Elevator motor
        if self.pending_drive:
            self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                           self.pending_drive)
            self.pending_drive = None
            self.pending_position = None  # Clear old pending position
            self._temp_hold = None

        elif self.pending_position is not None and self.is_encoder_connected():
            # Note: we don't clear the pending position so that we keep
            # on driving to the position in subsequent execute() cycles.

            # If not zeroed, try out "best shot" at getting to desired places
            if not self.has_zeroed:
                if self.pending_position == ElevatorPosition.GROUND or \
                        self.pending_position == ElevatorPosition.CARRYING:
                    # Drive downwards until we zero it
                    self._temp_hold = None
                    self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                   -self.kZeroingSpeed)
                else:
                    # Hopefully it's at the start of the match and we're still
                    # near the top. Just hold position right where we are
                    if not self._temp_hold:
                        self._temp_hold = self.motor.getQuadraturePosition()
                    self.motor.set(WPI_TalonSRX.ControlMode.MotionMagic,
                                   self._temp_hold)

            # Otherwise, if we're zeroed, just set position normally
            elif self.has_zeroed:
                self._temp_hold = None
                if self.USE_MOTIONMAGIC:
                    self.motor.set(WPI_TalonSRX.ControlMode.MotionMagic,
                                   self.pending_position)
                else:
                    self.motor.set(WPI_TalonSRX.ControlMode.Position,
                                   self.pending_position)

        else:
            if self.is_encoder_connected():
                # If no command, hold position in place (basically, a more
                # "aggressive" brake mode to prevent any slippage).
                self.needs_brake = True
                velocity = self.motor.getQuadratureVelocity()
                self.braking_direction = velocity / abs(velocity or 1)
            self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0)

        # Elevator deployment/retraction
        if self.state == ElevatorState.LOCKED:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
        elif self.state == ElevatorState.RELEASED:
            self.solenoid.set(DoubleSolenoid.Value.kForward)

        # Update dashboard PID values
        if self.pending_position:
            try:
                self.setpoint = self.motor.getClosedLoopTarget(0)
                self.value = self.motor.getSelectedSensorPosition(0)
                self.error = self.motor.getClosedLoopError(0)
            except NotImplementedError:
                # Simulator doesn't implement getError
                pass

    def get_state(self):
        return {
            'pending_position': self.pending_position
        }

    def put_state(self, state):
        self.pending_position = state['pending_position']
