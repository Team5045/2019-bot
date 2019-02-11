from wpilib import Timer

from collections import namedtuple
from controllers import position_controller, angle_controller, path_controller

TrajectoryAction = namedtuple('TrajectoryAction',
                              ['rotate', 'position', 'path', 'timeout',
                               'reverse'])


class TrajectoryController:

    position_controller = position_controller.PositionController
    angle_controller = angle_controller.AngleController
    path_controller = path_controller.PathController

    def __init__(self):
        self.actions = []
        self.current_action = None
        self.has_reset = False
        self.timeout_start = None

    def push(self, rotate=None, position=None, path=None, reverse=None,
             timeout=None):
        self.actions.append(TrajectoryAction(rotate=rotate, position=position,
                                             path=path, reverse=reverse,
                                             timeout=timeout))

    def reset(self):
        self.actions = []
        self.current_action = None
        self.has_reset = False

    def is_finished(self):
        return len(self.actions) == 0 and not self.current_action

    def execute(self):
        if not self.current_action:
            if self.actions:
                self.current_action = self.actions.pop(0)
                self.has_reset = False
                if self.current_action.timeout:
                    self.timeout_start = Timer.getFPGATimestamp()
            else:
                self.position_controller.stop()
                self.angle_controller.stop()
                self.path_controller.stop()

        if self.current_action:
            if self.current_action.rotate:
                if not self.has_reset:
                    self.angle_controller.reset_angle()
                    self.has_reset = True
                else:
                    self.angle_controller.align_to(self.current_action.rotate)
                    if self.angle_controller.is_aligned():
                        self.current_action = None

            elif self.current_action.position:
                if not self.has_reset:
                    self.position_controller.reset_position_and_heading()
                    self.has_reset = True
                else:
                    self.position_controller.move_to(
                        self.current_action.position)
                    if self.position_controller.is_at_location():
                        self.current_action = None

            elif self.current_action.path:
                if not self.has_reset:
                    self.path_controller.set(self.current_action.path,
                                             self.current_action.reverse)
                    self.has_reset = True
                else:
                    self.path_controller.run()
                    if self.path_controller.is_finished():
                        self.current_action = None

            if self.current_action and self.current_action.timeout:
                if Timer.getFPGATimestamp() - self.timeout_start > \
                        self.current_action.timeout:
                    self.current_action = None

    def on_disable(self):
        self.current_action = None
        self.actions = []
