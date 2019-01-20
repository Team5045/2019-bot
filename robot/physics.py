import constants
import math
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain

_talon_vel_conv = (10 * 4 * math.pi) / (80 * 6.67 * 12)
_max_speed = 370


def talon_target(hal_data, talon_id):
    hal_target = hal_data['CAN'][talon_id]['value']
    return hal_target


def copy_talon_analog_pos(hal_data, talon_id):
    hal_data['CAN'][talon_id]['analog_in_position'] = hal_data['CAN'][talon_id]['value']


def voltage_to_degrees(voltage):
    deg = (voltage/5)*360
    if deg < 0:
        deg += 360
    return deg


class PhysicsEngine:    
    def __init__(self, physics_controller):
        self.physics_controller = physics_controller

    def update_sim(self, hal_data, now, tm_diff):
        module_speeds = [
            talon_target(hal_data, constants.swerve_config[1][2])*4096/8,
            talon_target(hal_data, constants.swerve_config[0][2])*4096/8,
            talon_target(hal_data, constants.swerve_config[3][2])*4096/8,
            talon_target(hal_data, constants.swerve_config[2][2])*4096/8
        ]

        module_angles = [
            talon_target(hal_data, constants.swerve_config[1][1]),
            talon_target(hal_data, constants.swerve_config[0][1]),
            talon_target(hal_data, constants.swerve_config[3][1]),
            talon_target(hal_data, constants.swerve_config[2][1]),
        ]
        print('s',module_speeds)
        print('a',[voltage_to_degrees(i) for i in module_angles])
        vx, vy, vw = four_motor_swerve_drivetrain(
            *module_speeds,
            *module_angles,
            constants.chassis_length / 12,
            constants.chassis_width / 12,
            5.0
        )

        self.physics_controller.vector_drive(vx, vy, vw, tm_diff)

        copy_talon_analog_pos(hal_data, constants.swerve_config[0][1])
        copy_talon_analog_pos(hal_data, constants.swerve_config[1][1])
        copy_talon_analog_pos(hal_data, constants.swerve_config[2][1])
        copy_talon_analog_pos(hal_data, constants.swerve_config[3][1])
