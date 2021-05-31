# import time
# import serial
import math

import numpy as np
from pytweening import easeInOutQuint, easeOutSine
from scipy.misc import derivative
from scipy.interpolate import interp1d
# from raspberryturk.embedded.motion.arm_movement_engine import ArmMovementEngine
from pypose.ax12 import *
from pypose.drivers import Drivers
import motion.kinematic_solver as ks

SERVO_1 = 16
SERVO_2 = 10  # left
SERVO_3 = 15  # combined with SERVO_2
SERVO_4 = 2  # right
SERVO_5 = 17  # combined with SERVO_4
SERVO_6 = 11
SERVOS = [SERVO_1, SERVO_2, SERVO_4, SERVO_6]
SERVOS_TOTAL = [SERVO_1, SERVO_2, SERVO_3, SERVO_4, SERVO_5, SERVO_6]
MIN_SPEED = 20
MAX_SPEED = 80
RESTING_POSITION = (512, 512, 512, 512)


def _register_bytes_to_value(register_bytes):
    return register_bytes[0] + (register_bytes[1] << 8)


def _easing_derivative(p):
    d = 0.0
    try:
        d = derivative(easeInOutQuint(p), p, dx=1e-6)
    except ValueError:
        pass
    return d


def _adjusted_speed(start_position, goal_position, position):
    r = np.array([start_position, goal_position])
    clipped_position = np.clip(position, r.min(), r.max())
    f = interp1d(r, [0, 1])
    adj = _easing_derivative(f(clipped_position)) / _easing_derivative(0.5)
    amp = easeOutSine(abs(goal_position - +start_position) / 1023.0)
    return np.int_(MIN_SPEED + (MAX_SPEED - MIN_SPEED) * adj * amp)


class Arm(object):
    def __init__(self, port="/dev/ttyUSB0"):
        self.driver = Drivers(port=port)
        # self.movement_engine = ArmMovementEngine()

    def close(self):
        self.driver.close_port()

    # def recenter(self):
    #   self.move((512, 512))
    def initConfigration(self):
        self.move_new((1, 2, 3, 4))

    # def return_to_rest(self):  # position where dead pieces rest ?
    #  self.move_to_point([20, 13.5])

    def return_to_rest_new(self, joint_list):
        # default: [512, 512, 512, 512]
        joint1_teeth = joint_list[0]
        joint2_teeth = joint_list[1]
        joint3_teeth = joint_list[2]
        joint4_teeth = joint_list[3]
        print('[INFO] joint_list is:', joint_list)
        self.move_new((joint1_teeth, joint2_teeth, joint3_teeth, joint4_teeth))

    def driver_enable(self):
        for i in SERVOS_TOTAL:
            self.driver.torque_enable(i)

    def driver_disable(self):
        for i in SERVOS_TOTAL:
            self.driver.torque_disable(i)

    def set_driver_low_speed(self):
        for i in SERVOS_TOTAL:
            self.driver.setReg(i, P_GOAL_SPEED_L, LOW_SPEED)

    def move_new(self, goal_position):
        # start_position = self.current_position()
        self.set_driver_low_speed()
        for i in SERVOS:
            if i == SERVO_1:
                self.driver.set_goal_position(i, goal_position[0])
            elif i == SERVO_2:
                self.driver.syncwrite_goal_position([SERVO_2, goal_position[1]], [SERVO_3, 1023 - goal_position[1]])
            elif i == SERVO_4:
                self.driver.syncwrite_goal_position([SERVO_4, goal_position[2]], [SERVO_5, 1023 - goal_position[2]])
            elif i == SERVO_6:
                self.driver.set_goal_position(i, goal_position[3])

    def move(self, goal_position):
        start_position = self.current_position()
        self.set_speed([MIN_SPEED, MIN_SPEED])  # input 2 MIN_SPEED here ?

        for i in SERVOS:
            if i == SERVO_2:
                self.driver.setReg(i, P_GOAL_POSITION_L, [goal_position[i % 2] % 256, goal_position[i % 2] >> 8])
                self.driver.setReg(SERVO_3, P_GOAL_POSITION_L, [goal_position[i % 2] % 256, goal_position[i % 2] >> 8])

            elif i == SERVO_4:
                self.driver.setReg(i, P_GOAL_POSITION_L, [goal_position[i % 2] % 256, goal_position[i % 2] >> 8])
                self.driver.setReg(SERVO_5, P_GOAL_POSITION_L, [goal_position[i % 2] % 256, goal_position[i % 2] >> 8])

            else:
                self.driver.setReg(i, P_GOAL_POSITION_L, [goal_position[i % 2] % 256, goal_position[i % 2] >> 8])
        while self._is_moving():
            position = self.current_position()
            speed = [_adjusted_speed(start_position[i % 2], goal_position[i % 2], position[i % 2]) for i in SERVOS]
            self.set_speed(speed)

    def move_to_point_new(self, pt, piece_type):
        goal_position = self.movement_engine.convert_point_new(pt, piece_type)
        self.move_new(goal_position)

    def move_to_point(self, pt):
        goal_position = self.movement_engine.convert_point(pt)
        self.move(goal_position)

    def set_speed(self, speed):
        for i in SERVOS:
            self.driver.setReg(i, P_GOAL_SPEED_L, [speed[i % 2] % 256, speed[i % 2] >> 8])

    def current_position(self):
        return self._values_for_register(P_PRESENT_POSITION_L)

    def _is_moving(self):
        return any([self.driver.getReg(index, P_MOVING, 1) == 1 for index in SERVOS])

    def _values_for_register(self, register):
        return [_register_bytes_to_value(self.driver.getReg(index, register, 2)) for index in SERVOS]

    # def teeth2rad(self, teeth):
    #     max_teeth = 1024
    #     return 2 * math.pi * (teeth // 1024)

    def rad2teeth(self, rad, joint_num):
        if joint_num == 1 or joint_num == 2 or joint_num == 4:
            return int((240 - (rad * 180 // math.pi)) / 0.292969)
        elif joint_num == 3:
            return int((- (rad * 180 // math.pi) + 60) / 0.292969)


def main():
    arm = Arm(port='/dev/tty.usbserial-FT4THVJ7')
    # # arm.driver_enable()
    # # # start -> end: [0.3, 0.3, 0.03], [0.3, 0.2, 0.03]
    # # # arm.return_to_rest_new([512, 512, 200, 512])
    # # # ks.star_to_des_solver()
    #
    # """
    # Here is the test for ik_solver
    # """
    #
    # angles, _ = ks.ik_solver([0.3, 0.1], False)
    # print('[INFO] angles sloved by ik:', angles)
    # joint1_angle = arm.rad2teeth(0, 2)
    # joint2_angle = arm.rad2teeth(angles[0], 2)
    # joint3_angle = arm.rad2teeth(angles[1], 3)
    # joint4_angle = arm.rad2teeth(angles[2], 4)
    # arm.return_to_rest_new([joint1_angle, joint2_angle, joint3_angle, joint4_angle])
    #
    # """
    # Here is the test for start_to_end
    # """
    #
    # solved_angles = ks.star_to_des_solver([0.3, 0.2, 0.1], [0.15, 0.2, 0.1], False)
    # print('[INFO] solved_angles:', solved_angles)
    # pick_solved_angles = solved_angles[0]
    # base_solved_angle = solved_angles[1]
    # move_2_solved_angles = solved_angles[2]
    # place_solved_angles = solved_angles[3]
    #
    # # joint1_angle = arm.rad2teeth(0, 2)
    # joint2_angle = arm.rad2teeth(pick_solved_angles[0], 2)
    # joint3_angle = arm.rad2teeth(pick_solved_angles[1], 3)
    # joint4_angle = arm.rad2teeth(pick_solved_angles[2], 4)
    # arm.return_to_rest_new([joint1_angle, joint2_angle, joint3_angle, joint4_angle])
    #
    # joint1_angle = arm.rad2teeth(base_solved_angle, 2)
    # # joint2_angle = arm.rad2teeth(pick_solved_angles[0], 2)
    # # joint3_angle = arm.rad2teeth(pick_solved_angles[1], 3)
    # # joint4_angle = arm.rad2teeth(pick_solved_angles[2], 4)
    # arm.return_to_rest_new([joint1_angle, joint2_angle, joint3_angle, joint4_angle])
    #
    # # joint1_angle = arm.rad2teeth(base_solved_angle, 2)
    # joint2_angle = arm.rad2teeth(move_2_solved_angles[0], 2)
    # joint3_angle = arm.rad2teeth(move_2_solved_angles[1], 3)
    # joint4_angle = arm.rad2teeth(move_2_solved_angles[2], 4)
    # arm.return_to_rest_new([joint1_angle, joint2_angle, joint3_angle, joint4_angle])
    #
    # # joint1_angle = arm.rad2teeth(base_solved_angle, 2)
    # joint2_angle = arm.rad2teeth(place_solved_angles[0], 2)
    # joint3_angle = arm.rad2teeth(place_solved_angles[1], 3)
    # joint4_angle = arm.rad2teeth(place_solved_angles[2], 4)
    # arm.return_to_rest_new([joint1_angle, joint2_angle, joint3_angle, joint4_angle])

    arm.driver_disable()
    arm.close()


if __name__ == '__main__':
    main()
