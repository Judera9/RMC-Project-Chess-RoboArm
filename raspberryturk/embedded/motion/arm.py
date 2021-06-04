# import time
# import serial
import _thread
import time

import math

import numpy as np
from pytweening import easeInOutQuint, easeOutSine
from scipy.misc import derivative
from scipy.interpolate import interp1d
# from raspberryturk.embedded.motion.arm_movement_engine import ArmMovementEngine
from pypose.ax12 import *
from pypose.drivers import Drivers
import motion.kinematic_solver as ks
import motion.transform as trans
from gripper import *

SERVO_1 = 16
SERVO_2 = 10  # left
SERVO_3 = 15  # combined with SERVO_2
SERVO_4 = 2  # right
SERVO_5 = 17  # combined with SERVO_4
SERVO_6 = 11
SERVOS = [SERVO_1, SERVO_2, SERVO_4, SERVO_6]
SERVOS_24 = [SERVO_2, SERVO_4]
SERVOS_TOTAL = [SERVO_1, SERVO_2, SERVO_3, SERVO_4, SERVO_5, SERVO_6]
SERVOS_NO_GRIPPER = [SERVO_1, SERVO_2, SERVO_3, SERVO_4, SERVO_5]
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


def rad2teeth(rad, joint_num):
    if joint_num == 1 or joint_num == 2 or joint_num == 4:
        return int((240 - (rad * 180 // math.pi)) / 0.292969)
    elif joint_num == 3:
        return int((- (rad * 180 // math.pi) + 60) / 0.292969)


def teeth2rad(teeth, joint_num):
    if joint_num == 1 or joint_num == 2 or joint_num == 4:
        return (240 - 0.292969 * teeth) * math.pi / 180
    elif joint_num == 3:
        return (60 - 0.292969 * teeth) * math.pi / 180


class Arm(object):
    def __init__(self, port="/dev/ttyUSB0"):
        self.driver = Drivers(port=port)
        self.gripper = Gripper()
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

    def sycn_return_to_rest_new(self, joint_list):
        joint1_teeth = joint_list[0]
        joint2_teeth = joint_list[1]
        joint3_teeth = joint_list[2]
        joint4_teeth = joint_list[3]
        print('[INFO] joint_list is:', joint_list)
        self.move_new1((joint1_teeth, joint2_teeth, joint3_teeth, joint4_teeth))

    def driver_enable(self):
        for i in SERVOS_TOTAL:
            self.driver.torque_enable(i)

    def driver_disable(self):
        for i in SERVOS_TOTAL:
            self.driver.torque_disable(i)

    def set_driver_low_speed(self):
        for i in SERVOS_NO_GRIPPER:
            self.driver.setReg(i, P_GOAL_SPEED_L, LOW_SPEED)
        self.driver.setReg(SERVO_6, P_GOAL_SPEED_L, SERVOS_6_SPEED)

    def get_SERVO6_position(self):
        present_position_teeth = [self.driver.read_present_position(i) for i in SERVOS_24]
        joint_angle2 = teeth2rad(present_position_teeth[0], 2)
        joint_angle3 = teeth2rad(present_position_teeth[1], 3)
        present_position_angle = rad2teeth(joint_angle2 + joint_angle3 + 3.1415 / 2, 4)
        return present_position_angle

    def gripper_thread(self):
        while self._is_moving_new():
            position = self.get_SERVO6_position()
            self.driver.set_goal_position(SERVO_6, position)

    # position: the goal position for 3 groups of ax12a, the fourth ax12a is RTST(real time self-adjusting)
    def move_new_rtst(self, position):
        # try:
        #     _thread.start_new_thread(self.gripper_thread, (None,))
        # except:
        #     print("Error: Could not run new thread")
        goal_position = [position[0], position[1], 1023 - position[1], position[2], 1023 - position[2]]
        self.set_driver_low_speed()
        self.driver.syncwrite_more_goal_position(SERVOS_NO_GRIPPER, goal_position)
        while self._is_moving_new():
            position = self.get_SERVO6_position()
            self.driver.set_goal_position(SERVO_6, position)
        # self.gripper_thread()
        # time.sleep(2)

    # position: the goal position for 4 groups of ax12a
    def move_new1(self, position):
        # start_position = self.current_position()
        goal_position = [position[0], position[1], 1023 - position[1], position[2], 1023 - position[2], position[3]]
        self.set_driver_low_speed()
        self.driver.syncwrite_more_goal_position(SERVOS_TOTAL, goal_position)

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

    def current_position(self):  # get rad for 4 SERVOS
        return [teeth2rad(self.driver.read_present_position(i)) for i in SERVOS]

    """def current_position(self):
        return self._values_for_register_new(P_PRESENT_POSITION_L)"""

    def _is_moving_new(self):
        return any([self.driver.read_1byte(index, P_MOVING) == 1 for index in SERVOS_NO_GRIPPER])

    def _is_moving(self):
        return any([self.driver.getReg(index, P_MOVING, 1) == 1 for index in SERVOS])

    def _values_for_register_new(self, register):
        return [_register_bytes_to_value(self.driver.read_2byte(index, register)) for index in SERVOS]

    def _values_for_register(self, register):
        return [_register_bytes_to_value(self.driver.getReg(index, register, 2)) for index in SERVOS]


def test1():
    arm = Arm(port='COM3')
    # arm = Arm(port='/dev/tty.usbserial-FT4THVJ7')
    arm.driver_enable()
    arm.set_driver_low_speed()
    # # angles, _ = ks.ik_solver([math.sqrt(0.2 ** 2 + 0.1 ** 2), 0.075], True)
    # angles, _ = ks.ik_solver([ks.l_2 + ks.gripper_err, ks.l_1], True)
    # print('[INFO] angles sloved by ik:', angles)
    # joint1_angle = arm.rad2teeth(0, 2)
    # joint2_angle = arm.rad2teeth(angles[0], 2)
    # joint3_angle = arm.rad2teeth(angles[1], 3)
    # joint4_angle = arm.rad2teeth(angles[2], 4)
    # arm.sycn_return_to_rest_new([joint1_angle, joint2_angle, joint3_angle, joint4_angle])

    """
    Here is the test for ik_solver
    """

    angles, _ = ks.ik_solver([math.sqrt(0.2 ** 2 + 0.1 ** 2), 0.05], False)
    print('[INFO] angles sloved by ik:', angles)
    joint1_angle = rad2teeth(0, 2)
    joint2_angle = rad2teeth(angles[0], 2)
    joint3_angle = rad2teeth(angles[1], 3)
    joint4_angle = rad2teeth(angles[2], 4)
    arm.move_new_rtst([joint1_angle, joint2_angle, joint3_angle, joint4_angle])

    time.sleep(3)

    """
    Here is the test for start_to_end
    """

    solved_angles = ks.star_to_des_solver([0.2, 0.1, 0.075], [0.3, -0.1, 0.075], False)
    print('[INFO] solved_angles:', solved_angles)
    pick_solved_angles = solved_angles[0]
    base_solved_angle = solved_angles[1]
    move_2_solved_angles = solved_angles[2]
    place_solved_angles = solved_angles[3]

    # joint1_angle = arm.rad2teeth(0, 2)
    joint2_angle = rad2teeth(pick_solved_angles[0], 2)
    joint3_angle = rad2teeth(pick_solved_angles[1], 3)
    joint4_angle = rad2teeth(pick_solved_angles[2], 4)
    arm.move_new_rtst([joint1_angle, joint2_angle, joint3_angle, joint4_angle])

    time.sleep(3)

    joint1_angle = rad2teeth(base_solved_angle, 2)
    # joint2_angle = arm.rad2teeth(pick_solved_angles[0], 2)
    # joint3_angle = arm.rad2teeth(pick_solved_angles[1], 3)
    # joint4_angle = arm.rad2teeth(pick_solved_angles[2], 4)
    arm.move_new_rtst([joint1_angle, joint2_angle, joint3_angle, joint4_angle])

    time.sleep(3)

    # joint1_angle = arm.rad2teeth(base_solved_angle, 2)
    joint2_angle = rad2teeth(move_2_solved_angles[0], 2)
    joint3_angle = rad2teeth(move_2_solved_angles[1], 3)
    joint4_angle = rad2teeth(move_2_solved_angles[2], 4)
    arm.move_new_rtst([joint1_angle, joint2_angle, joint3_angle, joint4_angle])

    time.sleep(3)

    joint1_angle = rad2teeth(base_solved_angle, 2)
    joint2_angle = rad2teeth(place_solved_angles[0], 2)
    joint3_angle = rad2teeth(place_solved_angles[1], 3)
    joint4_angle = rad2teeth(place_solved_angles[2], 4)
    arm.move_new_rtst([joint1_angle, joint2_angle, joint3_angle, joint4_angle])

    time.sleep(10)
    arm.driver_disable()
    arm.close()


def test2():
    arm = Arm(port='COM3')
    arm.driver_enable()
    arm.move_new_rtst([512, 512, 512])
    time.sleep(1)
    arm.driver_disable()
    arm.close()


def move_from_to(from_position, to_position):
    # arm = Arm(port='COM3')
    arm = Arm(port='/dev/tty.usbserial-FT4THVJ7')

    arm.driver_enable()
    arm.set_driver_low_speed()

    # FIXME: reset

    solved_angles = ks.star_to_des_solver(from_position, to_position, False)
    print('solved angles:')
    need_use_gripper = 0  # trun to
    for solved_angle in solved_angles:
        print(solved_angle)
        joint1_angle = rad2teeth(solved_angle[0], 1)
        joint2_angle = rad2teeth(solved_angle[1], 2)
        joint3_angle = rad2teeth(solved_angle[2], 3)
        joint4_angle = rad2teeth(solved_angle[3], 4)
        if need_use_gripper == 1:
            print('pick up')
            arm.gripper.pickup()
        elif need_use_gripper == 4:
            print('drop off')
            arm.gripper.dropoff()
        arm.move_new_rtst([joint1_angle, joint2_angle, joint3_angle, joint4_angle])
        need_use_gripper += 1
        time.sleep(3)

    # FIXME: reset
    arm.driver_disable()
    arm.close()


def main():
    from_position = trans.transform_from_piece_inboard([0, 4], 'chess_KING')
    to_position = trans.transform_from_piece_inboard([0, 5], 'chess_KING')

    print('from:')
    print(from_position)
    print('to:')
    print(to_position)

    # from_position = [0, ks.l_2, ks.l_1]
    # to_position = [0, -ks.l_2, ks.l_1]
    move_from_to(from_position, to_position)  # FIXME: check that whether the minus value is supported


if __name__ == '__main__':
    main()
