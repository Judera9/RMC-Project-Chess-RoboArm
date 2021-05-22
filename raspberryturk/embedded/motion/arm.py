# import time
# import serial
import numpy as np
from pytweening import easeInOutQuint, easeOutSine
from scipy.misc import derivative
from scipy.interpolate import interp1d
from raspberryturk.embedded.motion.arm_movement_engine import ArmMovementEngine
from pypose.ax12 import *
from pypose.driver import Driver

SERVO_1 = 16
SERVO_2 = 10
SERVO_3 = 15  # combined with SERVO_2
SERVO_4 = 2
SERVO_5 = 17  # combined with SERVO_4
SERVO_6 = 11
# 这里可以打包两组电机
SERVOS = [SERVO_1, SERVO_2, SERVO_4, SERVO_6 ]
MIN_SPEED = 20
MAX_SPEED = 80
RESTING_POSITION = (512, 512)

# 输入一个list，返回list的第一位，和第二位二进制向左移8个0后对应的10进制的数
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
        self.driver = Driver(port=port)
        self.movement_engine = ArmMovementEngine()

    def close(self):
        self.driver.close()  # 也不知道该咋改

    def recenter(self):
        self.move((512, 512))

    def return_to_rest(self):  # position where dead pieces rest ?
        self.move_to_point([20, 13.5])

    def return_to_rest_new(self):  # position where dead pieces rest ?
        self.move_new((512, 512, 512, 512))

    def move_new(self, goal_position):
        # start_position = self.current_position()
        self.set_speed([MIN_SPEED, MIN_SPEED])
        for i in SERVOS:  # 遍历电机运动，打包两对电机   QAQ注意舵盘反向问题，改[i % 2]这个
            if i == SERVO_1:
                self.driver.setReg(i, P_GOAL_POSITION_L, goal_position[0])
            elif i == SERVO_2:
                self.driver.syncWrite(P_GOAL_POSITION_L, [SERVO_4, goal_position[1]], [SERVO_5, goal_position[1]])
            elif i == SERVO_4:
                self.driver.syncWrite(P_GOAL_POSITION_L, [SERVO_4, goal_position[2]], [SERVO_5, goal_position[2]])
            elif i == SERVO_6:
                self.driver.setReg(i, P_GOAL_POSITION_L, goal_position[3])
            '''while self._is_moving():  # 控制运动速度变化
                position = self.current_position()
                speed = [_adjusted_speed(start_position[i % 2], goal_position[i % 2], position[i % 2]) for i in SERVOS]
                self.set_speed(speed)'''

    def move_new1(self, goal_position):
        # start_position = self.current_position()
        self.set_speed([MIN_SPEED, MIN_SPEED])
        for i in SERVOS:  # 遍历电机运动，打包两对电机   QAQ注意舵盘反向问题，改[i % 2]这个
            if i == SERVO_2:
                self.driver.syncWrite(P_GOAL_POSITION_L, [[SERVO_2, goal_position[1] % 256, goal_position[1] >> 8],
                                      [SERVO_3, goal_position[1] % 256, goal_position[1] >> 8]])
            elif i == SERVO_4:
                self.driver.syncWrite(P_GOAL_POSITION_L, [[SERVO_4, goal_position[2] % 256, goal_position[2] >> 8],
                                      [SERVO_5, goal_position[2] % 256, goal_position[2] >> 8]])
            elif i == SERVO_1:
                self.driver.setReg(i, P_GOAL_POSITION_L, [goal_position[0] % 256, goal_position[0] >> 8])
            elif i == SERVO_6:
                self.driver.setReg(i, P_GOAL_POSITION_L, [goal_position[3] % 256, goal_position[3] >> 8])
            '''while self._is_moving():  # 控制运动速度变化
                position = self.current_position()
                speed = [_adjusted_speed(start_position[i % 2], goal_position[i % 2], position[i % 2]) for i in SERVOS]
                self.set_speed(speed)'''

    def move(self, goal_position):
        start_position = self.current_position()
        self.set_speed([MIN_SPEED, MIN_SPEED])  # input 2 MIN_SPEED here ?
        # 根据坐标旋转底部电机对准角度
        # self.driver.setReg(1,P)

        # self.driver.setReg(2,P)
        # self.driver.setReg(3,P)

        # self.driver.setReg(4,P)
        # self.driver.setReg(5,P)

        # self.driver.setReg(6,P)
        # 保持执行器末端z轴不变运动到棋子上方
        for i in SERVOS:  # 遍历电机运动，这里需要打包两对电机？ 目前只能动俩个(组)舵机
            if i == SERVO_2:
                self.driver.setReg(i, P_GOAL_POSITION_L, [goal_position[i % 2] % 256, goal_position[i % 2] >> 8])
                self.driver.setReg(SERVO_3, P_GOAL_POSITION_L, [goal_position[i % 2] % 256, goal_position[i % 2] >> 8])
                # 需要反向
            elif i == SERVO_4:
                self.driver.setReg(i, P_GOAL_POSITION_L, [goal_position[i % 2] % 256, goal_position[i % 2] >> 8])
                self.driver.setReg(SERVO_5, P_GOAL_POSITION_L, [goal_position[i % 2] % 256, goal_position[i % 2] >> 8])
                # 需要反向
            else:
                self.driver.setReg(i, P_GOAL_POSITION_L, [goal_position[i % 2] % 256, goal_position[i % 2] >> 8])
        while self._is_moving():  # 控制运动速度变化
            position = self.current_position()
            speed = [_adjusted_speed(start_position[i % 2], goal_position[i % 2], position[i % 2]) for i in SERVOS]
            self.set_speed(speed)

    def move_to_point_new(self, pt, piece_type):   # 将（x,y）+ piece_type 转化为关节的角度
        goal_position = self.movement_engine.convert_point_new(pt, piece_type)  # 在此改逆运动学
        self.move_new(goal_position)

    def move_to_point(self, pt):   # 在二维坐标系中，将（x,y）转化为俩个关节的角度
        goal_position = self.movement_engine.convert_point(pt)  # 在此改逆运动学
        self.move(goal_position)

    def set_speed(self, speed):    # 目前只能动俩个(组)舵机，但不必用到
        for i in SERVOS:
            self.driver.setReg(i, P_GOAL_SPEED_L, [speed[i % 2] % 256, speed[i % 2] >> 8])

    def current_position(self):
        return self._values_for_register(P_PRESENT_POSITION_L)

    def _is_moving(self):
        return any([self.driver.getReg(index, P_MOVING, 1) == 1 for index in SERVOS])

    def _values_for_register(self, register):
        return [_register_bytes_to_value(self.driver.getReg(index, register, 2)) for index in SERVOS]


def main():
    arm = Arm(port='COM3')
    arm.move(512)


if __name__ == '__main__':
    main()

