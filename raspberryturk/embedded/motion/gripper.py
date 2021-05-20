import RPi.GPIO as GPIO
from time import sleep
import chess

electromagnet_pin = 40  #选择控制电磁铁gpio引脚编号
servo_pin = 38   

PIECE_HEIGHTS = {  # changed according to the chess height (mm)
    chess.KING: 66,
    chess.QUEEN: 56,
    chess.ROOK: 37,
    chess.BISHOP: 48,
    chess.KNIGHT: 43,
    chess.PAWN: 30,
}

MAX_PIECE_HEIGHT = max(PIECE_HEIGHTS.values())
RESTING_HEIGHT = MAX_PIECE_HEIGHT + 15   # set rest size considering the length of metal rod ?


class Gripper(object):
    def __init__(self):
        self.previous_z = None
        GPIO.setmode(GPIO.BOARD)  #设置引脚编号规则
        GPIO.setup(servo_pin, GPIO.OUT)  #设置为输出
        GPIO.setup(electromagnet_pin, GPIO.OUT)

    def calibrate(self):    # reset to the rest height
        self.move(RESTING_HEIGHT)

    def move(self, z):  # Gripper move along Z axis，We do not have this DOF 末端舵机（我们pro没有）
        z = max(0.0, min(z, 100.0))
        dc = (z * 0.067) + 4.0
        p = GPIO.PWM(servo_pin, 50.0)   #将38号引脚初始化为PWM实例，频率为50Hz
        p.start(dc)  #开始脉宽调制，用于控制舵机速度
        if self.previous_z is None:
            t = 10.0
        else:
            t = (abs(self.previous_z - z) / 10.0) + 0.5
        sleep(t)
        p.stop()
        del p
        self.previous_z = z

    def electromagnet(self, on):    # Electromagnet state switching
        output = GPIO.HIGH if on else GPIO.LOW
        GPIO.output(electromagnet_pin, output)

    def pickup_new(self):
        self.electromagnet(True)
        sleep(0.2)

    def dropoff_new(self):
        self.electromagnet(False)
        sleep(0.4)

    def pickup(self, piece_type):
        piece_height = PIECE_HEIGHTS[piece_type]
        self.move(piece_height)  # 不用移动，根据高度需建立逆运动学
        sleep(0.4)
        self.electromagnet(True)
        sleep(0.2)
        self.move(RESTING_HEIGHT + piece_height)

    def dropoff(self, piece_type):
        piece_height = PIECE_HEIGHTS[piece_type]
        self.move(piece_height)
        sleep(0.2)
        self.electromagnet(False)
        sleep(0.4)
        self.move(RESTING_HEIGHT)

    def cleanup(self):
        GPIO.cleanup()
