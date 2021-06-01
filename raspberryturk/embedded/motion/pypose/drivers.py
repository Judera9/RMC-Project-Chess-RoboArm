#!/usr/bin/env python

import os
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from raspberryturk.embedded.motion.dynamixel_sdk import *
from raspberryturk.embedded.motion.pypose.ax12 import *


class Drivers:
    """ Class to open a serial port and control AX-12 servos
    through an arbotiX board or USB Dynamixel. """
    # port Windows: 'COM1' Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
    def __init__(self, port="/dev/ttyUSB0", baud=1000000):
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
        self.error = 0
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        if self.portHandler.setBaudRate(baud):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()
        time.sleep(3)

    # Enable Dynamixel Torque
    def torque_enable(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel %d has been successfully connected" % dxl_id)

    def torque_disable(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def close_port(self):
        # Close port
        self.portHandler.closePort()

    """only can read 2byte"""
    def read_2byte(self, dxl_id, address):
        result, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, address)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return result

    def read_1byte(self, dxl_id, address):
        result, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, address)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return result

    def read_present_position(self, dxl_id):
        dxl_present_position = self.read_2byte(dxl_id, ADDR_MX_PRESENT_POSITION)
        return dxl_present_position

    def set_goal_position(self, dxl_id, goal_position):
        self.setReg(dxl_id, ADDR_MX_GOAL_POSITION, goal_position)
        while 1:
            dxl_present_position = self.read_present_position(dxl_id)
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (dxl_id, goal_position, dxl_present_position))
            if not abs(goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                break

    def set_goal_speed(self, dxl_id, speed):
        self.setReg(dxl_id, P_GOAL_SPEED_L, speed)

    def read_present_speed(self, dxl_id):
        dxl_present_speed = self.read_2byte(dxl_id, P_GOAL_SPEED_L)
        return dxl_present_speed

    """only can write 2bytes"""
    def setReg(self, dxl_id, regstart, values):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, regstart, values)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    """Add Dynamixel goal position value to the Syncwrite parameter storage"""
    def moter_addparam(self, dxl_id, position_goal):
        dxl_addparam_result = self.groupSyncWrite.addParam(dxl_id, position_goal)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
            quit()

    """vals[0,1]       0:dxl_id,   1:goal_position  """
    def syncwrite_more_goal_position(self, ids, vals):
        list = [0, 1, 2, 3, 4, 5]
        param_goal_position = [[DXL_LOBYTE(vals[i]), DXL_HIBYTE(vals[i])] for i in list]

        # Add Dynamixel#1-3 goal position value to the Syncwrite parameter storage
        for i in list:
            self.moter_addparam(ids[i], param_goal_position[i])

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        while 1:
            # Read Dynamixel#1 present position
            dxl_present_position = [self.read_present_position(ids[i]) for i in list]

            for i in list:
                print("[ID:%03d] GoalPos:%03d PresPos:%03d" % (ids[i], vals[i], dxl_present_position[i]))

            if not any([(abs(vals[i] - dxl_present_position[i]) > DXL_MOVING_STATUS_THRESHOLD) for i in list]):
                break

    """vals[0,1]       0:dxl_id,   1:goal_position  """
    def syncwrite_goal_position(self, vals1, vals2):
        param_goal_position1 = [DXL_LOBYTE(vals1[1]), DXL_HIBYTE(vals1[1])]
        param_goal_position2 = [DXL_LOBYTE(vals2[1]), DXL_HIBYTE(vals2[1])]

        # Add Dynamixel#1-3 goal position value to the Syncwrite parameter storage
        self.moter_addparam(vals1[0], param_goal_position1)
        self.moter_addparam(vals2[0], param_goal_position2)

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        while 1:
            # Read Dynamixel#1 present position
            dxl1_present_position = self.read_present_position(vals1[0])
            dxl2_present_position = self.read_present_position(vals2[0])

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (vals1[0], vals1[1], dxl1_present_position, vals2[0], vals2[1], dxl2_present_position))
            if (not (abs(vals1[1] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD)) and (not (abs(vals2[1] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

    def test1(self):
        self.torque_enable(12)
        self.torque_enable(7)
        self.torque_enable(3)
        self.set_goal_position(3, 0)
        time.sleep(1)
        self.set_goal_position(7, 0)
        time.sleep(1)
        self.set_goal_position(12, 0)
        time.sleep(1)
        self.syncwrite_goal_position([3, 512], [7, 512])
        time.sleep(1)
        self.syncwrite_goal_position([3, 1023], [12, 512])
        time.sleep(1)
        self.syncwrite_goal_position([7, 1023], [12, 1023])
        time.sleep(3)
        self.torque_enable(12)
        self.torque_disable(7)
        self.torque_disable(3)
        self.close_port()

    def test2(self):
        self.torque_enable(7)
        self.torque_enable(3)
        self.syncwrite_goal_position([3, 512], [7, 512])
        self.torque_disable(7)
        self.torque_disable(3)
        self.close_port()

    def test3(self):
        self.torque_enable(12)
        self.set_goal_speed(12, LOW_SPEED)
        self.set_goal_position(12, 0)
        self.set_goal_position(12, 1023)
        self.torque_disable(12)
        self.close_port()


def main():
    driver = Drivers(port="COM3")
    driver.test1()


if __name__ == '__main__':
    main()


