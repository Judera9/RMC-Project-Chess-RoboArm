#!/usr/bin/env python

import serial
import time
import logging
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

from dynamixel_sdk import *
from ax12 import *


class Drivers:
    """ Class to open a serial port and control AX-12 servos
    through an arbotiX board or USB Dynamixel. """
    # port Windows: 'COM1' Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
    def __init__(self, port="/dev/ttyUSB0", baud=1000000):
        """ This may throw errors up the line -- that's a good thing. """
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
            print("Dynamixel has been successfully connected")

    def torque_disable(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def close_port(self):
        # Close port
        self.portHandler.closePort()

    def read_present_position(self, dxl_id):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_present_position

    def setReg(self, dxl_id, regstart, values):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, regstart, values)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        while 1:
            dxl_present_position = self.read_present_position(dxl_id)
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (dxl_id, values, dxl_present_position))
            if not abs(values - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                break

    # Add Dynamixel goal position value to the Syncwrite parameter storage
    def moter_addparam(self, dxl_id, position_goal):
        dxl_addparam_result = self.groupSyncWrite.addParam(dxl_id, position_goal)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
            quit()

    def syncWrite(self, regstart, vals1, vals2):
        param_goal_position1 = [DXL_LOBYTE(vals1[1]), DXL_HIBYTE(vals1[1])]
        param_goal_position2 = [DXL_LOBYTE(vals2[1]), DXL_HIBYTE(vals2[1])]

        # Add Dynamixel#1-3 goal position value to the Syncwrite parameter storage
        self.moter_addparam(vals1[0], param_goal_position1)
        self.moter_addparam(vals2[0], param_goal_position2)

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()
        while 1:
            # Read Dynamixel#1 present position
            dxl1_present_position = self.read_present_position(vals1[0])
            dxl2_present_position = self.read_present_position(vals2[0])

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (vals1[0], vals1[1], dxl1_present_position, vals2[0], vals2[1], dxl2_present_position))
            if not ((abs(vals1[1] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(vals2[1] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break


def main():
    driver = Drivers(port="COM3")
    driver.torque_enable(12)
    # driver.torque_enable(7)
    # driver.torque_enable(3)
    driver.setReg(12, P_GOAL_POSITION_L, 512)
    # time.sleep(1)
    # driver.syncWrite(P_GOAL_POSITION_L, [3, 512], [7, 512])
    driver.torque_disable(12)
    # driver.torque_disable(7)
    # driver.torque_disable(3)
    driver.close_port()


if __name__ == '__main__':
    main()


