#!/usr/bin/env python

"""
  PyPose: Serial driver for connection to arbotiX board or USBDynamixel.
  Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 
"""

# This is an adapted version of the driver.py file found in
# the pypose project (https://github.com/vanadiumlabs/pypose).
# serial 是外界接口相关的函数
# time 待查

import serial
import time
from ax12 import *
import logging


class Driver:
    """ Class to open a serial port and control AX-12 servos
    through an arbotiX board or USB Dynamixel. """
    # port Windows: 'COM1' Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
    def __init__(self, port="/dev/ttyUSB0", baud=38400, interpolation=False, direct=False, verbose=False):
        """ This may throw errors up the line -- that's a good thing. """
        self.ser = serial.Serial()
        self.ser.baudrate = baud  # 波特率
        self.ser.port = port  # 读或写端口
        self.ser.timeout = 0.5  # 读超时设置
        self.ser.open()  # 打开端口
        self.error = 0
        self.hasInterpolation = interpolation  # 查不到
        self.direct = direct  # 没查到
        self.logger = logging.getLogger(__name__)
        time.sleep(3)

    def execute(self, index, ins, params):
        """ Send an instruction to a device. """
        self.ser.flushInput()  # 丢弃接收缓存中的所有数据。
        length = 2 + len(params)
        checksum = 255 - ((index + length + ins + sum(params)) % 256)  # 检测总的值有没有超过255
        self.ser.write((chr(0xFF) + chr(0xFF) + chr(index) + chr(length) + chr(ins)).encode(encoding="utf-8"))  # 向端口写数据。
        for val in params:
            self.ser.write(chr(val).encode(encoding="utf-8"))   # 写入需要的参数
        self.ser.write(chr(checksum).encode(encoding="utf-8"))
        return self.getPacket(0)

    def setReg(self, index, regstart, values):
        """ Set the value of registers. Should be called as such:
        ax12.setReg(1,1,(0x01,0x05)) """
        self.execute(index, AX_WRITE_DATA, [regstart] + values)  # AX_WRITE_DATA 来自 ax12.py regstart 是来自
        return self.error

    def getPacket(self, mode, id=-1, leng=-1, error=-1, params=None):
        """ Read a return packet, iterative attempt """
        # need a positive byte
        d = self.ser.read().decode(encoding="utf-8")  # 从端口读字节数据。默认1个字节。
        if d == '':
            self.logger.debug("Fail Read")
            return None

        # now process our byte
        if mode == 0:  # get our first 0xFF：0x代表16进制数,0xff表示的数二进制1111 1111 占一个字节.和其进行&操作的数,最低8位,不会发生变化.
            if ord(d) == 0xff:  # ord ASII码
                self.logger.debug("Oxff found")
                return self.getPacket(1)
            else:
                self.logger.debug("Oxff NOT found, restart: " + str(ord(d)))
                return self.getPacket(0)
        elif mode == 1:  # get our second 0xFF
            if ord(d) == 0xff:
                self.logger.debug("Oxff found")
                return self.getPacket(2)
            else:
                self.logger.debug("Oxff NOT found, restart: " + str(ord(d)))
                return self.getPacket(0)
        elif mode == 2:  # get id
            if d != 0xff:
                self.logger.debug("ID found: " + str(ord(d)))
                return self.getPacket(3, ord(d))
            else:
                self.logger.debug("0xff is not ID, restart")
                return self.getPacket(0)
        elif mode == 3:  # get length
            self.logger.debug("Length found: " + str(ord(d)))
            return self.getPacket(4, id, ord(d))
        elif mode == 4:  # read error
            self.logger.debug("Error level found: " + str(ord(d)))
            self.error = ord(d)
            if leng == 2:
                return self.getPacket(6, id, leng, ord(d), list())
            else:
                return self.getPacket(5, id, leng, ord(d), list())
        elif mode == 5:  # read params
            self.logger.debug("Parameter found: " + str(ord(d)))
            params.append(ord(d))
            if len(params) + 2 == leng:
                return self.getPacket(6, id, leng, error, params)
            else:
                return self.getPacket(5, id, leng, error, params)
        elif mode == 6:  # read checksum
            self.logger.debug("Checksum found: " + str(ord(d)))
            checksum = id + leng + error + sum(params) + ord(d)
            self.logger.debug("Checksum computed: " + str(checksum))
            if checksum % 256 != 255:
                self.logger.debug("Checksum ERROR")
                return None
            return params
        # fail
        return None

    def getReg(self, index, regstart, rlength):
        """ Get the value of registers, should be called as such:
        ax12.getReg(1,1,1) """
        vals = self.execute(index, AX_READ_DATA, [regstart, rlength])  # vals=getPacket(0),也就是一直迭代？
        if vals is None:
            self.logger.debug("Read Failed: Servo ID = " + str(index))
            return -1
        if rlength == 1:
            return vals[0]
        return vals

    def syncWrite(self, regstart, vals):
        """ Set the value of registers. Should be called as such:
        ax12.syncWrite(reg, ((id1, val1, val2), (id2, val1, val2))) """
        self.ser.flushInput()
        length = 4
        valsum = 0
        for i in vals:
            length = length + len(i)
            valsum = valsum + sum(i)
        checksum = 255 - ((254 + length + AX_SYNC_WRITE + regstart + len(vals[0]) - 1 + valsum) % 256)
        # packet: FF FF ID LENGTH INS(0x03) PARAM .. CHECKSUM
        self.ser.write(chr(0xFF) + chr(0xFF) + chr(0xFE) + chr(length) + chr(AX_SYNC_WRITE) + chr(regstart) + chr(
            len(vals[0]) - 1))
        for servo in vals:
            for value in servo:
                self.ser.write(chr(value))
        self.ser.write(chr(checksum))
        # no return info...


def main():
    driver = Driver(port='COM3')
    driver.setReg(12, P_GOAL_POSITION_L, [512 % 256, 512 >> 8])



if __name__ == '__main__':
    main()
