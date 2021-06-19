#!/usr/bin/env python
import struct
import serial


class chassis:
    '''Chassis control protocol

    Speed control can be achieved now, and position control will be implemented in the future

    Args:
        port: the port will be opend

    Returns: boot

    Attributes:
        speed: Speed control
        position: Position control
        shoot: shoot control
        release: release control
    '''

    def __init__(self, port) -> bool:
        self.port = port
        self.rate = 115200

    def _send(self, data):
        with serial.Serial(self.port, self.rate) as s:
            count = s.write(data)
        if count == 14:
            return True
        else:
            return False

    def speed(self, vx, vy, wz):
        '''Speed control

        Args:
            vx: The velocity on the X-axis, m/s
            vy: The velocity on the Y-axis, m/s
            wz: The angular velocity on the z axis, rad/s
        Returns:
            bool
        '''
        data = bytes('V:'.encode()) + struct.pack('fff', vx, vy, wz)
        return self._send(data)

    def position(self, x, y, z):
        '''Position control

        Args:
            x: The displacement on the X-axis, m
            y: The displacement on the Y-axis, m
            z: The angle on the Z-axis, rad
        Returns:
            bool
        '''
        data = bytes('P:'.encode()) + struct.pack('fff', x, y, z)
        return self._send(data)

    def shoot(self, x):
        '''Shoot control

        Args:
            x: The distance
        Returns:
            bool
        '''
        data = bytes('A:'.encode()) + struct.pack('fff', x, 0, 0)
        return self._send(data)

    def release(self):
        '''Release control'''
        data = bytes('0:'.encode()) + struct.pack('fff', 0, 0, 0)
        return self._send(data)


if __name__ == '__main__':
    import sys

    ch = chassis(sys.argv[1])
    ch.speed(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    while True:
        inlist = input(':').split()
        print(ch.speed(float(inlist[0]), float(inlist[1]), float(inlist[2])))
