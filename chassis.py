#!/usr/bin/env python
import struct
import serial


class chassis:
    '''Chassis control protocol

    Speed control can be achieved now, and position control will be implemented in the future

    Args: string

    Returns: boot

    Attributes:
        speed: Speed control
        position: Position control
    '''

    def __init__(self, com) -> bool:
        self.com = serial.Serial(com, 115200)

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
        if self.com.write(data) == 14:
            return True
        else:
            return False

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
        if self.com.write(data) == 14:
            return True
        else:
            return False

    def shoot(self, x):
        '''Shoot control

        Args:
            x: The distance
        Returns:
            bool
        '''
        data = bytes('A:'.encode()) + struct.pack('fff', x, 0, 0)
        if self.com.write(data) == 14:
            return True
        else:
            return False

    def release(self):
        '''Release control'''
        data = bytes('0:'.encode()) + struct.pack('fff', 0, 0, 0)
        self.com.write(data)
        return self.com.close()

    def close(self):
        return self.com.close()


if __name__ == '__main__':
    import sys

    ch = chassis(sys.argv[1])
    ch.speed(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    ch.close()
