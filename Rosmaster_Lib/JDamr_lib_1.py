import serial
import threading 
import struct
import time 
'''
Description
- This script includes motor control only. 
- Next script to do 
  - protocol define, parsing, receiving packet
  - car motion 
  - IMU - raw sensor value, pitch/roll/yaw, speed 
  - auto report 
  - software version 
  - PID control 
  - battery
'''

class JDamr(object):
    def __init__(self, com="COM12"):
        self.ser = serial.Serial(com, 115200)
        self._HEAD = 0xf5
        self.CMD_SET_MOTOR = 0x05
        self.CMD_GET_SPEED = 0x0A
        self.CMD_GET_ENCODER = 0xff

        if self.ser.isOpen():
            print("JDamr serial port opened!")
        else:
            print("Can't open JDamr serial port!")

        time.sleep(1)

    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        try:
            speed_a = bytearray(struct.pack('b', speed_1))
            speed_b = bytearray(struct.pack('b', speed_2))
            speed_c = bytearray(struct.pack('b', speed_3))
            speed_d = bytearray(struct.pack('b', speed_4))
            cmd = [self._HEAD, 0x00, self.CMD_SET_MOTOR,
                    speed_a[0], speed_b[0], speed_c[0], speed_d[0]]
            cmd[1] = len(cmd) - 1
            checksum = 0xff #sum(cmd) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            print("motor:", cmd)
            time.sleep(0.002)
        except:
            pass

if __name__ == '__main__':
    com = 'COM12'
    bot = JDamr(com)
    time.sleep(1)
   
    while True:
        bot.set_motor(100, 100, 100, 100)
        time.sleep(5)
        bot.set_motor(50,50,50,50)
        time.sleep(5)


