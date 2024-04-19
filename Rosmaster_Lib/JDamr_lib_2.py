import serial
import threading 
import struct
import time 
'''
Description
- JDamr_lib1.py script has motor control code.
- This script has protocol define, parsing, receiving paket 
  - example: encoder 
- Next script to do 
  - car motion
  - IMU - raw sensor value, pitch/roll/yaw, speed 
  - auto report 
  - software version 
  - PID control 
  - battery
'''

class JDamr(object):
    def __init__(self, com="/dev/ttyACM0"):
        self.ser = serial.Serial(com, 115200)
        self.HEAD = 0xf5
        self.CMD_SET_MOTOR = 0x05
        self.CMD_GET_SPEED = 0x0A
        self.CMD_GET_ENCODER = 0x20

        if self.ser.isOpen():
            print("JDamr serial port opened!")
        else:
            print("Can't open JDamr serial port!")

        time.sleep(1)

    '''
    Protocol 
    - Packets have following bytes.
      - Header byte 
      - length byte
      - command byte
      - payload bytes 
      - checksum byte 
    '''
    def receive_data(self):     
        self.ser.flushInput()
        while True:
            head = bytearray(self.ser.read())[0]
            if head == self.HEAD:
                length = bytearray(self.ser.read())[0]  
                payload = [] 
                for i in range(length-1):
                    value = bytearray(self.ser.read())[0]
                    payload.append(value)
                self.parse_cmd(payload)

    def receive_thread(self):
        try:
            taks_name = "serial_thread"
            rx_task = threading.Thread(target=self.receive_data, name=taks_name)
            rx_task.setDaemon(True)
            rx_task.start()
            print("Start serial receive thread ")
            time.sleep(.05)
        except:
            pass

    def parse_cmd(self, payload):
        if self.CMD_GET_ENCODER == payload[0]:
            encode1_str = payload[1:5]
            print(encode1_str)
            encode1 = int.from_bytes(encode1_str, "big")
            print(encode1)

    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        try:
            speed_a = bytearray(struct.pack('b', speed_1))
            speed_b = bytearray(struct.pack('b', speed_2))
            speed_c = bytearray(struct.pack('b', speed_3))
            speed_d = bytearray(struct.pack('b', speed_4))
            cmd = [self.HEAD, 0x00, self.CMD_SET_MOTOR,
                    speed_a[0], speed_b[0], speed_c[0], speed_d[0]]
            cmd[1] = len(cmd) - 1
            checksum = 0xff #sum(cmd) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            print("motor:", cmd)
            time.sleep(0.002)
        except:
            print("set_motor error")
            pass

if __name__ == '__main__':
    com = '/dev/ttyACM0'
    bot = JDamr(com)
    time.sleep(1)
    bot.receive_thread()
   
    while True:
        bot.set_motor(100, 100, 100, 100)
        time.sleep(5)
        bot.set_motor(50,50,50,50)
        time.sleep(5)


