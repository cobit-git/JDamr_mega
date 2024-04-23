#include<Encoder.h>

#define HEAD              0xf5
#define CMD_SET_MOTOR     0x01
#define CMD_GET_SPEED     0x02
#define CMD_GET_ENCODER   0x03
#define CMD_CAR_RUN       0x04
/*
import serial
import threading 
import struct
import time 
'''
Description
- JDamr_lib1.py script has motor control code.
- This script has protocol define, parsing, receiving paket 
  - example: encoder 
  - car motion
  - encoder periodic read 
- Next script to do 
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
        self.CMD_SET_MOTOR = 0x01
        self.CMD_GET_SPEED = 0x02
        self.CMD_GET_ENCODER = 0x03
        self.CMD_CAR_RUN = 0x04

        self.encoder1 = 0
        self.encoder2 = 0
        self.encoder3 = 0
        self.encoder4 = 0

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
            print(payload)
            encode1_str = payload[1:5]
            encode2_str = payload[5:9]
            encode3_str = payload[9:13]
            encode4_str = payload[13:17]
            self.encode1 = int.from_bytes(encode1_str, byteorder="big")
            print(self.encode1)
            self.encode2 = int.from_bytes(encode2_str, byteorder="big")
            print(self.encode2)
            self.encode3 = int.from_bytes(encode3_str, byteorder="big")
            print(self.encode3)
            self.encode4 = int.from_bytes(encode4_str, byteorder="big")
            print(self.encode4)

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
            time.sleep(0.1)
        except:
            print("set_motor error")
            pass
    
    '''
    drive_mode: 
    1: go foreward 
    2: go backward 
    3: turn left 
    4: turn right 
    speed: 1 ~ 100 
    '''
    def set_car_run(self, drive_mode, speed):
        try:
            speed_0 = bytearray(struct.pack('b', speed))
            drive_mode_0 = bytearray(struct.pack('b', drive_mode))
            cmd = [self.HEAD, 0x00, self.CMD_CAR_RUN, drive_mode_0[0], speed_0[0]]
            cmd[1] = len(cmd) - 1
            checksum = 0xff #sum(cmd) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            print("car_run:", cmd)
            time.sleep(0.1)
        except:
            print('---set_car_run error!---')
            pass


if __name__ == '__main__':
    com = '/dev/ttyACM0'
    bot = JDamr(com)
    time.sleep(1)
    bot.receive_thread()
   
    while True:
        # CMD_SET_MOTOR test 
        #bot.set_motor(100, 100, 100, 100)
        #time.sleep(1)
        #bot.set_motor(50,50,50,50)
        #time.sleep(1)
        # CMD_CAR_RUN test 
        bot.set_car_run(1, 100) 
        time.sleep(5)
        bot.set_car_run(1, 50) 
        time.sleep(5)
    


*/

class SPDMotor {
  public:
  SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 );

  /// Set the PWM speed and direction pins.
  /// pwm = 0, stop (no active control)
  /// pwm = 1 to 255, proportion of CCW rotation
  /// pwm = -1 to -255, proportion of CW rotation
  void speed( int pwm );

  /// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
  void hardStop();

  /// Get the current speed.
  int getSpeed();

  /// Get the current rotation position from the encoder.
  long getEncoderPosition();

  private:
    Encoder *_encoder;
    bool _encoderReversed;
    int _motorPWM, _motorDir1, _motorDir2;

    // Current speed setting.
    int _speed;
};

SPDMotor::SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 ) {
  _encoder = new Encoder(encoderA, encoderB);
  _encoderReversed = encoderReversed;

  _motorPWM = motorPWM;
  pinMode( _motorPWM, OUTPUT );
  _motorDir1 = motorDir1;
  pinMode( _motorDir1, OUTPUT );
  _motorDir2 = motorDir2;
  pinMode( _motorDir2, OUTPUT );
}

/// Set the PWM speed and direction pins.
/// pwm = 0, stop (no active control)
/// pwm = 1 to 255, proportion of CCW rotation
/// pwm = -1 to -255, proportion of CW rotation
void SPDMotor::speed( int speedPWM ) {
  _speed = speedPWM;
  if( speedPWM == 0 ) {
    digitalWrite(_motorDir1,LOW);
    digitalWrite(_motorDir2,LOW);
    analogWrite( _motorPWM, 255);
  } else if( speedPWM > 0 ) {
    digitalWrite(_motorDir1, LOW );
    digitalWrite(_motorDir2, HIGH );
    analogWrite( _motorPWM, speedPWM < 255 ? speedPWM : 255);
  } else if( speedPWM < 0 ) {
    digitalWrite(_motorDir1, HIGH );
    digitalWrite(_motorDir2, LOW );
    analogWrite( _motorPWM, (-speedPWM) < 255 ? (-speedPWM): 255);
  }
}

/// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
void SPDMotor::hardStop() {
    _speed = 0;
    digitalWrite(_motorDir1,HIGH);
    digitalWrite(_motorDir2,HIGH);
    analogWrite( _motorPWM, 0);
}

/// Get the current speed.
int SPDMotor::getSpeed() {
    return _speed;
}

/// Get the current rotation position from the encoder.
long SPDMotor::getEncoderPosition() {
  long position = _encoder->read();
  return _encoderReversed ? -position : position;
}

SPDMotor *motorLF = new SPDMotor(18, 31, true, 12, 35, 34); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRF = new SPDMotor(19, 38, false, 8, 37, 36); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor *motorLR = new SPDMotor( 3, 49, true,  6, 42, 43); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRR = new SPDMotor( 2, A1, false, 5, A4, A5); // <- NOTE: Motor Dir pins reversed for opposite operation


void motor_control(int lf_, int lr_, int rf_, int rr_){
  motorLF->speed(lf_); 
  motorLR->speed(lr_); 
  motorRF->speed(rf_);
  motorRR->speed(rr_);
}

void read_encoder(){
  // arduino integer data type - 2 bytes!!!
  long lf_encoder = motorLF->getEncoderPosition();  // 22222
  long rf_encoder = motorRF->getEncoderPosition();  // 33333
  long lr_encoder = motorLR->getEncoderPosition();  // 44444
  long rr_encoder = motorRR->getEncoderPosition();  // 55555
  Serial2.println(lf_encoder);
  Serial2.println(rf_encoder);
  Serial2.println(lr_encoder);
  Serial2.println(rr_encoder);
  byte data_buffer[20] = {0};
  byte i, checknum = 0;
  data_buffer[0] = HEAD;                          // HEADER 
  data_buffer[1] = 19;                            // length 
  data_buffer[2] = CMD_GET_ENCODER;               // command 
  data_buffer[3] = (lf_encoder >> 24) & 0xff;     // payload 
  data_buffer[4] = (lf_encoder >> 16) & 0xff;
  data_buffer[5] = (lf_encoder >> 8) & 0xff;
  data_buffer[6] = lf_encoder & 0xff;
  data_buffer[7] = (rf_encoder >> 24) & 0xff;
  data_buffer[8] = (rf_encoder >> 16) & 0xff;
  data_buffer[9] = (rf_encoder >> 8) & 0xff;
  data_buffer[10] = rf_encoder & 0xff;
  data_buffer[11] = (lr_encoder >> 24) & 0xff;
  data_buffer[12] = (lr_encoder >> 16) & 0xff;
  data_buffer[13] = (lr_encoder >> 8) & 0xff;
  data_buffer[14] = lr_encoder  & 0xff;
  data_buffer[15] = (rr_encoder >> 24) & 0xff;
  data_buffer[16] = (rr_encoder >> 16) & 0xff;
  data_buffer[17] = (rr_encoder >> 8) & 0xff;
  data_buffer[18] = rr_encoder & 0xff;

  for (i = 2; i < 19; i++)
  {
    checknum += data_buffer[i];
  }
  data_buffer[19] = checknum;                     // checksum
  Serial.write(data_buffer, 20);
}

#define GO_FORWARD    1
#define GO_BACKWARD   2
#define TURN_LEFT     3
#define TURN_RIGHT    4
#define STOP          5
void car_run(byte state, byte speed){
  if(state == GO_FORWARD){
    motor_control(speed, speed, speed, speed);
  }else if(state == GO_BACKWARD){
    motor_control(-speed, -speed, -speed, -speed);
  }else if(state == TURN_LEFT){
    motor_control(speed, speed, -speed, -speed);
  }else if(state == TURN_RIGHT){
    motor_control(-speed, -speed, speed, speed);
  }else if(state == STOP){
    motor_control(0, 0, 0, 0);
  }
}

void byte_to_hex(byte a){
  char hexString[3];
  sprintf(hexString, "%02X", a);
  Serial2.println(hexString);
}


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial2.println("hello...");
}



void loop() {
   if(Serial.available() > 0){
    // Header byte 
    byte c = Serial.read();   //HEAD
    byte_to_hex(c);
    if( c == HEAD){
      c = Serial.read();      //length 
      byte_to_hex(c);
      c = Serial.read();        //command
      byte_to_hex(c);
      if(c == CMD_SET_MOTOR){
        byte  lf = Serial.read();    //speed 1 
        byte_to_hex(lf);
        byte rf = Serial.read();    //speed 2
        byte_to_hex(rf);
        byte lr = Serial.read();    //speed 3
        byte_to_hex(lr);
        byte rr = Serial.read();    //speed 4  
        byte_to_hex(rr);
        motor_control(lf, rf, lr, rr);
        c = Serial.read();  
        byte_to_hex(c);
        Serial2.print(lf);
        Serial2.print(lr);
        Serial2.print(rf);
        Serial2.println(rr);
        
      }else if(c == CMD_GET_SPEED){
        
      }else if(c == CMD_GET_ENCODER){
        
      }else if(c == CMD_CAR_RUN){
        byte  state = Serial.read();    // state 
        byte_to_hex(state);
        byte  speed = Serial.read();    // speed 
        byte_to_hex(speed);
        car_run(state, speed);
     }
    }
  }
  delay(500);
  read_encoder();
}
