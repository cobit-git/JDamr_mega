#include<Encoder.h>
/*
 install "Encoder" by Paul Stoffregen v1.4.1
 */
/*
This code can receive hex string for motor control from PC through Serial. 
You can send hex string using python code beloww. 

import serial
import threading 
import struct
import time 

class JDamr(object):
    def __init__(self, com="COM12"):
        self.ser = serial.Serial(com, 115200)
        self._HEAD = 0xf5
        self.CMD_SET_MOTOR = 0x05

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


void motor_control(){
  char c = ' ';
  if(Serial.available() > 0){
    // Header byte 
    c = Serial.read();
    unsigned int d = c & 0xFF;
    Serial2.print(d);
    Serial2.print(' ');
    // lenght of command packet 
    c = Serial.read();
    d = c & 0xFF;
    Serial2.print(d);
    Serial2.print(' ');
    // command byte 
    c = Serial.read();
    d = c & 0xFF;
    Serial2.print(d);
    Serial2.print(' ');
    // LF
    c = Serial.read();
    int lf = c & 0xFF;
    Serial2.print(d);
    Serial2.print(' ');
    // LR
    c = Serial.read();
    int lr = c & 0xFF;
    Serial2.print(d);
    Serial2.print(' ');
    // RF
    c = Serial.read();
    int rf = c & 0xFF;
    Serial2.print(d);
    Serial2.print(' ');
    // RR
    c = Serial.read();
    int rr = c & 0xFF;
    Serial2.print(d);
    Serial2.print(' ');
    // checksum 
    c = Serial.read();
    d = c & 0xFF;
    Serial2.print(d);
    Serial2.println(' ');

    motorLF->speed(lf); 
    motorLR->speed(lr); 
    motorRF->speed(rf);
    motorRR->speed(rr);
   
  }
}


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial2.println("hello...");
}

void loop() {
  
  motor_control();
  int lf_encoder = 56432; //motorLF->getEncoderPosition();
  int rf_encoder = 2; //motorRF->getEncoderPosition();
  int lr_encoder = 3; //motorLR->getEncoderPosition(); 
  int rr_encoder = 4; //motorRR->getEncoderPosition();
  Serial2.print( " LF:" );
  Serial2.print( lf_encoder );
  Serial2.print( " RF:" );
  Serial2.print( rf_encoder );
  Serial2.print( " LR:" );
  Serial2.print( lr_encoder );
  Serial2.print( " RR:" );
  Serial2.println( rr_encoder );
  delay(50);

  uint8_t data_buffer[20] = {0};
  uint8_t i, checknum = 0;
  data_buffer[0] = 0xf5;                          // HEADER 
  data_buffer[1] = 19;                            // length 
  data_buffer[2] = 0x20;                          // command 
  data_buffer[3] = (lf_encoder >> 24) & 0xff;             // payload 
  data_buffer[4] = (lf_encoder >> 16) & 0xff;
  data_buffer[5] = (lf_encoder >> 8) & 0xff;
  data_buffer[6] = lf_encoder & 0xff;
  data_buffer[7] = rf_encoder & 0xff;
  data_buffer[8] = (rf_encoder >> 8) & 0xff;
  data_buffer[9] = (rf_encoder >> 16) & 0xff;
  data_buffer[10] = (rf_encoder >> 24) & 0xff;
  data_buffer[11] = lr_encoder & 0xff;
  data_buffer[12] = (lr_encoder >> 8) & 0xff;
  data_buffer[13] = (lr_encoder >> 16) & 0xff;
  data_buffer[14] = (lr_encoder >> 24) & 0xff;
  data_buffer[15] = rr_encoder & 0xff;
  data_buffer[16] = (rr_encoder >> 8) & 0xff;
  data_buffer[17] = (rr_encoder >> 16) & 0xff;
  data_buffer[18] = (rr_encoder >> 24) & 0xff;

  for (i = 2; i < 19; i++)
  {
    checknum += data_buffer[i];
  }
  data_buffer[19] = checknum;                     // checksum
  Serial.write(data_buffer, 20);

}
