/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  330 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("4 channel Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

String inString = "";    // string to hold input

void loop() {

  //while (Serial.available() > 0) {
  //  int inChar = Serial.read();
  //  if (isDigit(inChar)) {
  //    // convert the incoming byte to a char and add it to the string:
  //    inString += (char)inChar;
  //  }
  //  // if you get a newline, print the string, then the string's value:
  //  if (inChar == '\n') {
  //    Serial.print("Value:");
  //    Serial.println(inString.toInt());
  //    Serial.print("String: ");
  //    Serial.println(inString);
  //    pwm.setPWM(servonum, 0, inString.toInt());
  //    //delay(500);
  //    //pwm.setPWM(4, 0, inString.toInt());
  //    //delay(500);
  //    //pwm.setPWM(11, 0, inString.toInt());
  //    //delay(500);
  //    //pwm.setPWM(1, 0, inString.toInt());
  //    //delay(500);
  //    // clear the string for new input:
  //    inString = "";
  //  }
  //}
  
//  if (Serial.available() > 0) {
//    // read the incoming byte:
//    string incomingByte = Serial.read();
//    int iPWM = toint(incomingByte);
//    pwm.setPWM(servonum, 0, incomingByte);
//    
//    Serial.print("I received: ");
//    Serial.println(iPWM, DEC);
//  }


  // Drive each servo one at a time
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
    delay(1);
  }

  delay(3000);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
    delay(10);
  }

  delay(700);

  servonum ++;
  if (servonum > 0) servonum = 0;
}
