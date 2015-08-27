/* This code drives a hexapod with 2 DOF legs and a SRF04 ultrasonic range finder. 

It was written for an Arduino Mega, and utilizes an Adafruit 16 channel servo driver (see text below)
and an SRF04 ultrasonic range finder. 

Written by Josh Herrala. www.networkoracle.com 

BSD license, all text above must be included in any redistribution.

*/

/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

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

#define SERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)


//******Rotation servo matches leg number**********
//Legs are numbered 1-6 starting with front left, 1-3 on left side, 4-6 on right. 4 is right front. 
//Triangle Left
int leglift1 = 0;
int leglift3 = 3;
int leglift5 = 4;
int legrot1 = 1;
int legrot3 = 2; //425bb servo
int legrot5 = 5;

//Triangle Right

int leglift4 = 6;
int leglift6 = 8;
int leglift2 = 14;
int legrot4 = 7;
int legrot6 = 9;
int legrot2 = 13;

/*Initial leg position variables. Variations in servo type and mechanical differences
require each servo to be tuned to an initial neutral position.*/
int frontleftliftpos = 375;
int frontleftrotpos = 470; //-- makes it go forward more
int midleftliftpos = 500; //425bb servo
int midleftrotpos = 350;
int backleftliftpos  = 400;
int backleftrotpos = 500;

int frontrightliftpos = 400;
int frontrightrotpos = 450;
int midrightliftpos = 500;
int midrightrotpos = 350; 
int backrightliftpos = 400;
int backrightrotpos = 450;

//Variables used to control walking speed.
int liftdelay = 1;
int rotdelay = 2;

//Variables used as counters in Do While loops that create walking motion.
int counter = 200;
int rotcounter = 200;
int wavecounter = 200;

//Following variables define the send/receive pins for the SRF04. 
const int triggerPin = 22; //pin to send pulse
const int echoPin = 31;  //pin to receive pulse

void setup() {
Serial.begin(9600);

pwm.begin();
pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}


/*Main loop centers on using the SRF04 to determine if there are obstacles
in front of the robot, and take avoidance actions if required.*/
void loop()
{
// establish variables for duration of the ping,
// and the distance result in centimeters:
long duration, cm;

// The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
pinMode(triggerPin, OUTPUT);
digitalWrite(triggerPin, LOW);
delayMicroseconds(2);
digitalWrite(triggerPin, HIGH);
delayMicroseconds(5);
digitalWrite(triggerPin, LOW);

// The echo pin is used to read the signal from the PING))): a HIGH
// pulse whose duration is the time (in microseconds) from the sending
// of the ping to the reception of its echo off of an object.
pinMode(echoPin, INPUT);
duration = pulseIn(echoPin, HIGH);

// convert the time into a distance
cm = microsecondsToCentimeters(duration);
//Use code below to see distance using serial monitors
Serial.print(cm);
Serial.print("cm");
Serial.println();

if (cm > 10) //If nothing is detected, it calls the walk function. Full steam ahead!
{
 walk();
}
 else
 wave();
 

//wave();

}

void walk() {
  //Activate servos to intial positions
  
  servofrontleftlift(frontleftliftpos);
  servomidrightlift(midrightliftpos);
  servofrontrightlift(frontrightliftpos);
  servobackrightlift(backrightliftpos);
  servomidleftlift(midleftliftpos);
  servobackleftlift(backleftliftpos);
  servobackleftrot(backleftrotpos); 
  servorightbackrot(backrightrotpos);
  
  
  
  //delay(1000);
  
  //Begin Walking sequence
  Serial.print("Lift Left");
  Serial.println();
  //Lift left triangle. This includes the front and rear legs on
  //on the left side, and the middle leg on the right side.
  do {
  servofrontleftlift(frontleftliftpos);
  servobackleftlift(backleftliftpos);
  servomidrightlift(midrightliftpos);
  midrightliftpos++;
  frontleftliftpos--;
  backleftliftpos--;
  counter++;
  delay(liftdelay);
} while (counter <= 300);
 
 Serial.print("Rotate Right Triangle Back, Left Forward.");
 Serial.println();
 //Right triangle legs move back, pushing robot forward. Left legs
 //move forward.
 do {
   servorightfrontrot(frontrightrotpos);
   servorightbackrot(backrightrotpos);
   servoleftmidrot(midleftrotpos);
   frontrightrotpos--;
   midleftrotpos++;  
   backrightrotpos--;
   servoleftfrontrot(frontleftrotpos);
   servorightmidrot(midrightrotpos);
   servobackleftrot(backleftrotpos);
   frontleftrotpos--;
   backleftrotpos--;
   midrightrotpos++; 
   
   
   rotcounter++;
   delay(rotdelay);
 } while (rotcounter <=300);
 
 
 Serial.print("Drop Left");
 Serial.println();
 //Drop left triangle legs
 do {
  servofrontleftlift(frontleftliftpos);
  servomidrightlift(midrightliftpos);
  servobackleftlift(backleftliftpos);
  midrightliftpos--;
  frontleftliftpos++;
  backleftliftpos++;
  counter--;
  delay(liftdelay);
  
} while (counter >= 200); 

Serial.print("Lift Right");
Serial.println();
//Lift right triangle legs
  do {
  servofrontrightlift(frontrightliftpos);
  servomidleftlift(midleftliftpos);
  servobackrightlift(backrightliftpos);
  midleftliftpos--;
  backrightliftpos++;
  frontrightliftpos++;
  counter++;
  delay(liftdelay);
 
} while (counter <= 300);


Serial.print("Rotate Right Forward, Left Backward");
Serial.println();
//Rotate right triangle forward, left triangle legs move backward
//pushing the robot forward.
 do {
   servorightfrontrot(frontrightrotpos);
   servorightbackrot(backrightrotpos);
   servoleftmidrot(midleftrotpos);
   frontrightrotpos++;
   backrightrotpos++;
   midleftrotpos--;  
   servoleftfrontrot(frontleftrotpos);
   servorightmidrot(midrightrotpos);
   servobackleftrot(backleftrotpos);
   frontleftrotpos++;
   midrightrotpos--;  
   backleftrotpos++;
  
   rotcounter--;
   delay(rotdelay);
 } while (rotcounter >= 200);

Serial.print("Drop Right");
Serial.println();
//Drop right triangle
  do {
  servofrontrightlift(frontrightliftpos);
  servomidleftlift(midleftliftpos);
  servobackrightlift(backrightliftpos);
  backrightliftpos--;
  midleftliftpos++;
  frontrightliftpos--;
  counter--;
  delay(liftdelay);
  
} while (counter >= 200);
  


}


void wave() {
  //Activate servos to intial positions
  
  servofrontleftlift(frontleftliftpos);
  servomidrightlift(midrightliftpos);
  servofrontrightlift(frontrightliftpos);
  servobackrightlift(backrightliftpos);
  servomidleftlift(midleftliftpos);
  servobackleftlift(backleftliftpos);
  servobackleftrot(backleftrotpos); 
  servorightbackrot(backrightrotpos);


do {
servofrontleftlift(frontleftliftpos);
servofrontrightlift(frontrightliftpos);  
frontleftliftpos--;
frontrightliftpos++;
wavecounter++;
  delay(liftdelay);
} while (wavecounter <= 300);

do {
servofrontleftlift(frontleftliftpos);
servofrontrightlift(frontrightliftpos);
frontleftliftpos++;
frontrightliftpos--;
wavecounter--;
  delay(liftdelay);
} while (wavecounter >= 200);






}


/*Servo positioning functions. These functions are called with each
pass through the do while loops above. */
//****************************************************************** 

void servofrontleftlift(int pulselen) //Lift position of leg 1, front left lift
{
   pwm.setPWM(leglift1, 0, pulselen);
   
  }

void servobackleftlift(int pulselen) //Lift position of leg 3, back left lift
{
   
   pwm.setPWM(leglift3, 0, pulselen); 
   }


void servomidrightlift(int pulselen) //Lift position of leg 5, middle right lift
  {
    pwm.setPWM(leglift5, 0, pulselen);
  }
  
void servofrontrightlift(int pulselen) //Lift position of leg 4, front right lift
{
   pwm.setPWM(leglift4, 0, pulselen);
   
   }
  
  void servobackrightlift(int pulselen) //Lift position of leg 6, back right lift
{
   
  pwm.setPWM(leglift6, 0, pulselen); 
   }
   
 void servomidleftlift(int pulselen)  //Lift position of leg 2, middle left lift
{
    pwm.setPWM(leglift2, 0, pulselen);
  }
  
  void servorightfrontrot(int pulselen) //Rotation position of leg 4, front right rotation
  {
    pwm.setPWM(legrot4, 0, pulselen);
   
  }
  
  void servorightbackrot(int pulselen) //Rotation position of leg 6, back right rotation
  {
    
   pwm.setPWM(legrot6, 0, pulselen);
  }
  
  void servoleftmidrot(int pulselen) //Rotation position of leg 2, middle left rotation
  {
    pwm.setPWM(legrot2, 0, pulselen); 
  } 
    
    void servoleftfrontrot(int pulselen) //Rotation position of leg 1, front left rotation
    {
    pwm.setPWM(legrot1, 0, pulselen);
     
    }
   
   void servobackleftrot(int pulselen) //Rotation position of leg 3, back left rotation
    {
    pwm.setPWM(legrot3, 0, pulselen);
    }
   
   
   void servorightmidrot(int pulselen) //Rotation position of leg 5, middle right rotation
  {
    pwm.setPWM(legrot5, 0, pulselen);  
  }  
  
  long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29 / 2;
}
    
