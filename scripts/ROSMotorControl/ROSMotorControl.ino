#define USE_USBCON 1
#include <ros.h>
#include <std_msgs/Int16.h>

//To start communication, use:
//rosrun rosserial_python serial_node.py /dev/ttyACM0

const int encPin1 = 0;
const int encPin2 = 1;
const int encPin3 = 3;
const int encPin4 = 4;

const int M1A = 9;
const int M1B = 10;
const int M2A = 11;
const int M2B = 12;
const int M3A = 7;
const int M3B = SCL;
const int M4A = A4;
const int M4B = A5;

const float dT = 0.02; //seconds
const float Kp = 1.0;
const float Ki = 10.0;
const float Kd = 0.01;

volatile unsigned long timeNow1 = 0, lastTime1;
volatile double tickTime1, spd1 = 0;
volatile int count1 = 0;

volatile unsigned long timeNow2 = 0, lastTime2;
volatile double tickTime2, spd2 = 0;
volatile int count2 = 0;

volatile unsigned long timeNow3 = 0, lastTime3;
volatile double tickTime3, spd3 = 0;
volatile int count3 = 0;

volatile unsigned long timeNow4 = 0, lastTime4;
volatile double tickTime4, spd4 = 0;
volatile int count4 = 0;

int scaledSpd1 = 0, scaledSpd2 = 0, scaledSpd3 = 0, scaledSpd4 = 0;
int dmd = 0, dmdMax = 200, scaledDmd = 0, pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;

float error1 = 0.0, lastError1 = 0.0, errorSum1 = 0.0;
float error2 = 0.0, lastError2 = 0.0, errorSum2 = 0.0;
float error3 = 0.0, lastError3 = 0.0, errorSum3 = 0.0;
float error4 = 0.0, lastError4 = 0.0, errorSum4 = 0.0;

//  -------------------------
//  ROS stuff
//  -------------------------

ros::NodeHandle nh;

std_msgs::Int16 demand;
ros::Publisher p("demand_confirm", &demand);

void callback(const std_msgs::Int16& msg)
{
  dmd = msg.data;
  demand.data = dmd;
  p.publish( &demand );
}

ros::Subscriber<std_msgs::Int16> s("demand_out", &callback);

void setup()
{
  //Motor pins
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  pinMode(M3A, OUTPUT);
  pinMode(M3B, OUTPUT);
  pinMode(M4A, OUTPUT);
  pinMode(M4B, OUTPUT);
  //Set ground on one side of motor (for now)
  digitalWrite(M1B, LOW);
  digitalWrite(M2B, LOW);
  digitalWrite(M3B, LOW);
  digitalWrite(M4B, LOW);
  //Encoder pins
  pinMode(encPin1, INPUT);
  pinMode(encPin2, INPUT);
  pinMode(encPin3, INPUT);
  pinMode(encPin4, INPUT);
  //Interrupts
  attachInterrupt(digitalPinToInterrupt(encPin1), ISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(encPin2), ISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(encPin3), ISR3, RISING);
  attachInterrupt(digitalPinToInterrupt(encPin4), ISR4, RISING);

  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
}

void loop()
{
  scaledDmd = map(dmd, 0, 1023, 0, dmdMax);
  scaledSpd1 = map(spd1, 0, 170, 0, 255);
  scaledSpd2 = map(spd2, 0, 170, 0, 255);
  scaledSpd3 = map(spd3, 0, 170, 0, 255);
  scaledSpd4 = map(spd4, 0, 170, 0, 255);

  //Motor 1
  lastError1 = error1;
  error1 = scaledDmd - scaledSpd1;
  errorSum1 += error1*dT;
  pwm1 = Kp*error1 + Ki*errorSum1 + Kd*(error1 - lastError1)/dT;
  if (scaledDmd == 0) pwm1 = 0;
  if (pwm1 < 0) pwm1 = 0;
  if (pwm1 > 255) pwm1 = 255;
  analogWrite(M1A, pwm1);
  if ((micros()-timeNow1) > 50000) spd1=0;

  //Motor 2
  lastError2 = error2;
  error2 = scaledDmd - scaledSpd2;
  errorSum2 += error2*dT;
  pwm2 = Kp*error2 + Ki*errorSum2 + Kd*(error2 - lastError2)/dT;
  if (scaledDmd == 0) pwm2 = 0;
  if (pwm2 < 0) pwm2 = 0;
  if (pwm2 > 255) pwm2 = 255;
  analogWrite(M2A, pwm2);
  if ((micros()-timeNow2) > 50000) spd2=0;

  //Motor 3
  lastError3 = error3;
  error3 = scaledDmd - scaledSpd3;
  errorSum3 += error3*dT;
  pwm3 = Kp*error3 + Ki*errorSum3 + Kd*(error3 - lastError3)/dT;
  if (scaledDmd == 0) pwm3 = 0;
  if (pwm3 < 0) pwm3 = 0;
  if (pwm3 > 255) pwm3 = 255;
  analogWrite(M3A, pwm3);
  if ((micros()-timeNow3) > 50000) spd3=0;

  //Motor 4
  lastError4 = error4;
  error4 = scaledDmd - scaledSpd4;
  errorSum4 += error4*dT;
  pwm4 = Kp*error4 + Ki*errorSum4 + Kd*(error4 - lastError4)/dT;
  if (scaledDmd == 0) pwm4 = 0;
  if (pwm4 < 0) pwm4 = 0;
  if (pwm4 > 255) pwm4 = 255;
  analogWrite(M4A, pwm4);
  if ((micros()-timeNow4) > 50000) spd4=0;

  nh.spinOnce();
  delay(dT*1000); //miliseconds
}

void ISR1()
{
  count1++;
  if (count1 < 5) return;
  lastTime1 = timeNow1;
  timeNow1 = micros();
  tickTime1 = float(timeNow1 - lastTime1);
  spd1 = 1000000/tickTime1; // gives Hz for 5-tick blocks
  count1=0;
}

void ISR2()
{
  count2++;
  if (count2 < 5) return;
  lastTime2 = timeNow2;
  timeNow2 = micros();
  tickTime2 = float(timeNow2 - lastTime2);
  spd2 = 1000000/tickTime2;
  count2=0;
}

void ISR3()
{
  count3++;
  if (count3 < 5) return;
  lastTime3 = timeNow3;
  timeNow3 = micros();
  tickTime3 = float(timeNow3 - lastTime3);
  spd3 = 1000000/tickTime3; // gives Hz for 5-tick blocks
  count3=0;
}

void ISR4()
{
  count4++;
  if (count4 < 5) return;
  lastTime4 = timeNow4;
  timeNow4 = micros();
  tickTime4 = float(timeNow4 - lastTime4);
  spd4 = 1000000/tickTime4;
  count4=0;
}
