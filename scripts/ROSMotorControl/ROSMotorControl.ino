#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "MotorPID.h"
#include "DeadReckoning.h"

std_msgs::Float32 debug;

//To start communication, use:
//rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=500000 (remember #define USE_USBCON 1)
//rosrun rosserial_python serial_node.py /dev/ttyAMA1  _baud:=500000 on pi serial pins)

const int num_motors = 4;
const float wheelbase = 0.12; // Dist between wheel centres. CHECK THIS
const float wheelDia = 0.035;

/* Motor array:
    {Front left,
     Front right,
     Back left,
     Back right}

     Entries in the form [Enc, M1, M2]*/

MotorController motors[num_motors] = {
  { 0, 9, 10 },
  { 1, 11, 12 },
  { 3, 7, SCL },
  { 4, A4, A5 },
};

float dmd[num_motors] = {0, 0, 0, 0};
int linDmdMax = 200;
int angDmdMax = 100;
const int PIDFreq = 50; //Hz - can't go too high because of lack of encoder pulses

//-------------------------
//  ROS stuff
//-------------------------

ros::NodeHandle nh;

geometry_msgs::Twist confirm;

//Position variables:
float x = 0.0;
float y = 0.0;
float theta = 0.0;

//Publishers and subscriber:
ros::Publisher p1("demand_confirm", &confirm);
ros::Publisher p2("debug", &debug);

void callback(const geometry_msgs::Twist& msg)
{
  confirm.linear.x = msg.linear.x;
  confirm.angular.z = msg.angular.z;
  p1.publish( &confirm );
}

ros::Subscriber<geometry_msgs::Twist> s("demand_out", &callback);

//  tf variables:
const int tfRateDivisor = 5;
int loopCount = 0;

//-------------------------
//  Loop handling variables
//-------------------------

unsigned long mainLoopTime = 0;
unsigned long PIDLoopTime = 0;
unsigned long timeTemp = 0;
unsigned long wait = 0;

//-------------------------
//  Interrupt functions
//-------------------------

void ISR1() { motors[0].handle_irq(); }
void ISR2() { motors[1].handle_irq(); }
void ISR3() { motors[2].handle_irq(); }
void ISR4() { motors[3].handle_irq(); }

//-------------------------
//  Functions
//-------------------------

void setup()
{
  motors[0].setup_pins(ISR1);
  motors[1].setup_pins(ISR2);
  motors[2].setup_pins(ISR3);
  motors[3].setup_pins(ISR4);

  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.advertise(p1);
  nh.advertise(p2);
  nh.subscribe(s);
  broadcaster.init(nh);
}

void set_levels()
{
  confirm.linear.x = constrain(confirm.linear.x, -linDmdMax, linDmdMax);
  confirm.angular.z = constrain(confirm.angular.z, -angDmdMax, angDmdMax);

  //Warn message here to say demand out of range
  for (int i = 0; i < num_motors; i++)
  {
    dmd[i] = confirm.linear.x + (i % 2 ? 1 : -1) * confirm.angular.z;
  }
  debug.data = dmd[2];
}

void calculate_moves()
{
  float wheelDist[num_motors] = {0, 0, 0, 0};
  float runningTotal = 0.0;
  float angTotal = 0.0;
  float fwdDist = 0.0;
  float angDist = 0.0;
  float dTheta = 0.0;

  for (int i = 0; i < num_motors; i++)
  {
    wheelDist[i] = PI * wheelDia * motors[i].encCount / 300; //NEEDS DIRECTION
    runningTotal += wheelDist[i];
    angTotal += (i % 2 ? -1 : 1) * wheelDist[i]; //Pos anti-clockwise, so add right, sub left
    motors[i].encCount = 0;
  }
  fwdDist = runningTotal / num_motors;
  angDist = angTotal / 2; //Arc length traced by one side about the centre of car

  //Set new position variables
  //Assume straight line at angle of theta + dTheta/2, then set new theta
  dTheta = angDist / wheelbase;
  x += fwdDist*cos(theta + dTheta/2);
  y += fwdDist*sin(theta + dTheta/2);
  theta += dTheta;
}

void loop()
{
  mainLoopTime = micros();
  loopCount += 1;

  set_levels();

  for (int i = 0; i < num_motors; i++)
  {
    motors[i].process_pid(dmd[i]);
  }

  if (loopCount = tfRateDivisor)
  {
    calculate_moves();
    
    loopCount = 0;
    publish_tf(nh, x, y, theta);
    p2.publish( &debug );
  }

  nh.spinOnce();

  wait = micros() - mainLoopTime;
  wait = 1000000 / PIDFreq - wait;
  delayMicroseconds(wait);
}
