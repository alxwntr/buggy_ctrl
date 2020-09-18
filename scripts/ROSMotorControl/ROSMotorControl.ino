#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

std_msgs::Float32 debug;

//To start communication, use:
//rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=500000 (remember #define USE_USBCON 1)
//rosrun rosserial_python serial_node.py /dev/ttyAMA1  _baud:=500000 on pi serial pins)

//-------------------------
//  PID variables
//-------------------------

const float dT = 0.02; //seconds
const float Kp = 0.5;
const float Ki = 10.0;
const float Kd = 0.01;

//-------------------------
//  Car hardware variables
//-------------------------

class MotorController {

    const int encPin_;
    const int motorA_;
    const int motorB_;

    volatile unsigned long lastTime_ = 0;
    volatile double spd_ = 0;
    volatile int count_ = 0;

    float lastError_ = 0.0, errorSum_ = 0.0;

  public:

    int dir = 0; // -1 = bkwrds, 0 = static, 1 = frwrds (need to think how to sort this out)
    bool braking = false;

    volatile float encCount = 0.0;
    float floorSpeed = 0.0; //not needed?

    MotorController (int encPin, int motorA, int motorB)
      : encPin_(encPin), motorA_(motorA), motorB_(motorB)
    { }

    void setup_pins (void(*isr)(void))
    {
      pinMode(motorA_, OUTPUT);
      //Set ground on one side of motor (for now)
      digitalWrite(motorB_, LOW);
      //Encoder pins
      pinMode(encPin_, INPUT);

      attachInterrupt(digitalPinToInterrupt(encPin_), isr, CHANGE);
    }

    //PID loop:
    void process_pid (int scaledDmd)
    {
      //Brake (if travelling in wrong direction)
      if (braking)
      {
        digitalWrite(motorA_, HIGH);
        digitalWrite(motorB_, HIGH);
        return;
      }

      //Temporary variables:
      int     pwm;
      int     scaledSpd = map(spd_, 0, 170, 0, 255);
      float   error;

      //Calculate error and set PWM level:
      error       = scaledDmd - scaledSpd;
      errorSum_   += error * dT;
      pwm         = Kp * error + Ki * errorSum_ + Kd * (error - lastError_) / dT;
      lastError_  = error;

      //Constrain PWM and ensure it gets to zero (immediately):
      if (scaledDmd == 0) pwm = 0;
      if (pwm < 0)        pwm = 0;
      if (pwm > 255)      pwm = 255;
      analogWrite(motorA_, pwm);

      //Allow spd_to reach zero if no motion
      if ((micros() - lastTime_) > 50000)
        spd_ = 0;
    }

    //Called from ISR#
    void handle_irq ()
    {
      unsigned long   timeNow;
      float           tickTime;

      //Do nothing for first 4 times:
      count_++;
      encCount++;
      if (count_ < 3) return;
      count_ = 0;

      //Set time variables and calculate speed:
      timeNow     = micros();
      tickTime    = float(timeNow - lastTime_);
      spd_        = 1000000 / tickTime; // gives Hz for 0.5 encoder revs. RECONSIDER
      lastTime_   = timeNow;
    }
};

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
double x = 0.0;
double y = 0.0;
double theta = 1.571;

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
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
const int tfRateDivisor = 5;
int loopCount = 0;

//-------------------------
//  Loop handling variables
//-------------------------

char base_link[] = "/base_link";
char odom[] = "/odom";

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

  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
}

void publish_tf()
{
  t.transform.translation.x = x;
  t.transform.translation.y = y;

  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();

  broadcaster.sendTransform(t);
}

void set_levels()
{
  if (confirm.linear.x > linDmdMax) confirm.linear.x = linDmdMax;
  if (confirm.linear.x < -linDmdMax) confirm.linear.x = -linDmdMax;

  //Warn message here to say demand out of range
  for (int i = 0; i < num_motors; i++)
  {
    dmd[i] = confirm.linear.x + (i % 2 ? 1 : -1) * confirm.angular.z;
  }
}



void calculate_moves()
{
  float wheelDist[num_motors] = {0, 0, 0, 0};
  float runningTotal = 0.0;
  float angTotal = 0.0;
  float fwdDist = 0.0;
  float angDist = 0.0;

  for (int i = 0; i < num_motors; i++)
  {
    wheelDist[i] = PI * wheelDia * motors[i].encCount / 300; //NEEDS DIRECTION
    runningTotal += wheelDist[i];
    angTotal += (i % 2 ? -1 : 1) * wheelDist[i]; //Pos anti-clockwise, so add right, sub left
    motors[i].encCount = 0;
  }
  fwdDist = runningTotal / num_motors;
  angDist = angTotal / 2; //Arc length traced by one side about the centre of car

  x += fwdDist*cos(theta);
  y += fwdDist*sin(theta);
  theta += angDist / wheelbase;
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
    publish_tf();
    p2.publish( &debug );
  }

  nh.spinOnce();

  wait = micros() - mainLoopTime;
  wait = 1000000 / PIDFreq - wait;
  delayMicroseconds(wait);
}
