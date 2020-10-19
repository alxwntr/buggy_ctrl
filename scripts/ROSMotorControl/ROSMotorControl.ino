#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include "DeadReckoning.h"
#include "MotorArray.h"

std_msgs::Float32 debug;

//To start communication, use:
//rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=1000000 (remember #define USE_USBCON 1)
//rosrun rosserial_python serial_node.py /dev/ttyAMA1  _baud:=1000000 on pi serial pins)

//-------------------------
//  ROS stuff
//-------------------------

ros::NodeHandle nh;

geometry_msgs::Twist confirm;

//Publishers and subscriber:
ros::Publisher p1("demand_confirm", &confirm);
ros::Publisher p2("debug", &debug);

void callback(const geometry_msgs::Twist& msg)
{
  confirm.linear.x = constrain (msg.linear.x, -linDmdMax, linDmdMax);
  confirm.angular.z = constrain (msg.angular.z, -angDmdMax, angDmdMax);
  p1.publish( &confirm );
}

ros::Subscriber<geometry_msgs::Twist> s("demand_out", &callback);

//  tf variables:
geometry_msgs::TransformStamped t;
char base_link[] = "/base_link";
char odom[] = "/odom";
tf::TransformBroadcaster broadcaster;
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
//  Functions
//-------------------------

void setup()
{
  setup_motors();

  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.advertise(p1);
  nh.advertise(p2);
  nh.subscribe(s);
  broadcaster.init(nh);
}

void publish_tf()
{
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = odometer.x;
  t.transform.translation.y = odometer.y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(odometer.theta);
  t.header.stamp = nh.now();

  broadcaster.sendTransform(t);
}

void loop()
{
  mainLoopTime = micros();
  loopCount += 1;

  for (auto &m: motors) {
    m.process_pid(confirm);
  }

  if (loopCount == tfRateDivisor)
  {
    odometer.calculate_moves();
    
    loopCount = 0;
    publish_tf();
    
    p2.publish( &debug );
  }

  nh.spinOnce();

  wait = micros() - mainLoopTime;
  wait = 1000000 / (1/dT) - wait;
  delayMicroseconds(wait);
}
