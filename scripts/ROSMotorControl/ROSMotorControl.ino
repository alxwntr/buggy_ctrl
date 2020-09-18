#define USE_USBCON 1
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

//To start communication, use:
//rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
//PID variables:
const float dT = 0.02; //seconds
const float Kp = 1.0;
const float Ki = 10.0;
const float Kd = 0.01;

/* We could move this whole class into MotorController.h and #include it
 * instead, but we would have to put the parameters above in that header
 * too. */
class MotorController {

    const int encPin_;
    const int motorA_;
    const int motorB_;

    volatile unsigned long lastTime_ = 0;
    volatile double spd_ = 0;
    volatile int count_ = 0;

    float lastError_ = 0.0, errorSum_ = 0.0;

public:
    
    /* This is a constructor: a function called to build a new
     * MotorController object. Constructor syntax is weird in C++: you
     * must write a function with no return type, with the same name as
     * the class. You can write more than one with different parameters
     * and the compiler will choose the right one to call. */
    MotorController (int encPin, int motorA, int motorB)
        /* This constructs the member variables using the arguments
         * passed to the constructor. The syntax is weird, again. You
         * could just have encPin_ = encPin and so on in the function
         * body, except that encPin_ is 'const' so you can't assign to
         * it. So you have to do it like this. */
        : encPin_(encPin), motorA_(motorA), motorB_(motorB)
    /* This is the body of the constructor function. It has nothing in
     * it, we have done all the work above in the initializer section.
     * This is common in C++ constructors. */
    { }

    /* This function should be called from setup(), to setup the pins
     * for this motor.
     *
     * This function has one parameter, isr, which has a function
     * pointer type. It takes a pointer to the ISR to install on the
     * encoder pin. This is the same type as the second parameter to
     * attachInterrupt, as you would expect. See also below, in setup().
     *
     * You will notice that, unlike Python, there is no 'self' parameter
     * to represent the object we are called on. In C++ the object is
     * magically made available as a variable called 'this'. (Python's
     * syntax is much cleaner.)
     */
    void setup_pins (void(*isr)(void))
    {
        pinMode(motorA_, OUTPUT);
        //Set ground on one side of motor (for now)
        digitalWrite(motorB_, LOW);
        //Encoder pins
        pinMode(encPin_, INPUT);

        attachInterrupt(digitalPinToInterrupt(encPin_), isr, RISING);
    }

    //PID loop:
    void process_pid (int scaledDmd)
    {
        //Temporary variables:
        int     pwm;
        int     scaledSpd = map(spd_, 0, 170, 0, 255);
        float   error;

        //Calculate error and set PWM level:
        error       = scaledDmd - scaledSpd;
        errorSum_   += error*dT;
        pwm         = Kp*error + Ki*errorSum_ + Kd*(error - lastError_)/dT;
        lastError_  = error;

        //Constrain PWM and ensure it gets to zero (immediately):
        if (scaledDmd == 0) pwm = 0;
        if (pwm < 0)        pwm = 0;
        if (pwm > 255)      pwm = 255;
        analogWrite(motorA_, pwm);

        //Allow spd_to reach zero if no motion
        if ((micros()-lastTime_) > 50000)
            spd_=0;
    }

    //Called from ISR#
    void handle_irq ()
    {
        unsigned long   timeNow;
        float           tickTime;

        //Do nothing for first 4 times:
        count_++;
        if (count_ < 5) return;
        count_ = 0;

        //Set time variables and calculate speed:
        timeNow     = micros();
        tickTime    = float(timeNow - lastTime_);
        spd_        = 1000000/tickTime; // gives Hz for 5-tick blocks
        lastTime_   = timeNow;
    }
/* Don't forget the semicolon here! */
};

const int num_motors = 4;

/* This is an array of MotorController objects. The parameters are
 * passed to the contstructor above. The curly-brace syntax is really
 * weird, but those numbers are actually function arguments. */
MotorController motors[num_motors] = {
    { 0, 9, 10 },
    { 1, 11, 12 },
    { 3, 7, SCL },
    { 4, A4, A5 },
};

int16_t dmd[num_motors] = {0,0,0,0};
int dmdMax = 200;

//-------------------------
//  ROS stuff
//-------------------------

ros::NodeHandle nh;

//Position variables:
double x = 0.0;
double y = 0.0;
double theta = 1.571;

char base_link[] = "/base_link";
char odom[] = "/odom";

geometry_msgs::Twist confirm;

ros::Publisher p("demand_confirm", &confirm);

void callback(const geometry_msgs::Twist& msg)
{
    confirm.linear.x = msg.linear.x;
    confirm.angular.z = msg.angular.z;
    p.publish( &confirm );
}

ros::Subscriber<geometry_msgs::Twist> s("demand_out", &callback);

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

const float rate_div = 10; //Divider to allow publishing at slower rate
int rate_cnt = 1;
long tfLoopTime = 0;

//-------------------------

void ISR1() { motors[0].handle_irq(); }
void ISR2() { motors[1].handle_irq(); }
void ISR3() { motors[2].handle_irq(); }
void ISR4() { motors[3].handle_irq(); }

void setup()
{
    motors[0].setup_pins(ISR1);
    motors[1].setup_pins(ISR2);
    motors[2].setup_pins(ISR3);
    motors[3].setup_pins(ISR4);

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(p);
    nh.subscribe(s);
    broadcaster.init(nh);
}

void publish_tf()
{
    // tf odom->base_link - why is this not in setup? It gets assigned every time.
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    
    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();
    
    broadcaster.sendTransform(t);

    tfLoopTime = micros();
}

void loop()
{
    long pidLoopTime = micros();
    int scaledDmd = 0;

//    for (int i = 0; i < num_motors; i++) {
//        map(dmd[i], 0, 1023, 0, dmdMax);
//        motors[i].process_pid(scaledDmd);
//    }
    
    // drive in a circle
    double dx = 0.2;
    double dtheta = 0.18;
    x += cos(theta)*dx*0.1;
    y += sin(theta)*dx*0.1;
    theta += dtheta*0.1;
    if(theta > 3.14)
      theta=-3.14;

    publish_tf();

    nh.spinOnce();

    delay(100);
}
