#ifndef _Monitor_h
#define _Monitor_h

#include <cstring>
#include <forward_list>

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <buggy_ctrl/MonitorControl.h>

class MonitorManager;

class Monitor {
    public:
    typedef const char *cstr;

    Monitor() : enabled(false), pub(nullptr) { }

    virtual ~Monitor() {
        /* It is always safe to delete nullptr */
        delete pub;
    }

    void    advertise(ros::NodeHandle &nh)
                            { nh.advertise(*pub); }
    void    enable(bool e)  { enabled = e; }
    cstr    getName()       { return pub->topic_; }

    protected:
    bool            enabled;
    ros::Publisher  *pub;

    MonitorManager &mm();
};

class MonitorManager {
    public:
    std_msgs::Int32         msg_int;
    std_msgs::Float32       msg_float;
    geometry_msgs::Twist    msg_twist;

    static MonitorManager &get()    { return monitor_manager; }

    void    add(Monitor &m)         { monitors.push_front(&m); }

    void    advertise(ros::NodeHandle &nh) {
        for (auto m: monitors) {
            m->advertise(nh);
        }
    }

    private:
    typedef buggy_ctrl::MonitorControl  CtrlMsg;
    typedef std::forward_list<Monitor*> MonitorList;

    static MonitorManager               monitor_manager;

    ros::Subscriber<CtrlMsg>            control;
    MonitorList                         monitors;

    static void on_control(const CtrlMsg &msg) {
        auto mm  = get();
        for (auto m: mm.monitors) {
            if (std::strcmp(m->getName(), msg.channel) != 0)
                continue;
            m->enable(msg.enable);
        }
    }

    MonitorManager() :
        control("/monitor/control", on_control)
    { }
};

inline MonitorManager &Monitor::mm() {
    return MonitorManager::get();
}

class MonitorInt : public Monitor {
    public:
    MonitorInt(cstr topic) {
        auto msg    = &mm().msg_int;
        pub         = new ros::Publisher(topic, msg);
    }

    void report(int value) {
        if (!enabled) return;

        auto &msg   = mm().msg_int;
        msg.data    = value;
        pub->publish(&msg);
    }
};

class MonitorFloat : public Monitor {
    public:
    MonitorFloat(cstr topic) {
        auto msg    = &mm().msg_float;
        pub         = new ros::Publisher(topic, msg);
    }

    void report(float value) {
        if (!enabled) return;

        auto &msg   = mm().msg_float;
        msg.data    = value;
        pub->publish(&msg);
    }
};

class MonitorTwist : public Monitor {
    public:
    MonitorTwist(cstr topic) {
        auto msg    = &mm().msg_twist;
        pub         = new ros::Publisher(topic, msg);
    }

    void report(const geometry_msgs::Twist &value) {
        if (!enabled) return;
         
        pub->publish(&value);
    }
};

#endif
