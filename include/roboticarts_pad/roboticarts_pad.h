#ifndef _ROBOTICARTS_PAD_
#define _ROBOTICARTS_PAD_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <xmlrpcpp/XmlRpcValue.h>
// #include <ctime>
#include "math.h"

class RoboticartsPad{


    public:

        RoboticartsPad(ros::NodeHandle nodehandle);
        void run();

    private:

        ros::NodeHandle _nh;

        ros::Subscriber joy_sub;
        ros::Publisher cmd_vel_pub;

        std::string nodeName;
        std::string cmd_vel_topic;
        
        float LINEAR_INIT, LINEAR_INCREMENT, LINEAR_MAX;
        float ANGULAR_INIT, ANGULAR_INCREMENT, ANGULAR_MAX;

        double TIMEOUT_CONNECTION;
        float MAX_ANALOG_VALUE, MIN_ANALOG_VALUE;

        std::string driver_name;
        uint8_t driver;
        enum driver_type {DS4DRV, GENERIC};

        float last_linear_limit = 0, last_angular_limit = 0;

        enum mode {NORMAL, TRIGGER, PULSE};
        enum key_type {DIGITAL, ANALOG};
        enum twist_component {LINEAR, ANGULAR};

        struct key {
            std::string name;
            int8_t type;         // DIGITAL or ANALOG   
            int8_t id;           // Assigned number on the pad
            bool digital_value;  // For digital values
            float analog_value;  // For analog values
            int triggered;       // For digital trigger function
            int pulse;           // For digital pulse function
            double init_time;    // For digital pulse function 
            
            key(): name(""),type(0), id(0), digital_value(false), analog_value(0.0), 
                   triggered(false), pulse(0), init_time(0.0){}  
        };

        typedef struct key key;

        key  horizontal_axis_stick_left, vertical_axis_stick_left, 
             horizontal_axis_stick_right, L2_trigger, R2_trigger,
             vertical_axis_stick_right, horizontal_cross_pad, vertical_cross_pad;

        key  square_button, x_button, circle_button, triangle_button,
             L1_button, R1_button, share_button, options_button, L3_button,
             R3_button, ps4_button, touchpad_button;

        float _current_linear, _current_angular;

        double last_connection = 0;

        bool zeroTwistFlag = false;

        enum state_resources {CONNECTED, DISCONNECTED};

        int last_print_state = DISCONNECTED;


        void readRosParams();
        key readKeyParam(std::string key_name, uint8_t default_id, uint8_t default_type);
        void waitJoystick(void);

        double getCurrentTime(void);

        void updateJoyKey(key &key, const sensor_msgs::Joy::ConstPtr& msg);
        void updateJoyValues(const sensor_msgs::Joy::ConstPtr& msg);
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void printJoyValues(void);

        bool digitalTriggerFilter(key &key, bool state);
        bool digitalPulseFilter(key &key, bool state);
        bool digitalRead(key &key, uint8_t mode=NORMAL);
        float analogRead(key &key);
        
        void printTwist(float linear_limit, float angular_limit);
        void setLimits(float &value, float min, float max);
        float getTwistLimit(key &increment_button, key &decrement_button, uint8_t twist_component);
        float getTwistRange(key &positive_key, key &negative_key, uint8_t twist_component, float limit_key);
        geometry_msgs::Twist getTwistFromPad();
        void pubTwist(geometry_msgs::Twist twist, uint8_t precission=5);
        
        void pubZeroTwistOnce();
        void resetPubZeroTwistOnce();
        bool deadManButton();
        void holdConnection();
        uint8_t checkConnection();
        int  checkJoystickState();
        void printJoystickState(int state);

        float fmap(float value, float in_min, float in_max, float out_min, float out_max, uint8_t precission=5);
        float fmultiple(float value, float multiple, bool up=false, uint8_t precission=5);
};


        
        /*
        std::vector<key *> digital_keys { &square_button, &x_button, &circle_button, &triangle_button,
                                          &L1_button, &R1_button, &share_button, &options_button, &L3_button,
                                          &R3_button, &ps4_button, &touchpad_button };
        */

#endif