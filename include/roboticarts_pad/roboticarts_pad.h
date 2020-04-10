#ifndef _ROBOTICARTS_PAD_
#define _ROBOTICARTS_PAD_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

#define SPEED_INCREMENT 0.1
#define TURN_INCREMENT 0.5
#define SPEED_INIT 0.3
#define TURN_INIT 0.5
#define MAX_SPEED 3.0
#define MAX_TURN 3.0
#define TIMEOUT_CONNECTION 2

class RoboticartsPad{


    public:

        RoboticartsPad(ros::NodeHandle nodehandle);
        void run();

    private:

        ros::NodeHandle _nh;
        ros::Subscriber joy_sub;
        ros::Publisher vel_pub;

        std::string nodeName;
        std::string cmd_vel_topic = "cmd_vel"; //Default value


        float horizontal_axis_stick_left, vertical_axis_stick_left, 
              horizontal_axis_stick_right, L2_analogic, R2_analogic,
              vertical_axis_stick_right, horizontal_cross_key, vertical_cross_key;

        int32_t square_button, x_button, circle_button, triangle_button,
                L1_button, R1_button, L2_button, R2_button, share_button, 
                options_button, L3_button, R3_button, ps4_button, touchpad_button;
        
        bool isExecuted[4] = {false,false,false, false};

        float _speed = SPEED_INIT, _turn = TURN_INIT;

        double last_connection = 0;

        double init_connecting = 0;

        bool firstConnection = true;

        enum state_resources {CONNECTING, CONNECTED, DISCONNECTED};
        int state = CONNECTING;

        int last_print_state = CONNECTING;

        enum command {SPEED_UP, SPEED_DOWN, TURN_UP, TURN_DOWN};

        void updateJoyValues(const sensor_msgs::Joy::ConstPtr& msg);
        void printJoyValues(void);
        void setLimits(float &value, float min, float max);
        bool isPressed(bool button);
        bool isReleased(bool button);
        float setTurn(uint8_t increment_button, uint8_t decrement_button);
        float setSpeed(uint8_t increment_button, uint8_t decrement_button);
        int8_t setSpeedDirection(uint8_t forward_stick, uint8_t backward_stick);
        int8_t setTurnDirection(uint8_t clockwise_button, uint8_t counterclockwise_button );
        geometry_msgs::Twist setVelocity ();
        void holdConnection();
        bool checkConnection();
        int  checkJoystickState();
        void printJoystickState(int state);
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);


};


#endif