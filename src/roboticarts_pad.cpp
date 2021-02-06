
#include <roboticarts_pad/roboticarts_pad.h>


RoboticartsPad::RoboticartsPad(ros::NodeHandle nodehandle):_nh(nodehandle)
{
    nodeName = ros::this_node::getName();
    readRosParams();

    joy_sub = _nh.subscribe("joy", 1, &RoboticartsPad::joyCallback, this);
    cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000);
    
    _current_linear = LINEAR_INIT;
    _current_angular = ANGULAR_INIT;
    
    ROS_INFO("%s node ready!",nodeName.c_str());

    waitJoystick();
}


void RoboticartsPad::readRosParams(){

    ros::param::param<std::string>(nodeName + "/cmd_vel_topic", cmd_vel_topic, "cmd_vel");

    ros::param::param<float>(nodeName + "/pad_twist/linear_init", LINEAR_INIT , 0.3);
    ros::param::param<float>(nodeName + "/pad_twist/linear_increment", LINEAR_INCREMENT, 0.1);
    ros::param::param<float>(nodeName + "/pad_twist/linear_max", LINEAR_MAX, 3.0);

    ros::param::param<float>(nodeName + "/pad_twist/angular_init", ANGULAR_INIT, 0.5);
    ros::param::param<float>(nodeName + "/pad_twist/angular_increment", ANGULAR_INCREMENT, 0.5);
    ros::param::param<float>(nodeName + "/pad_twist/angular_max", ANGULAR_MAX, 3.0);

    square_button   = readKeyParam("square_button",   0, DIGITAL);
    x_button        = readKeyParam("x_button",        1, DIGITAL);
    circle_button   = readKeyParam("circle_button",   2, DIGITAL);
    triangle_button = readKeyParam("triangle_button", 3, DIGITAL);
    L1_button       = readKeyParam("L1_button",       4, DIGITAL);
    R1_button       = readKeyParam("R1_button",       5, DIGITAL);
    share_button    = readKeyParam("share_button",    8, DIGITAL);
    options_button  = readKeyParam("options_button",  9, DIGITAL);
    L3_button       = readKeyParam("L3_button",       10, DIGITAL);
    R3_button       = readKeyParam("R3_button",       11, DIGITAL);
    ps4_button      = readKeyParam("ps4_button",      12, DIGITAL);
    touchpad_button = readKeyParam("touchpad_button", 13, DIGITAL);

    horizontal_axis_stick_left  = readKeyParam("horizontal_axis_stick_left",  0, ANALOG);
    vertical_axis_stick_left    = readKeyParam("vertical_axis_stick_left",    1, ANALOG);
    horizontal_axis_stick_right = readKeyParam("horizontal_axis_stick_right", 2, ANALOG);
    vertical_axis_stick_right   = readKeyParam("vertical_axis_stick_right",   5, ANALOG);
    L2_trigger                  = readKeyParam("L2_trigger",                  3, ANALOG);
    R2_trigger                  = readKeyParam("R2_trigger",                  4, ANALOG);
    horizontal_cross_pad        = readKeyParam("horizontal_cross_pad",        9, ANALOG);
    vertical_cross_pad          = readKeyParam("vertical_cross_pad",          10, ANALOG);

    ros::param::param<std::string>(nodeName + "/pad_settings/driver", driver_name, "ds4drv");

    if (driver_name == "ds4drv")
        driver = DS4DRV;
    if (driver_name == "generic")
        driver = GENERIC;

    ros::param::param<float>(nodeName + "/pad_settings/min_analog_value", MIN_ANALOG_VALUE, -1.0);
    ros::param::param<float>(nodeName + "/pad_settings/max_analog_value", MAX_ANALOG_VALUE, 1.0);
    ros::param::param<double>(nodeName + "/pad_settings/timeout_connection", TIMEOUT_CONNECTION, 2.0);

    if (driver == GENERIC)
        TIMEOUT_CONNECTION = 60*60;
}


RoboticartsPad::key RoboticartsPad::readKeyParam(std::string key_name, uint8_t default_id, uint8_t default_type){

    RoboticartsPad::key keyParam;
    XmlRpc::XmlRpcValue settings;

    keyParam.name = key_name;
    keyParam.id = default_id;
    keyParam.type = default_type;

    if (_nh.hasParam(nodeName + "/pad_map/" + key_name)){

        ros::param::get(nodeName + "/pad_map/" + key_name, settings);
        
        keyParam.id = int(settings[0]);

        if (std::string(settings[1]) == "digital")
            keyParam.type = DIGITAL;  

        if (std::string(settings[1]) == "analog")
            keyParam.type = ANALOG;

    }

    else
        ROS_WARN("Settings for %s not found, using default configuration", keyParam.name.c_str());
            
    return keyParam;
}


void RoboticartsPad::waitJoystick(){

    // Wait time clock
    bool use_sim_time = false;
    std::string use_sim_time_name;

    if (ros::param::search("use_sim_time", use_sim_time_name))
        ros::param::get(use_sim_time_name, use_sim_time);

    if (use_sim_time == true)
        while (getCurrentTime() == 0.0 && ros::ok())
            ROS_INFO_ONCE("use_sim_time is true but /clock is not publishing. Waiting for valid clock time...");

    // Scan joystick
    ROS_INFO("Scanning joystick...");

    if (driver == DS4DRV){
        while (ros::topic::waitForMessage<sensor_msgs::Joy>("joy", ros::Duration(2)) == NULL && ros::ok())
            ROS_WARN_ONCE("Joystick not found. Make sure 'ds4drv --hidraw' is installed and running");
    }

    if (driver == GENERIC){
        do
            ROS_INFO_THROTTLE(10, "Joystick without ds4drv, press any button and wait for the connection"); 
        while(ros::topic::waitForMessage<sensor_msgs::Joy>("joy", ros::Duration(2)) == NULL && ros::ok());
    }

    last_connection = getCurrentTime();
    
}


double RoboticartsPad::getCurrentTime(){
    
    //return double(time(0));
    return ros::Time::now().toSec(); 
}


void RoboticartsPad::updateJoyKey(key &key, const sensor_msgs::Joy::ConstPtr& msg){

    if (key.type == ANALOG){
        key.analog_value = msg -> axes[key.id];

        if (key.name == "L2_trigger" || key.name == "R2_trigger")
            key.analog_value = fmap(key.analog_value, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MAX_ANALOG_VALUE, 0);
    
        // Analog to digital conversion
        if (key.analog_value > 0)
            key.digital_value = true;
        else
            key.digital_value = false;
    }

    if (key.type == DIGITAL){
        key.digital_value = msg -> buttons[key.id];

        // Digital to analog conversion
        if (key.digital_value == true)
            key.analog_value = MAX_ANALOG_VALUE;
        else
            key.analog_value = 0;
    }
}


void RoboticartsPad::updateJoyValues(const sensor_msgs::Joy::ConstPtr& msg){

    // Digital values
    updateJoyKey(square_button, msg);
    updateJoyKey(x_button, msg);
    updateJoyKey(circle_button, msg);
    updateJoyKey(triangle_button, msg);
    updateJoyKey(L1_button, msg);
    updateJoyKey(R1_button, msg);
    updateJoyKey(share_button, msg);
    updateJoyKey(options_button, msg);
    updateJoyKey(L3_button, msg);
    updateJoyKey(R3_button, msg);
    updateJoyKey(ps4_button, msg);
    updateJoyKey(touchpad_button, msg);

    // Analog Values
    updateJoyKey(horizontal_axis_stick_left, msg);
    updateJoyKey(vertical_axis_stick_left, msg);
    updateJoyKey(horizontal_axis_stick_right, msg);
    updateJoyKey(L2_trigger, msg);
    updateJoyKey(R2_trigger, msg);
    updateJoyKey(vertical_axis_stick_right, msg);
    updateJoyKey(horizontal_cross_pad, msg);
    updateJoyKey(vertical_cross_pad, msg);
}


void RoboticartsPad::holdConnection(){

      last_connection = getCurrentTime();
}


void RoboticartsPad::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  updateJoyValues(msg);
  holdConnection(); 
  //printJoyValues();
}


void RoboticartsPad::printJoyValues(){

    ROS_INFO("horizontal_axis_stick_left: %f", analogRead(horizontal_axis_stick_left));
    ROS_INFO("vertical_axis_stick_left: %f", analogRead(vertical_axis_stick_left));
    ROS_INFO("horizontal_axis_stick_right: %f", analogRead(horizontal_axis_stick_right));
    ROS_INFO("vertical_axis_stick_right: %f", analogRead(vertical_axis_stick_right));
    ROS_INFO("L2_trigger: %f", analogRead(L2_trigger));
    ROS_INFO("R2_trigger: %f", analogRead(R2_trigger));
    ROS_INFO("horizontal_cross_pad: %f", analogRead(horizontal_cross_pad));
    ROS_INFO("vertical_cross_pad: %f", analogRead(vertical_cross_pad));

    ROS_INFO("-------");
   
    ROS_INFO("square_button: %d", digitalRead(square_button));
    ROS_INFO("x_button: %d", digitalRead(x_button));
    ROS_INFO("circle_button: %d", digitalRead(circle_button));
    ROS_INFO("triangle_button: %d", digitalRead(triangle_button));
    ROS_INFO("L1_button: %d", digitalRead(L1_button));
    ROS_INFO("R1_button: %d", digitalRead(R1_button));
    ROS_INFO("share_button: %d", digitalRead(share_button));
    ROS_INFO("options_button: %d", digitalRead(options_button));
    ROS_INFO("L3_button: %d", digitalRead(L3_button));
    ROS_INFO("R3_button: %d", digitalRead(R3_button));
    ROS_INFO("ps4_button: %d", digitalRead(ps4_button));
    ROS_INFO("touchpad_button: %d", digitalRead(touchpad_button));

    ROS_INFO("-------------------------------------");
}


float RoboticartsPad::fmap(float value, float in_min, float in_max, 
                            float out_min, float out_max, uint8_t precission){
    
    // Map a decimal
    int64_t exp = pow(10, precission);
    int64_t ivalue = value * exp;
    int64_t iin_min = in_min * exp;
    int64_t iin_max = in_max * exp;
    int64_t iout_min = out_min * exp;
    int64_t iout_max = out_max * exp;
    int64_t iresult;
    float result;

    iresult = (((ivalue+abs(iin_min))*(iout_max-iout_min))/(iin_max-iin_min))+iout_min;

    result = iresult/float(exp);

    return result;
}


float RoboticartsPad::fmultiple(float value, float multiple, bool up, uint8_t precission){
   
    int64_t exp = pow(10, precission);
    int64_t ivalue = abs(value * exp);
    int64_t imultiple = abs(multiple * exp);
    int64_t iremainder;
    int64_t iresult;
    float result;

    iremainder = ivalue % imultiple;
    //ROS_INFO("diff: %d", imultiple-iremainder);

    // Approximates 4.59999 to 4.6
    if ( imultiple-iremainder <= 10) 
        iremainder = -(imultiple-iremainder);

    if (up == true)
        iresult = ivalue + (imultiple - iremainder);
    else
        iresult = ivalue - iremainder;


    if (value >= 0)
        result = iresult/float(exp);
    else
        result = -iresult/float(exp);

    // ROS_INFO("ivalue: %d", ivalue);
    // ROS_INFO("imultiple: %d", imultiple);
    // ROS_INFO("iremainder: %d", iremainder);
    // ROS_INFO("result: %f", result);

    return result;
}   


bool RoboticartsPad::digitalTriggerFilter(key &key, bool state){

    if (key.triggered == false){
        if (key.digital_value == true){
            key.triggered = true;
            state = true;
        }
    }
    else
        state = false;
    
    if (key.digital_value == false)
        key.triggered = false;

    return state;
}


bool RoboticartsPad::digitalPulseFilter(key &key, bool state){

    if (state == true){
        
        switch(key.pulse){

            case 0:
                key.init_time = getCurrentTime();
                key.pulse = 1;
                break;
            case 1:
                if (getCurrentTime() - key.init_time <= 0.4)
                    state = false;
                else{
                    key.pulse = 2;
                    key.init_time = getCurrentTime();
                }
                break;
            case 2:
                if (getCurrentTime() - key.init_time <= 0.1)
                    state = false;
                else
                    key.init_time = getCurrentTime();
                break;       
        }
    }
    else{
        key.pulse = 0;
    }

    return state;
}


bool RoboticartsPad::digitalRead(key &key, uint8_t mode){

    bool state = key.digital_value;

    if (mode == TRIGGER)
        state = digitalTriggerFilter(key, state);
    if (mode == PULSE)
        state = digitalPulseFilter(key, state);

    return state;
}


float RoboticartsPad::analogRead(key &key){

    float value = key.analog_value;

    return value;
}


void RoboticartsPad::printTwist(float linear_limit, float angular_limit){

    if ((linear_limit != last_linear_limit) || (angular_limit != last_angular_limit)) {

        ROS_INFO("Joystick : Linear X: %.3f Linear Y: %.3f Angular Z: %.3f", 
                                linear_limit, linear_limit, angular_limit);
        
        last_linear_limit = linear_limit;
        last_angular_limit = angular_limit;
    }
}


void RoboticartsPad::setLimits(float &value, float min, float max){

    if (value >= max)
        value = max;

    if (value <= min)
        value = min;
}


float RoboticartsPad::getTwistLimit(key &increment_button, key &decrement_button, uint8_t twist_component){

    float twist_limit = 0;

    if (twist_component == LINEAR){

        if (digitalRead(increment_button, PULSE))
            _current_linear = _current_linear + LINEAR_INCREMENT;

        if (digitalRead(decrement_button, PULSE))
            _current_linear = _current_linear - LINEAR_INCREMENT;
    
        setLimits(_current_linear, 0, LINEAR_MAX);
        twist_limit = _current_linear;
    }

    if (twist_component == ANGULAR){

        if (digitalRead(increment_button, PULSE))
            _current_angular = _current_angular + ANGULAR_INCREMENT;

        if (digitalRead(decrement_button, PULSE))
            _current_angular = _current_angular - ANGULAR_INCREMENT;

        setLimits(_current_angular, 0, ANGULAR_MAX);
        twist_limit = _current_angular;
    }

    return twist_limit;
}


float RoboticartsPad::getTwistRange(key &positive_key, key &negative_key, uint8_t twist_component, float limit_key){

    float positive_value = analogRead(positive_key);
    float negative_value = analogRead(negative_key);
    float value;
    float increment;
    
    //if (negative_value >= 0 && negative_key.type == DIGITAL)
    //    negative_value = -negative_value;

    //if (positive_value <= 0)
    //    positive_value = 0;

    //if (negative_value >= 0)
    //    negative_value = 0;
    
    if (positive_key.name == negative_key.name){
        
        if (positive_value > 0)
            value = positive_value;
        else if (negative_value < 0)
            value = negative_value;
        else
            value = 0;
    }

    if (positive_key.name != negative_key.name){

        positive_value = fabs(positive_value);
        negative_value = -fabs(negative_value);
        value = positive_value + negative_value;  
    }

    if (twist_component == LINEAR)
        increment = LINEAR_INCREMENT;
    if (twist_component == ANGULAR)
        increment = ANGULAR_INCREMENT;
 
    value = fmap(value, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, -limit_key, limit_key);
    value = fmultiple(value, increment);

    return value;
}


geometry_msgs::Twist RoboticartsPad::getTwistFromPad(){

    geometry_msgs::Twist twist;
    float linear_limit, angular_limit;

    linear_limit  = getTwistLimit(triangle_button, x_button, LINEAR); // Increment, decrement
    angular_limit = getTwistLimit(circle_button, square_button, ANGULAR); // Increment, decrement
    
    printTwist(linear_limit, angular_limit);
    
    twist.linear.x  = getTwistRange(vertical_axis_stick_left, // Foward key
                                    vertical_axis_stick_left, // Backward key
                                    LINEAR, linear_limit);    // Linear & Limit 

    twist.linear.y  = getTwistRange(horizontal_axis_stick_left, // To left key
                                    horizontal_axis_stick_left, // To right key
                                    LINEAR, linear_limit);      // Linear & Limit 

    twist.angular.z = getTwistRange(horizontal_axis_stick_right, // Clockwise key
                                    horizontal_axis_stick_right, // Counterclockwise key
                                    ANGULAR, angular_limit);     // Angular & Limit  

    return twist;
}


void RoboticartsPad::pubTwist(geometry_msgs::Twist twist, uint8_t precission){

    int64_t exp = pow(10, precission);

    twist.linear.x = std::round(twist.linear.x*exp)/float(exp);
    twist.linear.y = std::round(twist.linear.y*exp)/float(exp);
    twist.angular.z = std::round(twist.angular.z*exp)/float(exp);
    cmd_vel_pub.publish(twist);
}


void RoboticartsPad::resetPubZeroTwistOnce(){   

    zeroTwistFlag = true;
}


void RoboticartsPad::pubZeroTwistOnce(){

    if(zeroTwistFlag){
        
        geometry_msgs::Twist zero_twist;
        cmd_vel_pub.publish(zero_twist);
        zeroTwistFlag = false;
    }
}


bool RoboticartsPad::deadManButton(){

    return digitalRead(R1_button); 
}


uint8_t RoboticartsPad::checkConnection(){

    int connection;
    double current_time = getCurrentTime();

    //ROS_INFO("current: %f", current_time);
    //ROS_INFO("last: %f", last_connection);
    //ROS_INFO("time diff: %f", current_time-last_connection);

    if(current_time - last_connection < TIMEOUT_CONNECTION) 
        connection = CONNECTED;
    else
        connection = DISCONNECTED;
    
    return connection;

}


void RoboticartsPad::printJoystickState(int print_state){

    if(print_state != last_print_state){

        if (print_state == CONNECTED)
            ROS_INFO("Joystick connected [Last sync: %f]", getCurrentTime() - last_connection );
                    
        if (print_state == DISCONNECTED)
            ROS_ERROR("Lost joystick connection [Last sync: %f]", getCurrentTime() - last_connection );
                    
        last_print_state = print_state;
    }
}
 

void RoboticartsPad::run(){

    geometry_msgs::Twist twist;
    ros::Rate loop_rate(50);

    while (ros::ok()){

        uint8_t connection = checkConnection();
        printJoystickState(connection);

        if (connection == CONNECTED){

            if(deadManButton()){
                twist = getTwistFromPad();
                pubTwist(twist);
                resetPubZeroTwistOnce();
            }
            else
                pubZeroTwistOnce();        
        }

        if (connection == DISCONNECTED)
            pubZeroTwistOnce();
           
        ros::spinOnce();
        loop_rate.sleep();

  }

}
