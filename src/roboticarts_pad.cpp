
#include <roboticarts_pad/roboticarts_pad.h>


RoboticartsPad::RoboticartsPad(ros::NodeHandle nodehandle):_nh(nodehandle)
{


    nodeName = ros::this_node::getName();
    _nh.getParam(nodeName + "/cmd_vel_topic", cmd_vel_topic);

    joy_sub = _nh.subscribe("joy", 1, &RoboticartsPad::joyCallback, this);
    vel_pub = _nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000);

    init_connecting  = ros::Time::now().toSec();


    ROS_INFO("%s node already!",nodeName.c_str());

} 


void RoboticartsPad::updateJoyValues(const sensor_msgs::Joy::ConstPtr& msg){

    /* Axis */
    horizontal_axis_stick_left = msg -> axes[0]; 
    vertical_axis_stick_left = msg -> axes[1]; 
    horizontal_axis_stick_right = msg -> axes[2]; 
    L2_analogic = msg -> axes[3];
    R2_analogic = msg -> axes[4];
    vertical_axis_stick_right = msg -> axes[5]; 
    horizontal_cross_key = msg -> axes[9];
    vertical_cross_key = msg -> axes[10];

    /* Buttons */
    square_button = msg -> buttons[0];
    x_button = msg -> buttons[1];
    circle_button = msg -> buttons[2];
    triangle_button = msg -> buttons[3];
    L1_button  = msg -> buttons[4];
    R1_button  = msg -> buttons[5];
    L2_button = msg -> buttons[6];
    R2_button = msg -> buttons[7];
    share_button = msg -> buttons[8];
    options_button = msg -> buttons[9];
    L3_button = msg -> buttons[10];
    R3_button = msg -> buttons[11];
    ps4_button = msg -> buttons[12];
    touchpad_button = msg -> buttons[13];

}

void RoboticartsPad::printJoyValues(){

  ROS_INFO("horizontal_axis_stick_left: %f", horizontal_axis_stick_left);
  ROS_INFO("vertical_axis_stick_left: %f", vertical_axis_stick_left);

  ROS_INFO("horizontal_axis_stick_right: %f", horizontal_axis_stick_right);
  ROS_INFO("vertical_axis_stick_right: %f", vertical_axis_stick_right);

  ROS_INFO("L2_analogic: %f", L2_analogic);
  ROS_INFO("R2_analogic: %f", R2_analogic);

  ROS_INFO("horizontal_cross_key: %f", horizontal_cross_key);
  ROS_INFO("vertical_cross_key: %f", vertical_cross_key);

  ROS_INFO("-------");

  ROS_INFO("square_button: %d", square_button);
  ROS_INFO("x_button: %d", x_button);
  ROS_INFO("circle_button: %d", circle_button);
  ROS_INFO("triangle_button: %d", triangle_button);
  ROS_INFO("L1_button: %d", L1_button);
  ROS_INFO("R1_button: %d", R1_button);
  ROS_INFO("L2_button: %d", L2_button);
  ROS_INFO("R2_button: %d", R2_button);
  ROS_INFO("share_button: %d", share_button);
  ROS_INFO("options_button: %d", options_button);
  ROS_INFO("L3_button: %d", L3_button);
  ROS_INFO("R3_button: %d", R3_button);
  ROS_INFO("ps4_button: %d", ps4_button);
  ROS_INFO("touchpad_button: %d", touchpad_button);

  ROS_INFO("-------------------------------------");


}


void RoboticartsPad::setLimits(float &value, float min, float max){

    if (value >= max)
        value = max;

    if (value <= min)
        value = min;

}

bool RoboticartsPad::isPressed(bool button){

    bool state;

    if(button)
      state = true;
    else
      state = false;
    
    return state;
}

bool RoboticartsPad::isReleased(bool button){
    
    return !isPressed(button);
}


float RoboticartsPad::setTurn(uint8_t increment_button, uint8_t decrement_button){

  // Execute once when button is pressed
  if(!isExecuted[TURN_UP]) {

      if (isPressed(increment_button)){

          _turn = _turn + TURN_INCREMENT;
          isExecuted[TURN_UP] = true;
      }

  }

  if(!isExecuted[TURN_DOWN]) {

      if (isPressed(decrement_button)){

          _turn = _turn - TURN_INCREMENT;
          isExecuted[TURN_DOWN] = true;
      }

  }

  // Wait for button released to execute again
  if(isReleased(increment_button)){

      isExecuted[TURN_UP] = false;
  }

  if(isReleased(decrement_button)){

      isExecuted[TURN_DOWN] = false;
  }

  setLimits(_turn, 0, MAX_TURN);

  return _turn;
}


float RoboticartsPad::setSpeed(uint8_t increment_button, uint8_t decrement_button){

  // Execute once when button is pressed
  if(!isExecuted[SPEED_UP]) {

      if (isPressed(increment_button)){

          _speed = _speed + SPEED_INCREMENT;
          isExecuted[SPEED_UP] = true;
      }

  }

  if(!isExecuted[SPEED_DOWN]) {

      if (isPressed(decrement_button)){

          _speed = _speed - SPEED_INCREMENT;
          isExecuted[SPEED_DOWN] = true;
      }

  }

  // Wait for button released to execute again
  if(isReleased(increment_button)){

      isExecuted[SPEED_UP] = false;
  }

  if(isReleased(decrement_button)){

      isExecuted[SPEED_DOWN] = false;
  }


  setLimits(_speed, 0, MAX_SPEED);

  return _speed;
}




int8_t RoboticartsPad::setSpeedDirection(uint8_t forward_stick, uint8_t backward_stick){

    int8_t speed_direction_value;

    // Forward
    if(forward_stick > 0.5)
        speed_direction_value = 1;
    
    // Backward
    else if (backward_stick < -0.5)
        speed_direction_value = -1;
    
    //Stop
    else
        speed_direction_value = 0;

    
    return speed_direction_value;
}


int8_t RoboticartsPad::setTurnDirection(uint8_t clockwise_button, uint8_t counterclockwise_button ){

    int8_t turn_direction_value;

    // Turn right
    if(clockwise_button && !counterclockwise_button)
        turn_direction_value = 1;

    // Turn left
    else if(!clockwise_button && counterclockwise_button)
      turn_direction_value = -1;

    // Stop
    else
      turn_direction_value = 0;


    return turn_direction_value;

}



geometry_msgs::Twist RoboticartsPad::setVelocity (){

  geometry_msgs::Twist vel;

  float speed, turn;
  float isConnected;
  int8_t speedDirection, turnDirection;

  // Dead man button 
  if(R1_button){

      speed = setSpeed(triangle_button, x_button); // Increment, decrement
      turn= setTurn(square_button, circle_button); // Increment, decrement

      speedDirection = setSpeedDirection(vertical_axis_stick_left, vertical_axis_stick_left); // Forward, backward
      turnDirection = setTurnDirection(R2_button, L2_button); // Clockwise, Counterclockwise 

      vel.linear.x = speedDirection * speed;
      vel.angular.z = turnDirection * turn;
  }
 
  else {

    // Reset velocity
    geometry_msgs::Twist reset_vel;
    vel = reset_vel;

  }


  return vel;
}

void RoboticartsPad::holdConnection(){

      if(firstConnection)
        last_connection = ros::Time::now().toSec();

      firstConnection = false;
      last_connection = ros::Time::now().toSec();

}

bool RoboticartsPad::checkConnection(){

    bool isConnected;  

    if(!firstConnection){

        if(ros::Time::now().toSec() - last_connection < TIMEOUT_CONNECTION) 
            isConnected = true;
        else
            isConnected = false;
    }
    else
    {
        isConnected = false;
    }
    
    return isConnected;

}

void RoboticartsPad::printJoystickState(int print_state){

    if(print_state != last_print_state){

        switch(print_state){
            
            case CONNECTED:
                    ROS_INFO("Joystick connected [Last sync: %f]", ros::Time::now().toSec() - last_connection );
                    break;

            case DISCONNECTED:
                    ROS_ERROR("Lost joystick connection [Last sync: %f]", ros::Time::now().toSec() - last_connection );
                    break;
        }

        last_print_state = print_state;

    }
}

int RoboticartsPad::checkJoystickState(){

    bool isConnected = checkConnection();

    switch(state){

        case CONNECTING:

            if(ros::Time::now().toSec() - init_connecting < TIMEOUT_CONNECTION){
                if(isConnected)
                    state = CONNECTED;
            }

            else{
                if(!isConnected)
                    state = DISCONNECTED;
            }

            break;

        case CONNECTED:

            printJoystickState(CONNECTED);

            if(!isConnected) 
                state = DISCONNECTED;

            break;

        case DISCONNECTED:

            printJoystickState(DISCONNECTED);

            if(isConnected) 
                state = CONNECTED;

            break;
    }

    return state;
}

 
void RoboticartsPad::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  updateJoyValues(msg);
  holdConnection();
  //printJoyValues();
}


void RoboticartsPad::run(){

    geometry_msgs::Twist vel;
    ros::Rate loop_rate(50);

    while (ros::ok()){

        int status = checkJoystickState();

        switch(status){
        
            case CONNECTED:
                // Read from joystick, write to cmd_vel
                vel = setVelocity();
                vel_pub.publish(vel);
                break;

            case DISCONNECTED:
                // Clear cmd_vel
                geometry_msgs::Twist reset_vel;;
                vel_pub.publish(reset_vel);
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();

  }


}
