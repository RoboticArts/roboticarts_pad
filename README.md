# roboticarts_pad

## Basic installation

Install this ROS package, for example inside ```~/catkin_ws``` workspace

```
cd ~/catkin_ws/src
git clone https://github.com/RoboticArts/roboticarts_pad.git
```

Install ROS dependencies

```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ sudo apt-get install ros-$ROS_DISTRO-joy
```

Build the workspace

```
catkin_make
source devel/setup.bash
```

## Installation complete

If you have a ps4 controller it is highly recommended to install the ds4drv driver. This driver allows Linux to read the IMU of the PS4. Thanks to this, the ```roboticarts_pad``` knows when the PS4 controller is disconnected. In this way, if the connection is lost the robot stops immediately.

Install pip3 in your machine

```
$ sudo apt-get update
$ sudo apt install python3-pip
```

Install ds4drv which allows read from ps4 controller

```
$ sudo pip3 install ds4drv
```

Add udev rule and reload them:

```
$ cd /etc/udev/rules.d && sudo wget https://github.com/RoboticArts/roboticarts_pad/blob/master/utils/ds4drv_utils/ds4drv.rules
$ sudo udevadm control --reload-rules && udevadm trigger
```

Add configuration file to work with ds4drv

```
$ cd /etc && sudo wget https://github.com/RoboticArts/roboticarts_pad/blob/master/utils/ds4drv_utils/ds4drv.config
```

## Check your connection

If you want to check that your pad is connected to your machine install ```jstest```

```
$ sudo apt-get update
$ sudo apt-get install -y jstest-gtk
```

Then, go to ```/dev/input``` and check the devices connected

```
$ cd /dev/input
$ ls
```

Usually, the pad is called ```js0``` or ```js1```. Keep in mind that if you computer has an IMU, you shoud see here as ```js0```. If you installed the ds4drv driver and the udev rule, you should see ```js_robot```. Use ```jstest``` command to test the pad

```
$ jstest js0
```
or

```
$ jstest js1
```

## Usage

Launch ```roboticarts_pad``` package to control your robots:

```
$ roslaunch roboticarts_pad roboticarts_pad.launch
```

Check the output on the ```cmd_vel``` topic

```
$ rostopic echo /cmd_vel
```

## To do

Adding explanation about how to use this pacakge