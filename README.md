# jetbot2_ws

## ubuntu 18.04 + jetpack + jetbot install + link

### Build jetson-inference

Clone and build the [`jetson-inference`](https://github.com/dusty-nv/jetson-inference) repo:

```bash
# git and cmake should be installed
$ sudo apt-get install git cmake

# clone the repo and submodules
$ cd ~/workspace
$ git clone https://github.com/dusty-nv/jetson-inference
$ cd jetson-inference
$ git submodule update --init

# build from source
$ mkdir build
$ cd build
$ cmake ../
$ make

# install libraries
$ sudo make install
```

### Install non-ROS dependencies

These Python libraries from Adafruit support the TB6612/PCA9685 motor drivers and the SSD1306 debug OLED:

```bash
# pip should be installed
$ sudo apt-get install python-pip

# install Adafruit libraries
$ pip install Adafruit-MotorHAT
$ pip install Adafruit-SSD1306
```

Grant your user access to the i2c bus:

```bash
$ sudo usermod -aG i2c $USER
```

The [`RTIMULib`](https://github.com/RPi-Distro/RTIMULib) libraries support the calibration/demonstration of multiple off-the-shelf IMU variants.

```bash
# clone the repo
$ git clone https://github.com/RPi-Distro/RTIMULib
$ cd RTIMULib/Linux
```

Further information about calibration, execution and demo can be found [here.](https://github.com/RPi-Distro/RTIMULib/tree/master/Linux)

### Install ROS Melodic

```bash
# enable all Ubuntu packages:
$ sudo apt-add-repository universe
$ sudo apt-add-repository multiverse
$ sudo apt-add-repository restricted

# add ROS repository to apt sources
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ROS Base
$ sudo apt-get update
$ sudo apt-get install ros-melodic-ros-base

# install other ROS dependencies
$ sudo apt-get install ros-melodic-vision-msgs ros-melodic-image-transport ros-melodic-image-publisher ros-melodic-joy

# add ROS paths to environment
$ sudo sh -c 'echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc'
```

Close and restart the terminal.

Reboot the system for the changes to take effect.

## Clone and build this ROS workspace

```bash
# Clone the ROS workspace
$ cd ~
$ git clone https://github.com/adityakamath/jetbot2_ws
$ cd jetbot2_ws
$ git submodule update --init
$ git submodule foreach git pull origin main

# Install remaining necessary dependencies
$ rosdep install --from-paths src --ignore-src -r -y

# Build the code (catkin build could also be used instead of catkin_make if the necessary dependencies are installed)
$ cd ~/jetbot2_ws
$ source opt/ros/melodic/setup.bash
$ catkin_make
```

## Running the system from a terminal:
Open a new terminal and run the following in the jetbo2_ws directory:

```bash
$ source devel/setup.bash
$ roslaunch akros_jetson drive_jetbot2.launch
```

Turn the joystick on. If the connection is made, the LEDs on the Jetbot2 will turn yellow (idle mode). Clicking the start button will make the robot switch to the test mode (still yellow). Once, the select button is pressed, the robot switches to the teleop mode (red LEDs), which means the robot can be moved using the joystick. To move it autonomously (edge avoidance, like in this example), press the select button 2 more times till the LEDs turn blue. The autonomous operation can be turned on or off by pressing the start button. 


## Running the system on startup:
The [`robot_upstart`](http://docs.ros.org/en/jade/api/robot_upstart/html/) package creates services from ROS launch files, so that they can be launched when the Jetson Nano is powered on. 

```bash
# Install robot_upstart and generate the startup service 'my_jetbot2'
$ sudo apt-get install ros-melodic-robot-upstart
$ cd jetbot2_ws && source devel/setup.bash
$ rosrun robot_upstart install akros_jetson/launch/drive_jetbot2.launch --job my_jetbot2 --symlink
```

In the last command, 'my_jetbot2' can be replace with any custom name you want. The following commands can be used to start/stop/enable/disable the created service.

```bash
# to enable the service to start when the Jetson nano is next booted. To disable the service use 'disable'
$ sudo systemctl enable my_jetbot2.service

# to start the service. To stop, use 'stop' instead of 'start'
$ sudo systemctl start my_jetbot2.service

# to completely uninstall the service
$ sudo systemctl uninstall my_jetbot2.service
```

More information can be found in this tutorial [here.](https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/). Once the service is generated and enabled, the main launch file (drive_jetbot2.launch) should start running once the Jetson Nano is powered on. While the Jetson Nano takes a while to boot and the ROS code doesn't run immediately, you can tell from the colors of the LEDs. It should turn yellow once you have the joystick controller on. 





