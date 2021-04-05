# jetbot2_ws

This repository is the ROS catkin workspace for projects using the Jetbot2. The Jetbot2 is essentially an [`NVidia Jetbot`](https://jetbot.org/master/) with an additional IMU (MPU6050), an Arduino Nano with some NeoPixel LEDs as visual indicators, and support for a wireless SixAxis controller. The software is built using [`ROS Melodic`](http://wiki.ros.org/melodic) onn NVidia's Jetpack 4.3 image, which is based on Ubuntu 18.04. The ROS software uses the [`ros_deep_learning`](https://github.com/dusty-nv/ros_deep_learning) package directly, and the [`jetbot_ros`](https://github.com/dusty-nv/jetbot_ros) package as a reference, so much thanks to [@dusty-nv](https://github.com/dusty-nv) for the helpful software and inspiration.

## Setup the SD card and NVidia's Jetbot software
Use [this tutorial](https://jetbot.org/master/software_setup/sd_card.html) to setup NVidia's Jetbot software. On the other hand, you can flash NVidia's latest Jetpack image (v4.3+) from [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write) and then install and build the Jetbot packages from [here](https://github.com/NVIDIA-AI-IOT/jetbot). These images and software packages have a few features that were very useful in setting up this project:

* JupyterLab: The NVidia image provides a pre-installed and configured JupyterLab setup. This means you can edit, build and run code from your browser instead of using SSH and a terminal or connecting to a screen.
* Jetbot Notebooks: The Jetbot packages also contain a bunch of example notebooks and tools to (rte)train DL models for specific applications. For example, in this project, the collision_avoidance example was retrained to detect/avoid the edges of my desk.
* PiOLED service: The Jetbot installation also configures a service to display the IP address and the CPU/GPU/Mem usage on the PiOLED screen over I2C. The IP address is essential to use JupyterLab from the browser.

### Build jetson-inference 
(If its not already built and installed in the OS image)

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
Open a new terminal and run the following in the jetbot2_ws directory:

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

More information can be found in this tutorial [here](https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/). Once the service is generated and enabled, the main launch file (drive_jetbot2.launch) should start running once the Jetson Nano is powered on. While the Jetson Nano takes a while to boot and the ROS code doesn't run immediately, you can tell from the colors of the LEDs. It should turn yellow once you have the joystick controller on. 





