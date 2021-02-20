# Overview
This project can be used to control one or more Bitcraze Crazyflies in ROS using python. Writing navigation programs with this software is simple and flexible. Users are free to interact with the Crazyflies through ROS topics and actions, or through a wrapper of the official Bitcraze python API. This allows users familiar with ROS to quickly integrate their crazyflies with part of a larger robotic system, while also providing an appropriate starting point for beginners.

# Dependencies
To use this project, you will need
- Ubuntu (tested on 16.04)  
- Python (tested on 2.7) Note: ROS 1 is designed for Python 2, not Python 3
- a ROS distribution (tested on Kinetic Kame)
- ROS dependencies for building packages
- Bitcraze crazyflie-lib-python


To install ROS and the dependences for building packages, please follow the [official instructions](http://wiki.ros.org/ROS/Installation).


It is also highly reccommended that you install the Bitcraze [Crazyflie Client GUI](https://github.com/bitcraze/crazyflie-clients-pthon). This GUI provides a means to debug Crazyflies, modify their link URI, and more.

## Installing crazyflie-lib-python
To install this dependency, first upgrade your pip wheel for Python 2

```
sudo apt install python-pip
sudo pip install --upgrade pip
```


Now clone crazyflie-lib-python and install it in your Python 2 environment

```
git clone https://github.com/bitcraze/crazyflie-lib-python
cd crazyflie-lib-python
sudo python -m pip install .
cd ..
```

## Add permissions for the Crazyradio
In order to use the Crazyradio without being root some permissions need to be added for the user. Create the plugdev group and add your user to it.


```
sudo groupadd plugdev
sudo usermod -a -G plugdev <username>
```

Create a file named


`/etc/udev/rules.d/99-crazyradio.rules`


and add the following line to it;


`SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"`

# Installation
To install this project, open a terminal to create a catkin workspace


```
mkdir catkin_ws
mkdir catkin_ws/src
cd catkin_ws/src
```


then clone this repository;


`git clone https://github.com/jgsuw/rospy_crazyflie.git`


Now change directory back to `catkin_ws` and build this package.


```
cd ..
catkin_make
```

Lastly, source the devel directory

`source devel/setup.bash`


In order to avoid having to source this file whenever you open a new terminal, you should append the following line to your `~/.bashrc` file;

`source <catkin workspace path>/devel/setup.bash`

Make sure to replace `<catkin workspace path>` with the actual path of your catkin workspace. 

# [View The Quickstart Guide](https://github.com/JGSuw/rospy_crazyflie/wiki/Quick-Start-Guide)
