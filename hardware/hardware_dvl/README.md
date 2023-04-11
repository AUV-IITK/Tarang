# Water Linked DVL A50 - ROS Package

A ROS package for the Water Linked DVL A50. Along with a subscribing client for easy visualization of the communication through ROS.

## Installation
Assuming you created your catkin workspace at the default location. And have git installed. The below steps should work:
```bash
cd ~/catkin_ws/src
git clone -b master https://github.com/waterlinked/dvl-a50-ros-driver.git
cd ~/catkin_ws
catkin_make
```

The serial port logic is based on [this](https://github.com/waterlinked/dvl-python/tree/master/serial)
### Usage
Find the DVLs IP address. Once that's done, the package and it's components can be run by following these steps:

**To run the publisher that listens to the TCP port and sends the data to ROS**
```bash
rosrun waterlinked_a50_ros_driver publisher.py _ip:=192.168.2.95
```

where TCP_IP should be replaced by the IP of the DVL. You can also display the raw DVL data in the terminal by specifying the argument "do_log_data":

**To run the publisher that listens to the TCP port, displays the raw data in the DVL and sends the data to ROS**
```bash
rosrun waterlinked_a50_ros_driver publisher.py _ip:=192.168.2.95 _do_log_raw_data:=true
```

**To run a subscriber node that listens to the DVL topic. Helpful for debugging or checking if everything is running as it should be. Choose between "subscriber_gui.py" and "subscriber.py". The GUI makes reading data visually much easier. While the non-GUI version makes it easier to read through the code to see how you can implement code yourself.**
```bash
rosrun waterlinked_a50_ros_driver subscriber_gui.py
```

## Documentation
The node publishes data to the topics: "*dvl/json_data*" and "*dvl/data*".
* *dvl/json_data*: uses a simple String formated topic that publishes the raw json data coming from the DVL.
* *dvl/data*: Uses a custom message type that structures the parsed data following our protocol. Read more about the protocol here: [DVL Protocol](https://waterlinked.github.io/docs/dvl/dvl-protocol/)

*The graph illustrates the topics and nodes created when the package is run.*
