# ROS Driver for Xsens AHRS

## Overview

This is a ROS package for interfacing with the [Xsens MTi-300 AHRS](https://www.xsens.com/mti-300). The user manual can be found [here](https://www.xsens.com/hubfs/Downloads/usermanual/MTi_usermanual.pdf).

The driver communicates with the AHRS through a binary communication protocol called the 'XBus Protocol'. The documentation to the protocol can be found [here](https://mtidocs.xsens.com/mt-low-level-communication-protocol-documentation).

The package can be run as a ROS node or as a standalone python executable file.

The `xsens_driver` package has been tested under ROS Ubuntu 20.04 LTS.

## Installation

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- Following Python3 Package: [pyserial](https://pypi.org/project/pyserial/).

## Building the Package

- In the package run `chmod +x ./nodes/mtdevice.py ./nodes/mtnode.py ./nodes/mtdef.py` to make the python nodes executable.
- Run `catkin_make --pkg xsens_driver` in the catkin workspace to build the package.
- Update the environment variables by `source ./devel/setup.bash` (there are mutiple setup files, source according to the terminal).

## Nodes in the Package

### mtnode.py

The `xsens_driver` node is the main node that interfaces with the AHRS and publishes the readings provided by it.

#### Published Topics

- `~anahita/imu` ([sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))
- `~diagnostics` ([diagnostic_msgs/DiagnosticArray](http://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html))
- `~imu/pitch` ([std_msgs/Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- `~imu/roll` ([std_msgs/Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- `~imu/yaw` ([std_msgs/Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- `~rosout` ([rosgraph_msgs/Log](http://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Log.html))
- `~time_reference` ([sensor_msgs/TimeReference](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/TimeReference.html))

#### Parameters

- `~device` (default: auto): Device file of the IMU
- `~baudrate` (default: 0): Baudrate of the IMU
- `~timeout` (default: 0.002): Timeout for the IMU communication
- `~initial_wait` (default: 0.1): Initial wait to allow device to come up
- `~no_rotation_duration` (default: 0): Duration (int in seconds) of the no-rotation calibration procedure
- `~frame_id` (default: /base_imu): Frame id of the IMU
- `~frame_local` (default: NWU): Desired frame orientation (ENU, NED or NWU)
- `~angular_velocity_covariance_diagonal` (default: [0.0004, 0.0004, 0.0004]): Diagonal elements of angular velocity covariance matrix
- `~linear_acceleration_covariance_diagonal` (default: [0.0004, 0.0004, 0.0004]): Diagonal elements of the orientation covariance matrix
- `~orientation_covariance_diagonal` (default: [0.01745, 0.01745, 0.15708]): Diagonal elements of the orientation covariance matrix

#### Services

- `~nav/set_imu_quat` (xsens_driver/ResetIMUOrient)

## Running the Package

The package has a single launch file `xsens_driver.launch`. Run it using `roslaunch xsens_driver xsens_driver.launch`.

#### Arguments

- `device` (default: auto): Device file of the IMU
- `baudrate` (default: 0): Baudrate of the IMU
- `timeout` (default: 0.002): Timeout for the IMU communication
- `initial_wait` (default: 0.1): Initial wait to allow device to come up
- `no_rotation_duration` (default: 0): Duration (int in seconds) of the no-rotation calibration procedure
- `frame_id` (default: /base_imu): Frame id of the IMU
- `frame_local` (default: NWU): Desired frame orientation (ENU, NED or NWU)
- `angular_velocity_covariance_diagonal` (default: [0.0004, 0.0004, 0.0004]): Diagonal elements of angular velocity covariance matrix
- `linear_acceleration_covariance_diagonal` (default: [0.0004, 0.0004, 0.0004]): Diagonal elements of the orientation covariance matrix
- `orientation_covariance_diagonal` (default: [0.01745, 0.01745, 0.15708]): Diagonal elements of the orientation covariance matrix

**NOTE**: Make sure you have sourced the setup file.

The python node `mtdevice.py` can be run as a stand-alone executable (along with `mtdef.py`)to communicate with the IMU. Run `./mtdevice.py -h` for more info.
