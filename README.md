
# Tarang: Software Stack

<!-- [![Build Status](https://travis-ci.org/AUV-IITK/triton.svg?branch=master)](https://travis-ci.org/AUV-IITK/triton) -->


<!-- The structure of this repository will follow the broadly be like that of Anahita, but with some changes.  -->

# To run this repository:

### Create a catkin worspace following the guidelines given here
```
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src
catkin_init_workspace
cd ..
catkin build
```

### Clone this repository to your catkin workspace
```
cd ~/auv_ws/src
git clone https://github.com/AUV-IITK/Tarang.git
catkin build
```

Install all dependency packages to run the repository.Add python-dependencies in `requirements.txt`

```
rosdep install --from-paths src --ignore-src --rosdistro=noetic
pip install -r requirements.txt 
```

You can build and install those packages from their respective sources or you can use the following command in Ubuntu 20.04 to install them. If you are building from source or using a different package manager, make sure you are building the noetic version of these packages to ensure maximum compatibility.

```
cd ~/catkin_ws
catkin build
```

### Workspace structure
```
.
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── Tarang
    ├── controls
    ├── master
    ├── navigation
    ├── utils
    ├── vision
    ├── README.md
    ├── .rosdep
    └── requirements.txt
```

The branch when not doing any development work would be `master` on `Tarang`.

### Contribution Guidelines

To get started with contributing to this repository, look out for open issues here. Kindly read the Developer's Guide before sending a pull request! :)
