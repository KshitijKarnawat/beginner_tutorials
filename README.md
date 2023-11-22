# beginner_tutorials

## Overview

This package contains code for a simple `publisher` and `subscriber` written in C++ for ROS2 (Humble). Along with services. Additionally, it also contains a launch file to launch all the nodes at once and a unit test.

## Building and Running

Clone the repository in your ROS2 workspace.

```sh
cd < path_to_your_workspace >/src

# For cloning using SSH
git clone git@github.com:KshitijKarnawat/beginner_tutorials.git

```

### Building

To build the package follow the following steps

```sh
cd .. # Make sure you are in the workspace folder and not in src

# This will build all the packages in your workspace
colcon build

# To build only this package use
colcon build --package-select beginner_tutorials

# Source your setup file
source install/setup.bash
```

### Running

We first run the `subscriber` so that we don't lose any of the data the `publisher` has already published.

To run the `publisher`, `server` and `subscriber`.

```sh
# Runs the subscriber
ros2 run beginner_tutorials listener
```

In a new terminal

```sh
cd < path_to_your_workspace >

# Source your setup file
. install/setup.bash

# Runs the publisher
ros2 run beginner_tutorial talker
```

Again in a new terminal

```sh
cd < path_to_your_workspace >

# Source your setup file
. install/setup.bash

# Runs the server
ros2 run beginner_tutorial server
```

Alternatively you can run all three at once using the launch file

```sh
cd < path_to_your_workspace >

# Using ROS2 Launch 
ros2 launch beginner_tutorial server.launch

# You can also edit the publishing frequency using the launch file
ros2 launch beginner_tutorial server.launch pub_freq:=< double value >
```

### Testing

To run the unit tests

```sh
cd < path_to_your_workspace >

# Source your setup file
. install/setup.bash

# Runs the unit tests
ros2 run beginner_tutorials test_beginner_tutorials 
```

### Running ROSBAG

To record the data published by the `publisher` and `server` we can use ROSBAG.

```sh
cd < path_to_your_workspace >

# Source your setup file
. install/setup.bash

# To check if the bag file is valid run the listener node (run this first as we don't want to lose any data)
ros2 run beginner_tutorials listener

# Runs the bag file
ros2 bag play src/beginner_tutorials/bagfiles/tutorial_bagfile

```

## Dependencies

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- CMake Version 3.8 or greater
- C++ 17 or newer

## Assumptions

The above instruction assume that you have installed all the Dependecies and are working on a Ubuntu 22.04 LTS system and have created your ROS2 workspace beforehand.

## Results

Results for `cppcheck`, `cpplint` and `rqt-gui` can be viewed in `results`` folder
