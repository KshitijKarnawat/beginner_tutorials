# beginner_tutorials

## Overview

This package contains code for a simple `publisher` and `subscriber` written in C++ for ROS2 (Humble).

## Building and Running

Clone the repository in your ROS2 workspace.

```sh
cd < path_to_your_workspace >/src

# For cloning using SSH
git clone git@github.com:KshitijKarnawat/beginner_tutorials.git

# For cloning using HTTPS
git clone https://github.com/KshitijKarnawat/beginner_tutorials.git
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
. install/setup.bash
```

### Running

We first run the `subscriber` so that we don't lose any of the data the `publisher` has already published.

To run the `publisher` and `subscriber`.

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

## Dependencies

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- CMake Version 3.8 or greater
- C++ 17 or newer

## Assumptions

The above instruction assume that you have installed all the Dependecies and are working on a Ubuntu 22.04 LTS system and have created your ROS2 workspace beforehand.
