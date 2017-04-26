[![Build Status](https://travis-ci.org/banuprathap/turtlebot_explorer.svg?branch=master)](https://travis-ci.org/banuprathap/turtlebot_explorer)


# ROS Turtlebot Explorer

Most mobile robot applications require the ability to navigate. While many robots can navigate using maps, and some can map what they can see, few can explore autonomously beyond their immediate surroundings. Our goal is to develop exploration strategies for the complex indoor environments typically found in real office buildings. Our approach is based on the detection of frontiers, regions on the border between open space and unexplored space.

This project is aimed at rescue operation in indoor environments during disasters. This project has several applications in this domain ranging from inspection of nuclear plants or initial inspection of a damaged building during an earthquake where there is little to zero information about the environment. It will be implemented on a Turtlebot simulated in Gazebo.


Author(s): Banuprathap Anandan

Maintainer: Banuprathap Anandan, bprathap@umd.edu

Affiliation: University of Maryland

<img src="http://s9.postimg.org/aah3joxv3/image.jpg" width="450">

## Dependencies

- Ubuntu Linux 14.04 as the development platform
- git version control system with GitHub as the host
- ROS Indigo as the middleware
- Gazebo multi-robot simulator version 2.2.3
- C++ language using g++ compiler with C++03 and Catkin build system
- nav_core ROS package (BSD)
- costmap_2D ROS package (BSD)
- gmapping ROS package (CC)
- Travis Continual Integration
- Coveralls coverage monitoring

If you do not have ROS, download the bash script [here](https://gist.github.com/banuprathap/b2dab970df1f89573203b546c5eb3a5c) and run it as **sudo**. Note: This script assumes you're running Ubuntu 14.04.


## Development

The project follows SIP(Solo Iterative Process) to track software changes(SC). Further details can be found [here]().

Please click [here]() for details about SPRINT.


## Build steps

- Open a terminal
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src && catkin_init_workspace
git clone https://github.com/banuprathap/turtlebot_explorer.git
cd ..
catkin_make
source ./devel/setup.bash
```

## License

See the [LICENSE](https://github.com/banuprathap/turtlebot_explorer/blob/master/LICENSE) file for license rights and limitations (MIT).

## Status

Stub

## Running the demo

Stub

## Running Tests

Stub

## Documentation

Stub

### ROS Node Details

Stub

## Issues and bugs

Stub