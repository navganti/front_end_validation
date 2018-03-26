# front_end_validation
Validation tools to publish front-end visual tracking data. To be used in conjunction with [libwave](https://github.com/wavelab/libwave).

# Installation

## Dependencies

This module depends on [libwave](https://github.com/wavelab/libwave). Please follow the instructions to install libwave, found on the project's Github page.

In addition to this, this code in this repository uses [ROS](http://wiki.ros.org/), and has only been tested using ROS Kinetic and Ubuntu 16.04. If ROS is not installed on your machine, please follow the instructions at the ROS wiki.


## Build

Clone this repository to your machine `git clone https://github.com/navganti/front_end_validation.git` or `git clone git@github.com:navganti/front_end_validation.git`.

Navigate to your Catkin workspace source folder (typically `~/catkin_ws/src`), and link the respository.

```
cd /path/to/catkin_ws/src
ln -s /path/to/front_end_validation
```

Build using `catkin_build`.


# Usage

In order to run the validation tool, use the ROS launch file, and specify the `topic` parameter. This topic should be publishing the images that you wish to run through the validator.

`roslaunch offline_validation offline_validator.launch topic:=/image/topic`


# Visualization

Open a new terminal window, and launch [`rqt`](http://wiki.ros.org/rqt). The config folder within the `front_end_validation` contains RQT perspective files to illustrate the validation data.

