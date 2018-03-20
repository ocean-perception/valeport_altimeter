# Valeport Altimeter ROS Package

This ROS package configures and communicates with the Valeport Altimeter
**This has been tested on ROS Indigo and Kinetic over RS232.**

## Setting up

You must clone this repository as `valeport_altimeter` into your catkin workspace:

```bash
git clone https://github.com/olayasturias/valeport_altimeter
```

## Dependencies

Before proceeding, make sure to install all the dependencies by running:

```bash
rosdep update
rosdep install valeport_altimeter
```

## Compiling

You **must** compile this package before being able to run it. You can do so
by running:

```bash
catkin_make
```

from the root of your workspace.

## Running

To run, simply connect the Tritech Profiler sonar over RS232 and launch the
package with:

```bash
rosrun valeport_altimeter altimeter.py 
```


