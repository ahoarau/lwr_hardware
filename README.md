Kuka LWR FRI interface
=============================

This package implements two orocos components for communicating with the Kuka LWR through the Fast Research Interface (FRI). One component is used to read diagnostics messages from the robot, when the onther one serves status/command data.

You can compile it for gnulinux or xenomai+rtnet targets if you want 1Khz hard-realtime communication.

### Install with Catkin

```bash
cd ~/catkin_ws
git clone https://https://github.com/ahoarau/lwr_hardware.git
catkin_make
source devel/setup.sh
```

> Note: If you don't want to use catkin, you can also compile it as an independant orocos package.

### Use it in other packages

Add in you package.xml : 

```xml
<build_depend>kuka_lwr_fri</build_depend>
<run_depend>lwr_fri</run_depend>
```

