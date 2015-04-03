Kuka LWR FRI interface
=============================

This package implements an orocos component to read status and send commands to the Kuka LWR via the Fast Research Interface (FRI).

### Install with Catkin

```bash
cd ~/catkin_ws
git clone https://https://github.com/ahoarau/lwr_hardware.git
catkin_make
source devel/setup.sh
```

> Note: you can also compile it as an independant orocos package

### Use it in other packages

Add in you CMakeLists.txt : 

```xml
<build_depend>kuka_lwr_fri</build_depend>
<run_depend>lwr_fri</run_depend>
```

