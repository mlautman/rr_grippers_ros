# rr_grippers_ros
ROS interface for Rapyuta supported grippers and sensors

---
### Structure
- `rr_hw_interface`: This package have interface class and class for handle libsoc gpio, [revolutional pi](https://revolution.kunbus.com/) and etc. ROS packages in the `grippers` and `sensors` directory are use this interface and access the hardware.
  - `external`: external driver repositories.
- `grippers`: have packages for controlling grippers
  - `rr_suction_pump`: tested schumaltz fxp series and vg10 gripper
- `sensors`: have packages for controlling sensors
  - `rr_analog_sensors`: general ROS driver for analog input
  - `vl53lox_ros_driver`: [vl53lox](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html)  
### Prerequisites
gitsubmodules under rr_hw_interface/external
* [`libsoc`](https://github.com/bhuvanchandra/libsoc.git)
* [`librevpi`](https://github.com/bhuvanchandra/librevpi.git)
* [`mcp23017_libsoc`](https://github.com/bhuvanchandra/mcp23017_libsoc.git)

### Setup
1. git clone and submodule setting
```
cd <path to catkin_ws>/src
git clone git@github.com:rapyuta-robotics/rr_grippers_ros.git
cd rr_grippeprs_ros
git submodule init
git submodule update
```
2. external pacage setup
```
cd rr_hw_interface/external
./install_all.sh
```
3. ros workspace build
```
cd <path to catkin_ws>/src
catkin build
```

### Todo
- Permission handling.Some procesure require root access.
- Update `rr_hw_interface/external/install_all.sh`