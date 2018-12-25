# rr_grippers_ros
ROS interface for Rapyuta supported grippers with Kamajii controller

## Introduction
---
### vg10_pump
A simple ros node to control [vg10_pump](https://onrobot.com/products/vg10-vacuum-gripper/) using libsoc

### Prerequisites
* `libsoc`

Instructions for building and installing libsoc can be found at
[http://github.com/bhuvanchandra/libsoc](http://github.com/bhuvanchandra/libsoc)

### Installing libsoc

To support Echo236FE IOs via libsoc, one need to install the echo236fe.conf config file:

```
$ ./configure --enable-board=echo236fe
$ make
$ sudo make install
```
