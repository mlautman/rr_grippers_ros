## Introduction
---
### fxp_pump
A simple ros node to control [fxp_pump](https://www.schmalz.com/en/vacuum-technology-for-automation/vacuum-components/area-gripping-systems-and-end-effectors/vacuum-area-gripping-system-fxp-fmp/area-gripping-systems-fxp) using libsoc

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
