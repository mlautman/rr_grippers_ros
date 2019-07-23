# rr_hw_interface
hw_interface class and classes wrapping external drivers with the interface.

---
### Structure
- `external`: external driver repositories.
    - [`libsoc`](https://github.com/bhuvanchandra/libsoc.git): System on Chips (SoC) through generic Linux Kernel interfaces.
    - [`librevpi`](https://github.com/bhuvanchandra/librevpi.git): control gpio from [revolutional pi](https://revolution.kunbus.com/)
    - [`mcp23017_libsoc`](https://github.com/bhuvanchandra/mcp23017_libsoc.git): MCP23017 I2C-GPIO port expander

### Setup
1. external pacage setup
```
cd rr_hw_interface/external
./install_all.sh
```

### Todo
- Permission handling.Some procesure require root access.
- Update `external/install_all.sh`