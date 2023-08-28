# VanGo Firmware

## Development Setup

This project was generated with [esp-idf-template](https://github.com/esp-rs/esp-idf-template).
The README in the esp-idf-template repository details the prerequisites that one must
install to setup a development environment.

## Build & Flash
To build and flash the code run the following
```
source export-esp.sh # source ESP toolchain
cargo flash # build and flash with custom partition table (see alias in .cargo/config.toml/)
