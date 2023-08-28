# Embedded Development
To set up a development environment to modify and flash firmware to the device,
you will need to install several things such as the Espressif compiler toolchain,
espflash cargo utility, and of course rust and cargo if you don't have that already.

The best way to get all this setup and make sure it works is to follow the directions
on setting up an `esp-idf-template` project: [esp-idf-template](https://github.com/esp-rs/esp-idf-template).
If you are able to successfully build the template project, you should in theory be able to build and flash
the `vango-firmware` without issue. In the `vango-firmware/.cargo/config.toml` file, there is an alias
for "flash", which allows you to flash the board with `espflash` by simply typing `cargo flash`.
