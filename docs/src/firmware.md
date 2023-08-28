# Firmware
The firmware running on the ESP32 microcontroller is contained in the `vango-firmware` directory.
The firmware is responsible for establishing Bluetooth Low-Energy (BLE) communications,
reading encoders, computing wheel angles and speeds,and computing the robot's pose through wheel odometry.

## Communication
The VanGo robot is controlled via a BLE server. Using the `esp32-nimble` crate with `esp-idf-hal`,
several BLE characteristics are established. For those unfamiliar with BLE, a characteristic is simply
a part of the server that a client can read/write data to. Each characteristic is given a
unique UUID to identify it by.

Below are the BLE characteristics provided by the robot and some information about what they are for.

**Left/Right Speed Characteristics** (Read | Write):
- Read: When read from, returns the current wheel speed
- Write: When written to, sets the target wheel speed to the written value

**Pose X, Y, and \\(\theta\\) Characteristics** (Read | Write):
- Read: When read from, returns the current x,y, or \\(\theta\\) position of the robot
- Write: When '0' is written to these characteristics, the value is zeroed

**Pen Characteristic** (WRITE):
- Write: When '1' is written, raises the pen, when '0' is written, lowers the pen

