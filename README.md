# VanGo
A wheeled mobile robot that draws pictures.

- `vango-firmware` is the embedded firmware for the ESP32
- `vango-client` is a desktop command line application for controlling the robot
- `vango-utils` is a utility library used by both the firmware and client.
- The `hardware` directory contains the KiCAD project for the PCB

This project also uses another library I have developed, [diff-drive](https://github.com/nmarks99/diff-drive),
primarily for pose estimation via wheel odometry.
