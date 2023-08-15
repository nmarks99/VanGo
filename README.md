# VanGo: A Drawing Robot
VanGo is a differential drive robot that can draw SVG images on a page.
First, an SVG image is converted to XY coordinates using this online tool: [Coordinator](https://spotify.github.io/coordinator/).
Then, using these XY coordinates, a trajectory of waypoints is generated for the robot to follow.
Trajectory tracking is implemented using the *INSERT ALGORITHM HERE* algorithm, and the robot determines its pose in space
through wheel odometry, using quadrature encoders on the motors. For more in-depth information, please see the VanGo documentation
at *INSERT LINK TO DOCS*.

## Project Structure
- `vango-firmware` is the firmware for the ESP32 board based on [esp-idf-hal](https://github.com/esp-rs/esp-idf-hal)
- `vango-client` is a desktop command line application for controlling the robot
- `vango-utils` is a utility library used by both the firmware and client.
- `hardware` directory contains the KiCAD project for the PCB

This project also uses another library I have developed, [diff-drive](https://github.com/nmarks99/diff-drive),
primarily for pose estimation via wheel odometry. In the future I hope to use both the VanGo and diff-drive projects
as starting points for developing a general purpose, high-performance, and open-source, differential drive robot
that anyone can build with firmware based on Rust on the ESP32 (think DIY [TurtleBot](https://www.robotis.us/turtlebot-3/)).
