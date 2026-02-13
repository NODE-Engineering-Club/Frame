# Frame

main ros/simulink frame schema of interconnected components

## Arduino + Camera colour detection (Raspberry Pi + Arduino)

This package contains:

- `camera_detect` — a ROS2 node that performs HSV-based detection of **red**, **green**, and **blue** from a camera feed and publishes `detected_colors` (`std_msgs/String`).
- `arduino_serial` — an existing ROS2 node that forwards the short colour string (e.g. `R`, `GB`, `RGB`, `0`) to a serial-connected Arduino.
- `led_controller.ino` — Arduino sketch that reads the colour string over serial and switches LEDs accordingly.

### Quick start

1. Build the workspace (from workspace root):

   colcon build --packages-select arduino_bridge
   . install/setup.bash

2. Upload `led_controller.ino` to your Arduino (Serial baud 115200).
   - Pins used in the sketch: `RED=11`, `GREEN=10`, `BLUE=9`.
   - Example (arduino-cli):
     - `arduino-cli compile --fqbn arduino:avr:uno path/to/led_controller`
     - `arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno path/to/led_controller`

3. Run the Arduino serial bridge (adjust `serial_port` if needed):

   ros2 run arduino_bridge arduino_serial --ros-args -p serial_port:=/dev/ttyACM0

4. Run the camera detector (use `video_source` to change camera device):

   ros2 run arduino_bridge camera_detect --ros-args -p video_source:=0 -p show_window:=false

Run both nodes with a single launch (recommended):

   ros2 launch arduino_bridge camera_arduino_launch.py

Run inside your Raspberry Pi Docker container (ROS2 Jazzy image):

```sh
docker run -it --rm \
  --device=/dev/ttyACM0 \
  --device=/dev/video0 \
  --group-add dialout \
  -v ~/ros2_ws:/ros2_ws \
  ros2:jazzy-perception
```

Then inside the container:

```sh
cd /ros2_ws
colcon build --packages-select arduino_bridge
. install/setup.bash
ros2 launch arduino_bridge camera_arduino_launch.py
```

Parameters you can tune on the camera node:

- `min_area` (pixel count) — how large a masked region must be to count as detected
- `publish_rate` (Hz)
- `show_window` (bool) — enable a debug window on systems with a display

---

For details, see the sources in `src/arduino_bridge/arduino_bridge`.
