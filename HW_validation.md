# Optical flow validation

For sensor validation:

Connections: <https://maidbot.atlassian.net/wiki/spaces/HAR/pages/615284817/VL53L0X+Time+Of+Flight#Single-Mode>

- To check if sensor is detected, run:

  `roscore`
  `rosrun vl53l0x_ros detect_tof`

- To read measurements from the sensor, run:

  `roscore`
  `rosrun vl53l0x_ros read_tof`
