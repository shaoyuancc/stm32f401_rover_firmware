# STM32F401 Rover Firmware


## Features
- [x] Send and receive commands over USB.
- [x] Motor driver for L298N.
- [x] Encoder driver using STM32 Timer Encoder Mode.
- [x] PID drivers for PID control of motors.
- [x] Differential drive control via [ROS 2 Serial Motor Demo](https://github.com/shaoyuancc/ros2_serial_motor_demo) GUI node.
- [x] Differential drive control via [Twist Teleop](https://github.com/ros2/teleop_twist_keyboard/blob/dashing/teleop_twist_keyboard.py) and [ROS 2 Serial Motor Demo](https://github.com/shaoyuancc/ros2_serial_motor_demo) driver node.
- [ ] Cliff detection using VL6180X ToF sensors
- [ ] Lidar
- [ ] SSD1306 OLED driver
- [ ] Battery level indicator
- [ ] Self docking and recharging
- [ ] Dust sensor


## Credits
Heavily referenced the following sources:
- https://github.com/joshnewans/ros_arduino_bridge
- https://github.com/linorobot/linorobot2_hardware
- https://github.com/1sand0s-git/QuartzArc_STM32F769I_Discovery