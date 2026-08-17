LINOROBOT2 - PID TUNING QUA ROS 2 / PLOTJUGGLER
=================================================

NOI DUNG DA BO SUNG
-------------------
- Subscriber /pid/gains (geometry_msgs/msg/Vector3)
  x = Kp, y = Ki, z = Kd
- Publisher /pid/setpoint (geometry_msgs/msg/Vector3)
  x = RPM motor 1, y = RPM motor 2
- Publisher /pid/measured (geometry_msgs/msg/Vector3)
  x = RPM motor 1, y = RPM motor 2
- Publisher /pid/output (geometry_msgs/msg/Vector3)
  x = PWM motor 1, y = PWM motor 2
- PID reset khi dung va khi doi gains de tranh integral windup.
- Executor timeout giam tu 100 ms xuong 10 ms.

CAI FILE VAO REPO
-----------------
1. Sao luu file hien tai:

   cd ~/linorobot2_hardware
   cp firmware/src/firmware.ino firmware/src/firmware.ino.backup
   cp firmware/lib/pid/pid.h firmware/lib/pid/pid.h.backup
   cp firmware/lib/pid/pid.cpp firmware/lib/pid/pid.cpp.backup

2. Giai nen archive vao thu muc repo:

   cd ~/linorobot2_hardware
   tar -xzf ~/Downloads/linorobot_pid_files_completed.tar.gz

BUILD VA UPLOAD
---------------
   cd ~/linorobot2_hardware/firmware
   pio run -e esp32 -t clean
   pio run -e esp32
   pio run -e esp32 -t upload --upload-port /dev/ttyUSB0

Neu ESP32 nam o cong khac, kiem tra:
   ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

CHAY MICRO-ROS AGENT
--------------------
   source /opt/ros/jazzy/setup.bash
   ros2 run micro_ros_agent micro_ros_agent serial \
     --dev /dev/ttyUSB0 -b 921600 -v4

KIEM TRA TOPIC
--------------
   source /opt/ros/jazzy/setup.bash
   ros2 topic list | grep '^/pid/'

Phai thay:
   /pid/gains
   /pid/measured
   /pid/output
   /pid/setpoint

DOI PID KHONG CAN UPLOAD LAI
----------------------------
Bat dau P-only:
   ros2 topic pub --once /pid/gains geometry_msgs/msg/Vector3 \
     "{x: 0.6, y: 0.0, z: 0.0}"

Vi du them I nho:
   ros2 topic pub --once /pid/gains geometry_msgs/msg/Vector3 \
     "{x: 0.8, y: 0.002, z: 0.0}"

CHAY THU ROBOT
--------------
Ke banh khoi mat dat truoc khi test:
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

Nhan Ctrl+C de dung.

PLOTJUGGLER
-----------
1. Chay:
   source /opt/ros/jazzy/setup.bash
   ros2 run plotjuggler plotjuggler

2. Streaming -> ROS2 Topic Subscriber -> Start.
3. Ve motor 1:
   /pid/setpoint/x
   /pid/measured/x
4. Ve motor 2:
   /pid/setpoint/y
   /pid/measured/y
5. Ve PWM:
   /pid/output/x
   /pid/output/y

LUU Y
-----
- Gains gui qua /pid/gains chi ton tai den khi ESP32 reset/mat nguon.
- Sau khi reset, PID quay ve K_P, K_I, K_D trong config.
- Chi khi tim duoc gains tot moi ghi chung vao config va upload ban cuoi.
