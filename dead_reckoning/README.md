To run the drivers you should run either of the following.
```bash
ros2 launch gps_driver sf_launch.py
```
or
```bash
ros2 launch imu_driver sf_launch.py 
```
Note that the ports for the GPS and the IMU are set in the launch file and not as arguments.
GPS is set to /dev/ttyUSB1
IMU is set to /dev/ttyUSB0

If the ports were different on your device, manually change them in the launch file found in /src/gps_driver/launch/sf_launch.py or /src/imu_driver/launch/sf_launch.py depending on which one you run.
(it's a little inconvenient. Apologies)