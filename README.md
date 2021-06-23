# ROS Depth-Based Robot Tracking Package (handy)

This is the package for running Probabilistic Articulated Real-time Tracking on the IVALab Handy robotic manipulator. It assumes you have properly installed (and cited!) https://github.com/bayesian-object-tracking/dbrt. You should not only install dbrt but dbot. Please follow the installation step by step.

It also requires the finalarm_control and finalarm_moveit_config packages. You can find these packages under handy repository.

## Running tracker pipeline
1. start kinect camera
```
roslaunch freenect_launch freenect.launch
```
2. run controller manager
```
roslaunch finalarm_control controller_manager.launch
```
If you get the error of serial.serialutil.SerialException: [Errno 2] could not open port /dev/ttyUSB0: [Errno 2] No such file or directory: '/dev/ttyUSB0', you should run
```
sudo chmod 666 /dev/ttyUSB0
```
3. start multiple controllers for each motor
```
roslaunch finalarm_control start_controller.launch
```
If warning pops out, you'd better redo the step 2 and 3. Since there is only one starting launch file for both position controller and trajectory controller which depends on position controller, there may be some sync problems.
4. publish robot state
```
roslaunch finalarm_dbrt dbrt_robot_state_pub.launch
```
5. run tracker
```
roslaunch finalarm_dbrt dbrt_launch_gpu.launch
```
6. the rest steps are the same as common steps for running grapsing experiments for handy.

## Depth projection
### Setup transformation between robot and camera
In order to align estimated and sensed world, we need to setup the transformation between robot and camera. You need to go to finalarm_dbrt/launch/dbrt_launch_gpu.launch and change the transformation information of static_transform_publisher according to your camera setup.
### Running pipeline
1. You need to follow the above steps until step 5. 
2. Add depth camera in rviz, detailed step: press add -> select By topic -> select camera_depth_image_rect -> clikc Camera
3. Add RGB camera in rviz, detailed step: press add -> select By topic -> select camera_rgb_image -> clikc Camera
### Note
1. Change the value of alpha will affect the transparency of robot model. 
