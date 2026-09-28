# Simultaneous Localization and Mapping (SLAM)

## Student
Vaibhav Prakashrao Bhosale

## Roll No.
24082

## Course
ECS(323)

## Project
Implementation of SLAM on a 6-wheel drive skid-steer robot chassis using LiDAR and IMU.
and for dataset recording purpose i have made some nodes for camera bag recording also ,and done the yolo object detection using realsense camera,all those ros2 nodes are uploaded .
### files uploaded 
i have uploaded ,the files i made for  movement , realsense camera + object detection ,imu visualizer,which are the base for future work of slam implememtation.
### movement signal flow.
Implementation of SLAM on a 6-wheel drive skid-steer robot chassis using LiDAR and IMU.
currently i have completed the pipeline where the movement commands(joystick) are sent to jetson orin nano over bluetooth and then jetsons converts them into w/s/a/d type simple commands 
and sends to arduino uno over serial communication ,and then uno sends pwm signals to motor drivers. 
# signal flow 
joystick---->> jetson---->>arduino uno---->>motor_drivers---->>motors.

this code is in motor_node.py   file

## future work
use of  extended kalman filter based algo for sensor fusing(imu+lidar).

### result 
i have attached some screenshots and videos of working imu_visualizer script, yolo object detection using realsese camera mounter on robot and movement of robot .
