<img width="1548" alt="image" src="https://github.com/briancheng195/Development-of-Tele-Operated-Interbotix-WX-Series-Arm/assets/122734373/80a5e79a-5d68-4aa5-a3e0-ba395d63ff73"># Development-of-Tele-Operated-Interbotix-WX-Series-Arm
- Developed Python algorithms with ROS2 for tele-operation and manipulation of Interbotix X-Series Robotic Arm
- This project is done up with ROS2 and the ROS2 library packages with the Interbotix WX Series Robotic Arm

# Introduction
- Head Tracking for Tele-operation of Interbotix WX200 Robotic Arm (5 DOF Robotic Arm, excluding gripper)
- The end effector of the robotic arm acts as the vision of the robotic head 
- Intel RealSense depth camera (D415) is mounted on the end effector of the robotic arm, as it acts as position of the robotic head
- Robotic arm acts as the the neck for the robotic head (pitch, yaw, roll, head lean forwards/backwards)
- Head Tracking Data is transmitted from Arduino to the Interbotix WX200 Robotic Arm

# Data Transmission
- With Arduino, the IMU sensor data are sent over the serial port to the computer (data transmission)
- Created a publisher node, sensor_data_publisher, which publishes the IMU data to the ROS2 topic, sensor_data
- Created a subscriber node, sensor_data_subscriber, which subscribes to the sensor_data topic and used the services from the Interbotix libraries to process 
  the IMU data in controlling the movement of the robotic arm 

# Services used from in-built ROS2 Packages from WX200 Interbotix Arm
1. bot_set_single_joint_position(): parameters - "joint_name = " and "position = positional value for servo motor (in radians) = "
- To control a selected single positional motor from one of the joint linkages of the robotic arm
2. bot_set_ee_cartesian_trajectory(): parameters - "x/y/z = end effector coordinates of robotic arm"
- Input the end effector coordinates of the robotic arm, so that its in-built inverse kinematics engine will control the positional motors accordingly to move the robotic arm to the given end effector position

# Controlling the Interbotix WX200 Robotic Arm with GY-85 9D0F IMU Sensor (teleop_IMU sensor.py)
- Obtaining pitch, roll and yaw data from the IMU sensor, to control the movements of the end effector of the robotic arm (Refer to Arduino code)
- Mapping the pitch, roll, yaw data in radians and calling the service, bot_set_single_position to input the data in controlling the positional servo motor of 
  the robotic arm
- For yaw movement, bot_set_single_position(joint_name = 'waist', position = yaw data from IMU sensor)
- For pitch movement, bot_set_single_position(joint_name = 'wrist_angle', position = pitch data from IMU sensor)
- For roll movement, bot_set_single_position(joint_name = 'wrist_rotate', position = roll data from IMU sensor)

# Controlling the Interbotix WX200 Robotic Arm with GY-85 9DOF IMU Sensor and HC-SR04 Ultrasonic Sensor (teleop_final.py)
- Obtaining pitch, roll and yaw data from the IMU sensor, to control the movements of the end effector of the robotic arm (Refer to Arduino code)
- Obtaining distance travelled from the ultrasonic sensor to control the linear movement (in the x-axis) of the end effector of the robotic arm (Refer to 
  Arduino code)
- The ultrasonic sensor helps to track the head movement of the user leaning forwards and backwards
- Linearly mapping the distance travelled from the ultrasonic sensor to x-coordinate positional values of the end effector of the robotic arm
- User's head leans forward, end effector x-coordinates increases. User's head leans backward, end effector x-coordinataes decreases
- For the linear movement (x-axis) of the end effector of robotic arm, need to control the positional servo motors located in the 'shoulder' and 'elbow' of the 
  robotic arm
- Considered the robotic arm as a 2DOF arm in the planar view and needed an Inverse Kinematics Solver for 2DOF Robotic Arm to determine the positional values of 
  the servo positional motors, in order to control the robotic arm to move to the given end effector position (Refer to IK for 2DOF Robotic Arm Python code)
- Calling the service, bot_set_single_position(joint_name = 'shoulder', position = positional value calculated from IK Solver for 2DOF Robotic Arm)
- Calling the service, bot_set_single_position(joint_name = 'elbow', position = positional value calculated from IK Solver for 2DOF Robotic Arm)

# Inverse Kinematics Solver for 2DOF Robotic Arm (Planar View)
- Created a Python script which simulates the movement of the robotic arm, in moving to the given end effector position from its initial home position
- The inverse kinematics solver in the Python script calculates the positional values of the servo motors in the 'elbow' and 'shoulder' linkage joints of the 
  arm, when end effector x-coordinates are given to move the robotic arm

# Considerations
- Long term goal would be to develop a program which takes in the IMU and ultrasonic sensor data and maps them as x,y,z cartesian coordinates for the end 
  effector of the arm
- With the end effector coordinates, utilise the inverse kinematics solver of the Interbotix robotic arm to control the positional motors of the arm accordingly 
  to move to its given end effector position





