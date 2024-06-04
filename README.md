# Development-of-Tele-Operated-Interbotix-WX-Series-Arm for Head Tracking
- Developed Python algorithms with ROS2 for tele-operation and manipulation of Interbotix X-Series Robotic Arm
- This project is done up with ROS2 and the ROS2 library packages with the Interbotix WX Series Robotic Arm

# Introduction
- Head Tracking for Tele-operation of Interbotix WX200 Robotic Arm (5 DOF Robotic Arm, excluding gripper)
- The end effector of the robotic arm acts as the vision of the robotic head 
- Intel RealSense depth camera (D415) is mounted on the end effector of the robotic arm, as it acts as position of the robotic head
- Robotic arm acts as the the neck for the robotic head (pitch, yaw, roll, head lean forwards/backwards)
- Head Tracking Data is transmitted from Arduino to the Interbotix WX200 Robotic Arm
- This approach was inspired by the humanoid design of Team NimbRo, the winner of the ANA Avatar XPrize Competition in 2021. Their design was to incorporate a 5-DOF robotic arm to 
  act as the neck of the robot’s head, so as to accurately control the head movements of the humanoid (http://nimbro.net/AVATAR/).
  <img width="400" alt="image" src="https://github.com/briancheng195/Development-of-Tele-Operated-Interbotix-WX-Series-Arm-for-Head-Tracking/assets/122734373/8e224312-aeeb-42ca-9029-bbd2ea7da359">

# Proof of Concept 
<img width="550" alt="image" src="https://github.com/briancheng195/Development-of-Tele-Operated-Interbotix-WX-Series-Arm-for-Head-Tracking/assets/122734373/19343372-8528-4a1c-a1c0-de3573d1b646">

# Data Transmission
- With Arduino, the IMU sensor data are sent over the serial port to the computer (ROS2 C++ node to read array of floats and create sensor_data_publisher)
- Created a publisher node, sensor_data_publisher, which publishes the IMU data to the ROS2 topic, sensor_data
- Created a subscriber node, sensor_data_subscriber, which subscribes to the sensor_data topic and used the services from the Interbotix libraries to process 
  the IMU data in controlling the movement of the robotic arm 

# Services used from in-built ROS2 Packages from WX200 Interbotix Arm
1. bot_set_single_joint_position(): parameters - "joint_name = " and "position = positional value for servo motor (in radians) = "
- To control a selected single positional motor from one of the joint linkages of the robotic arm
2. bot_set_ee_cartesian_trajectory(): parameters - "x/y/z = end effector coordinates of robotic arm"
- Input the end effector coordinates of the robotic arm, so that its in-built inverse kinematics engine will control the positional motors accordingly to move the robotic arm to the given end effector position

# Overview of Interbotix WX200 Robotic Arm and its Basic Movements
<img width="500" alt="image" src="https://github.com/briancheng195/Development-of-Tele-Operated-Interbotix-WX-Series-Arm-for-Head-Tracking/assets/122734373/7391cd0c-9456-4ef6-849a-93f64730cf3e">

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

# Electronics Design for Head Tracking Device
<img width="500" alt="image" src="https://github.com/briancheng195/Development-of-Tele-Operated-Interbotix-WX-Series-Arm-for-Head-Tracking/assets/122734373/def403ca-99ab-4852-9e2a-8037d1fead54">
<img width="400" alt="image" src="https://github.com/briancheng195/Development-of-Tele-Operated-Interbotix-WX-Series-Arm-for-Head-Tracking/assets/122734373/d864fcb6-4fa8-4bb1-a8e1-6bdfbafae36f">

# Inverse Kinematics Solver for 2DOF Robotic Arm (Planar View)
- Created a Python script which simulates the movement of the robotic arm, in moving to the given end effector position from its initial home position
- The inverse kinematics solver in the Python script calculates the positional values of the servo motors in the 'elbow' and 'shoulder' linkage joints of the 
  arm, when end effector x-coordinates are given to move the robotic arm

# Video Demonstration
https://youtu.be/YtYOpwMk1rM

# Considerations
- Long term goal would be to develop a program which takes in the IMU and ultrasonic sensor data and maps them as x,y,z cartesian coordinates for the end 
  effector of the arm
- With the end effector coordinates, utilise the inverse kinematics solver of the Interbotix robotic arm to control the positional motors of the arm accordingly 
  to move to its given end effector position

# Future Developments
- To develop an algorithm which takes in the IMU and ultrasonic sensors data and maps them as cartesian coordinates for the end effector of the arm. With the end effector 
  coordinates, I would utilise the Inverse Kinematics solver in the robotic arm to control the positional motors of the arm accordingly
- There is a time lag for this project’s tele-operation approach. This is because the robotic arm has to process every dataset from the IMU and ultrasonic sensors, before 
  completing its arm movement. This sequence of data processing results in the delay in controls
- To consider another method in transmitting the IMU and ultrasonic sensors data to the robotic arm, to reduce the time lag for tele-operation. For example, possibility of using
  the microcontroller, ESP32, for wireless transmission of data, as its Wi-Fi and Bluetooth functionality can transmit the IMU and ultrasonic sensors data from the Arduino to the    Robotic Arm through its SPI or I2C interfaces







