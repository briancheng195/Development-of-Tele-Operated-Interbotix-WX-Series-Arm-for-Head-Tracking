import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import math

def main():
    # Initialize the robot arm
    bot = InterbotixManipulatorXS(
        robot_model='wx200',
        group_name='arm',
        gripper_name='gripper'
    )

    if (bot.arm.group_info.num_joints < 5):
        bot.core.get_logger().fatal('This demo requires the robot to have at least 5 joints!')
        bot.shutdown()
        sys.exit()

    def map_value(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def inverse_kinematics(x, y, L_shoulder, L_elbow):
        #Calculate the inverse kinematics for the Interbotix Arm (2DOF Planar).
        #Parameters: x,y - desired end-effector position
        #Parameters: L_shoulder, L_elbow - lengths of the shoulder and elbow links
        #Returns: theta_shoulder and theta_elbow - joint angles in radians

        #Calculate theta_elbow
        D = (x**2 + y**2 - L_shoulder**2 - L_elbow**2) / (2 * L_shoulder * L_elbow)
        theta_elbow = math.atan2(-math.sqrt(1-D**2), D)

        #Calculate theta_shoulder
        K1 = L_shoulder + L_elbow * math.cos(theta_elbow)
        K2 = L_elbow * math.sin(theta_elbow)
        theta_shoulder = math.atan2(y,x) - math.atan2(K2, K1)
        theta_shoulder = -theta_shoulder

        return theta_shoulder, theta_elbow
    
    bot.arm.go_to_home_pose()

    def sensor_data_callback(msg):
        for data in msg.data:
            bot.core.get_logger().info(f'received accelerometer data: {data}')

        pitch_mapped = map_value(msg.data[2], -50, 50, np.pi/4, -np.pi/4)
        print(f'pitch movement: {pitch_mapped}')

        yaw_mapped = map_value(msg.data[1], -30, 30, np.pi/2, -np.pi/2)
        print(f'yaw movement: {yaw_mapped}')
        
        linear_mapped = map_value(msg.data[0], -5, 5, 248.32, 500)
        print(f'linear movement: {linear_mapped}')
        
        L_shoulder = 206.16
        L_elbow = 200+174.15
        end_effector_x = linear_mapped   #x=374.16, y=0 is home pose
        end_effector_y = 206.16
        theta_shoulder, theta_elbow = inverse_kinematics(end_effector_x, end_effector_y, L_shoulder, L_elbow)

        # To control the yaw movements of the robot arm
        bot.arm.set_single_joint_position(joint_name = 'wrist_angle', position=pitch_mapped)
        bot.arm.set_single_joint_position(joint_name = 'waist', position=yaw_mapped)
        bot.arm.set_single_joint_position(joint_name='elbow', position = -(theta_elbow+np.pi/2))
        bot.arm.set_single_joint_position(joint_name='shoulder', position=theta_shoulder+np.pi/2)

    sensor_node = Node('sensor_data_subscriber')
    subscription = sensor_node.create_subscription(
        Float32MultiArray,
        'sensor_data',
        sensor_data_callback,
        10
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(sensor_node, timeout_sec=1.0)  # Process events for up to 1 second
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
