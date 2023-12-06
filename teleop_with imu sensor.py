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

    bot.arm.go_to_home_pose()

    def sensor_data_callback(msg):

        for data in msg.data:
            bot.core.get_logger().info(f'received accelerometer data: {data}')
        
        pitch_mapped = map_value(msg.data[2], -90, 90, np.pi/2, -np.pi/2)
        print(f'pitch movement: {pitch_mapped}')

        yaw_mapped = map_value(msg.data[1], -90, 90, np.pi/2, -np.pi/2)
        print(f'yaw movement: {yaw_mapped}')

        linear_mapped = map_value(msg.data[0], -90, 90, -np.pi/2, np.pi/2)
        print(f'linear movement: {roll_mapped}')

        # To control the yaw movements of the robot arm
        bot.arm.set_single_joint_position(joint_name = 'elbow', position=pitch_mapped)
        bot.arm.set_single_joint_position(joint_name = 'waist', position=yaw_mapped)
        bot.arm.set_single_joint_position(joint_name = 'wrist_angle', position=roll_mapped)

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
