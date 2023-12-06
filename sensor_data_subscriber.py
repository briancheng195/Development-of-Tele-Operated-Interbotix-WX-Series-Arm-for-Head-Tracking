import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SensorDataSubscriber(Node):
	def __init__(self):
		super().__init__('sensor_data_subscriber')
		self.subscription = self.create_subscription(Float32MultiArray,'sensor_data',self.sensor_data_callback,10)
		self.subscription
		
	def sensor_data_callback(self, msg):
		for data in msg.data:
			self.get_logger().info(f'received accelerometer data: {data}')
			
def main(args=None):
	rclpy.init(args=args)
	node = SensorDataSubscriber()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
