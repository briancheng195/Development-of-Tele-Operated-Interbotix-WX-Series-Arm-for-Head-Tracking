// ROS 2 C++ Node to Read Array of Floats from Serial
#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include "std_msgs/msg/float32_multi_array.hpp"
//#include "std_msgs/msg/int32_multi_array.hpp"

class SerialNode : public rclcpp::Node {
public:
  SerialNode() : Node("serial_node") {
    // Set up serial communication
    serial_port = std::make_unique<serial::Serial>("/dev/ttyACM0", 19200); // Adjust the port name

    // Create a publisher for the sensor data
    sensor_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("sensor_data", 10);

    // Create a timer to periodically read sensor data from serial
    timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SerialNode::readSensorData, this));
  }

private:
  void readSensorData() {
    if (serial_port->available()) {
      std::string data = serial_port->readline();
      std_msgs::msg::Float32MultiArray msg;
      size_t pos = 0;
      while ((pos = data.find(",")) != std::string::npos) {
        msg.data.push_back(std::stof(data.substr(0, pos)));
        data.erase(0, pos + 1);
      }
      msg.data.push_back(std::stof(data)); // Add the last float value
      
      sensor_publisher->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Publishing:'%f''%f''%f'", msg.data[0], msg.data[1], msg.data[2]);
    }
  }

  std::unique_ptr<serial::Serial> serial_port;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr sensor_publisher;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}

