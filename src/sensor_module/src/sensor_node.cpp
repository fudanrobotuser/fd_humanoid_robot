#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"

class SensorNode : public rclcpp::Node {
public:
    SensorNode() : Node("sensor_node") {
        sensor_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("sensor_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&SensorNode::publishSensorData, this));
    }

private:
    void publishSensorData() {
        auto message = sensor_msgs::msg::Temperature();
        message.temperature = 25.0; // 模拟温度数据
        sensor_pub_->publish(message);
    }

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sensor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode>());
    rclcpp::shutdown();
    return 0;
}
