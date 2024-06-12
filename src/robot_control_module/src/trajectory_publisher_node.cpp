
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

struct JointTrajectoryData {
    std::vector<std::string> joint_names;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
};

JointTrajectoryData read_csv(const std::string &filename) {
    JointTrajectoryData data;
    std::ifstream file(filename);
    std::string line, cell;

    // 读取关节点名称
    if (std::getline(file, line)) {
        std::istringstream lineStream(line);
        // 跳过第一个时间列
        std::getline(lineStream, cell, ',');
        while (std::getline(lineStream, cell, ',')) {
            // 清除回车符
            cell.erase(std::remove(cell.begin(), cell.end(), '\r'), cell.end());
            data.joint_names.push_back(cell);
        }
    }

    // 读取关节位置数据
    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        trajectory_msgs::msg::JointTrajectoryPoint point;
        double time;
        // 读取时间列
        std::getline(lineStream, cell, ',');
        std::stringstream(cell) >> time;
        point.time_from_start = rclcpp::Duration::from_seconds(time);
        // 读取每个关节点的位置
        while (std::getline(lineStream, cell, ',')) {
            double position;
            std::stringstream(cell) >> position;
            point.positions.push_back(position);
        }
        data.points.push_back(point);
    }

    return data;
}

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher(const std::string &filename): Node("trajectory_publisher"), data_(read_csv(filename)), point_index_(0) {

        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory_controller/joint_trajectory", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(7500), std::bind(&TrajectoryPublisher::publish_next_batch, this));

    }

private:
    void publish_next_batch() {
        if (point_index_ < data_.points.size()) {
            auto message = trajectory_msgs::msg::JointTrajectory();
            message.joint_names = data_.joint_names;

            size_t end_index = std::min(point_index_ + batch_size_, data_.points.size());
            for (size_t i = point_index_; i < end_index; ++i) {
                message.points.push_back(data_.points[i]);
            }
            publisher_->publish(message);
            point_index_ = end_index;
        } else {
            // 数据发送完毕，重新开始
            RCLCPP_INFO(this->get_logger(), "All points have been published");
            timer_->reset();
            timer_.reset();
        }
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    JointTrajectoryData data_;
    size_t point_index_;
    const size_t batch_size_ = 1000; // 每次发送5条数据
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::string filename = "/home/fudanrobotuser/fd_humanoid_robot/src/robot_control_module/motor1.csv";
    auto node = std::make_shared<TrajectoryPublisher>(filename);
    rclcpp::spin(node);
    rclcpp::shutdown();
}