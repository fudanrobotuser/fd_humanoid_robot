#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

int gap = 1000;
std::vector<std::string> csv_joint_names_ = {"W1L",	"W2R",	"W3B",	"M1L",	"M2L",	"M3L",	"M4L",	"M5L",	"M6L",	"M1R",	"M2R",	"M3R",	"M4R",	"M5R",	"M6R"};

bool arrived = true;
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
        // point.time_from_start = rclcpp::Duration::from_seconds(time);
        // 读取每个关节点的位置
        while (std::getline(lineStream, cell, ',')) {
            double position;
            std::stringstream(cell) >> position;
            point.time_from_start =  rclcpp::Duration(0, 8000000);
            point.positions.push_back(position);
        }
        data.points.push_back(point);
    }

    return data;
}

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher(const std::string &filename): Node("trajectory_publisher"), data_(read_csv(filename)), point_index_(0) {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory_controller/joint_trajectory", 1);
        timer_ = this->create_wall_timer( std::chrono::milliseconds(1),std::bind(&TrajectoryPublisher::publish_trajectory, this));
        subscription_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>( "/joint_states", 1, std::bind(&TrajectoryPublisher::joint_state_topic_callback, this, std::placeholders::_1));
    }



private:

    void joint_state_topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
       {
            if (csv_joint_names_.size() != msg->name.size())
            {
                RCLCPP_INFO(this->get_logger(), "jonit_names.size() != msg->name.size()");
                return;
            }

           int avg = 0;
           for (size_t i = 0; i < msg->name.size(); ++i) {
            //    RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", msg->name[i].c_str(), msg->position[i]);
               for(int j=0;j<sizeof(csv_joint_names_);j++){
                    std::string name = csv_joint_names_[j];
                    if(name.compare(msg->name[i].c_str())==0){
                        avg += abs(msg->position[i] - data_.points[point_index_].positions[j]) ;
                        break;
                    }                    
               }
                
           }
           if( avg/12 <= gap ){
             arrived = true;
             std::cout << "arrived" << std::endl;
           }
       }

    void publish_trajectory() {
        if (point_index_ >= data_.points.size()) {
            RCLCPP_INFO(this->get_logger(), "All points have been published");
            timer_->reset();
            timer_.reset();
            return;
        }

        trajectory_msgs::msg::JointTrajectory jt;
        jt.joint_names = data_.joint_names;
        jt.points.push_back(data_.points[point_index_]);
        jt.header.stamp = this->now();
        jt.header.frame_id = "";
        if(arrived){
            publisher_->publish(jt);
            arrived = false;
            point_index_++;
            std::cout << "point_index_" << point_index_ << std::endl;
        }
         
    }




    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    JointTrajectoryData data_;
    size_t point_index_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::string filename = "/home/fudanrobotuser/fd_humanoid_robot/src/robot_control_module/motor.csv";
    auto node = std::make_shared<TrajectoryPublisher>(filename);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
