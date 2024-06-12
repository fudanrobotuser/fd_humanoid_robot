#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <rclcpp/rclcpp.hpp>                              // ROS 2 C++客户端库
#include <trajectory_msgs/msg/joint_trajectory.hpp>       // ROS 2消息
#include <trajectory_msgs/msg/joint_trajectory.hpp>       // 包含ROS 2消息定义
#include <trajectory_msgs/msg/joint_trajectory_point.hpp> // 包含ROS 2消息定义
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>  // 添加这一行
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <filesystem>
#include "robot_control_module/csv_read.hpp"
class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode() : Node("robot_control_node")
    {
        // control_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        //     "control_topic", 10, std::bind(&RobotControlNode::controlCallback, this, std::placeholders::_1));

        // 创建订阅
        control_sub_ = this->create_subscription<std_msgs::msg::String>(
            "control_topic", 10, std::bind(&RobotControlNode::controlCallback, this, std::placeholders::_1));

        // ????Publisher
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory_controller/joint_trajectory", 1000);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds (8000), std::bind(&RobotControlNode::readCSV_And_SendTopic, this));

             // 订阅轨迹控制器的状态话题
        state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/trajectory_controller/controller_state", 10,
            std::bind(&RobotControlNode::stateCallback, this, std::placeholders::_1));

    }

private:
    void controlCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // readCSV_And_SendTopic();
    }

     void stateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
    {
        // 检查轨迹是否执行完毕
        if (isTrajectoryComplete(msg))
        {
            std::cout << "total_rows = " << total_rows<< std::endl;
            if (readCount < total_rows)
            {
                std::cout << "isTrajectoryComplete" << std::endl;
                // 发送新的轨迹
                // readCSV_And_SendTopic();
            }else{
                 std::cout << "-------ETHE END--------" << std::endl;
            }
        }else {
            // std::cout << "-------isTrajectoryComplete  false--------" << std::endl;
        }
    }

     bool isTrajectoryComplete(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
    {


        // RCLCPP_INFO(this->get_logger(), "Received state message.");

        //  std::cout << "-------control_msgs   msg --------" << std::endl;
            // 检查是否有轨迹点信息 // 如果缺少实际位置或期望位置数据，则无法判断轨迹是否完成
        if (msg->actual.positions.empty() || msg->desired.positions.empty()) {
            // std::cout << "-------positions is empty --------" << std::endl;
            return false;
        }

        std::cout << "-------msg->actual.positions size-----=" << msg->actual.positions.size() << std::endl;

        // 定义一个误差容限
        const double position_tolerance =10.0; // 位置容差
        // 检查每个关节的当前位置是否接近目标位置
        for (size_t i = 0; i < msg->actual.positions.size(); ++i) {
            double position_error = std::abs(msg->actual.positions[i] - msg->desired.positions[i]);
            // 如果位置或速度误差超过容差，说明轨迹尚未完成
            if (position_error > position_tolerance ) {
                std::cout << "---- position_actual ----" << msg->actual.positions[i] << "----- position_desired -----"  << msg->desired.positions[i] << std::endl;
                return false;
            }
        }

        // 如果所有关节的误差都在容差范围内，则认为轨迹已完成
        return true;
    }


    bool fileExists(const std::string &filePath)
    {
        return std::filesystem::exists(filePath) && std::filesystem::is_regular_file(filePath);
    }

    void readCSV_And_SendTopic()
    {
        // 创建CSVReader对象
        CSVReader reader;

        // 定义CSV文件名和要读取的行范围
        std::string filename = "/home/fudanrobotuser/fd_humanoid_robot/src/robot_control_module/motor0611170000.csv";
        if (fileExists(filename))
        {
            std::cout << "文件存在: " << filename << std::endl;
        }
        else
        {
            std::cout << "文件不存在: " << filename << std::endl;
        }

        std::ifstream file(filename);

        if (!file.is_open())
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }

        std::string file_content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        file.close();

        total_rows = std::count(file_content.begin(), file_content.end(), '\n');
        // size_t total_rows =751;
        std::cout << "Total number of rows in the CSV file: " << total_rows << std::endl;
        // 每次读取的行数
        int batch_size = maxRead;
        int sleep = 1;

        int remaining = total_rows - readCount;
        if (remaining < maxRead)
        {
            batch_size = remaining;
        }
        else
        {
            batch_size = maxRead;
        }

        if (readCount + batch_size <= total_rows)
        {
            std::string json_string = reader.readCSVToJSON(filename, readCount, readCount + batch_size);
            //  std::string json_string = reader.readCSVToJSON(filename, 1, 1002);

            // 输出JSON字符串
            //std::cout << json_string << std::endl;

            // 解析JSON字符串
            nlohmann::json root = nlohmann::json::parse(json_string);

            // 创建一个JointTrajectory消息实例
            trajectory_msgs::msg::JointTrajectory jt;
            // jt.header.stamp.sec = root["header"]["stamp"]["sec"].get<int>();
            // jt.header.stamp.nanosec = root["header"]["stamp"]["nanosec"].get<int>();
            jt.header.stamp = rclcpp::Clock().now();
            jt.header.frame_id = root["header"]["frame_id"].get<std::string>();

            // 设置关节名称
            for (const auto &name : root["joint_names"])
            {
                jt.joint_names.push_back(name.get<std::string>());
            }

            long referenceTime = 0;
            int p = 0;

            // 设置轨迹点
            for (const auto &point_json : root["points"])
            {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                // 获取JSON数组的长度
                int positions_length = point_json["positions"].size();
                int velocities_length = point_json["velocities"].size();
                int accelerations_length = point_json["accelerations"].size();

                // 确保数组长度匹配
                if (positions_length != velocities_length || positions_length != accelerations_length)
                {
                    std::cerr << "Lengths of positions, velocities, and accelerations must be the same." << std::endl;
                    return;
                }

                // 添加值到ROS 2消息数组
                for (int i = 0; i < positions_length; ++i)
                {
                    point.positions.push_back(point_json["positions"][i].get<double>());
                    point.velocities.push_back(point_json["velocities"][i].get<double>());
                    point.accelerations.push_back(point_json["accelerations"][i].get<double>());
                }

                point.time_from_start.sec = point_json["time_from_start"]["sec"].get<int>();
                
                point.time_from_start.nanosec = point_json["time_from_start"]["nanosec"].get<int>();

                jt.points.push_back(point);
               
            }

            

            publisher_->publish(jt);

            readCount = readCount + batch_size + 1;

            std::cout << "The readCount=" << readCount << std::endl;

            if (remaining < maxRead)
            {
                std::cout << "The End readCount=" << readCount << std::endl;
                // 停止并销毁定时器
                timer_->reset();
                timer_.reset();
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points_;

    int total_rows;
    int maxRead = 1500;
    int readCount = 1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControlNode>());
    rclcpp::shutdown();
    return 0;
}
