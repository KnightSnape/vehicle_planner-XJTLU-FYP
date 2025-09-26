#include<Eigen/Eigen>
#include<yaml-cpp/yaml.h>
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/pose.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<nav_msgs/msg/path.hpp>
#include<thread>
#include<cstdio>
#include<vector>
#include<ament_index_cpp/get_package_share_directory.hpp>
#include<cstring>

enum class DEMO_TYPE
{
    GLOBAL,
    LOCAL,
    GLOBAL_PLUS_LOCAL
};

DEMO_TYPE demo_type = DEMO_TYPE::GLOBAL;

class TestPathPublisher : public rclcpp::Node
{
    public:
        TestPathPublisher():Node("test_path_publisher")
        {
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/target_pos_vec",10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000), std::bind(&TestPathPublisher::mainprocess, this));
        }
    private:
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        void mainprocess()
        {
            std::string package_path = ament_index_cpp::get_package_share_directory("planner");
            std::string yaml_base_path = package_path + "/config/test_path_config/";
            switch(demo_type)
            {
                case DEMO_TYPE::GLOBAL:
                    yaml_base_path += "global_plan_line.yaml";
                    break;
                case DEMO_TYPE::LOCAL:
                    yaml_base_path += "easy_line.yaml";
                    break;
                case DEMO_TYPE::GLOBAL_PLUS_LOCAL:
                    yaml_base_path += "fusion.yaml";
                    break;
            }
            YAML::Node config = YAML::LoadFile(yaml_base_path);
            std::vector<geometry_msgs::msg::PoseStamped> points;
            const YAML::Node& points_node = config["points"];
            for (const auto& point_node : points_node) {
                geometry_msgs::msg::PoseStamped point;
                point.pose.position.x = point_node["x"].as<double>();
                point.pose.position.y = point_node["y"].as<double>();
                point.pose.position.z = point_node["z"].as<double>();
                points.push_back(point);
            }

            for (const auto& point : points) {
                RCLCPP_INFO(this->get_logger(), "Point: (%.2f, %.2f, %.2f)", point.pose.position.x, point.pose.position.y, point.pose.position.z);
            }

            nav_msgs::msg::Path path;
            path.header.frame_id = "lidar";
            path.header.stamp = this->now();
            path.poses = points;
            path_pub_->publish(path);
        }

        void complete()
        {
            std::cout<<"Publish Complete"<<std::endl;
            exit(0);
        }
    
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TestPathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}