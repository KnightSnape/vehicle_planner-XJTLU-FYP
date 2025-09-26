#pragma once

#include<geometry_msgs/msg/twist_stamped.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<rclcpp/rclcpp.hpp>
#include<non_uniform_bspline.hpp>
#include<planner/msg/bspline.hpp>
#include<std_msgs/msg/empty.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include<chrono>

using planner::NonUniformBspline;

class TrajectoryServer : public rclcpp::Node
{
    public:
        TrajectoryServer();
        ~TrajectoryServer() = default;
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void bsplineCallback(const planner::msg::Bspline::SharedPtr msg);
        void newSubCallback(const std_msgs::msg::Empty::SharedPtr msg);
        void replanSubCallback(const std_msgs::msg::Empty::SharedPtr msg);

        void cmdTimerCallback();
        void publishTwist();

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<planner::msg::Bspline>::SharedPtr bspline_sub_;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr new_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr traj_pub_;

        rclcpp::TimerBase::SharedPtr cmd_timer_;

        std::string new_sub_topic_;
        std::string replan_sub_topic_;
        std::string odom_topic_;
        std::string bspline_topic_;

        std::vector<NonUniformBspline> traj_;

        std::vector<Eigen::Vector3d> traj_cmd_, traj_real_;

        std::chrono::system_clock::time_point start_time_;

        nav_msgs::msg::Odometry odom;
        double traj_duration_;
        int traj_id;
        double last_yaw_;

        bool receive_traj_;

};