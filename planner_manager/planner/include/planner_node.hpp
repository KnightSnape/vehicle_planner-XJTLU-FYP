#pragma once

#include<planner_process.hpp>

class PlannerNode : public rclcpp::Node
{
    public:
        typedef std::shared_ptr<PlannerNode> Ptr;
        PlannerNode();
        ~PlannerNode();
        void node_init();

        void EstimaterInit();
        void initial_pos_transform();
        void mid360PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void lioOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void roomPosCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
        void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void goalvecCallback(const nav_msgs::msg::Path::SharedPtr msg);
        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

        void publishTwist(const geometry_msgs::msg::TwistStamped& twist_msg);
        void publishBspline(const planner::msg::Bspline& bspline_msg);

        void mainProcess();
        void sensorStateTimerInit();
        void sensorStateCheckerCallback();
        void odomTimerCallback();

    private:
        std::string package_path_root;

        std::string odom_topic_name;
        std::string room_pos_topic_name;
        std::string imu_topic_name;
        std::string gps_topic_name;
        std::string cloud_topic_name;
        std::string target_pos_name;
        std::string target_pos_vec_name;
        std::string bspline_topic_name;
        std::string depth_topic_name;

        std::string odom_frame_id;
        std::string map_frame_id;
        std::string base_frame_id;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr las_odom_sub;
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr _room_pos_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goal_vec_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
        rclcpp::Publisher<planner::msg::Bspline>::SharedPtr bspline_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_now_pub;

        rclcpp::TimerBase::SharedPtr _sensor_state_timer_;
        rclcpp::TimerBase::SharedPtr _odom_state_pub_timer_;

        nav_msgs::msg::Odometry mid_odom;
        sensor_msgs::msg::PointCloud2 mid_cloud;
        geometry_msgs::msg::TransformStamped room_pos;
        nav_msgs::msg::Path target_pos_vec;
        sensor_msgs::msg::NavSatFix gps_pos;
        sensor_msgs::msg::Imu imu_msg;
        sensor_msgs::msg::Image depth_msg;

        std::mutex target_mutex;
        std::condition_variable target_cv;

        std::mutex sensor_mutex;

        std::condition_variable sensor_cv;

        std::vector<Eigen::Vector3d> gps_pos_vec;
        std::vector<Eigen::Vector3d> mid360_pos_vec;
        std::vector<Eigen::Quaterniond> mid360_quat_vec;
        std::vector<Eigen::Quaterniond> imu_quat_vec;

        std::chrono::steady_clock::time_point last_mid360_pointcloud_time;
        std::chrono::steady_clock::time_point last_las_odom_time;
        std::chrono::steady_clock::time_point last_gps_time;
        std::chrono::steady_clock::time_point last_amcl_room_time;
        std::chrono::steady_clock::time_point last_imu_time;
        std::chrono::steady_clock::time_point last_depth_time;

        PlannerEstimatorPtr planner_estimator;
        std::thread mainprocess_thread;

        int sensor_state;

        bool is_initialized;
        bool is_navigating;
        bool have_target;

};