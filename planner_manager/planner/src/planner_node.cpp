#include<planner_node.hpp>
#include<plugin/backward.hpp>
#include<sensor_msgs/point_cloud2_iterator.hpp>
#include<pcl_conversions/pcl_conversions.h>
namespace backward
{
    backward::SignalHandling sh;
}

double getFarthestPointDistance(const sensor_msgs::msg::PointCloud2 &cloud) {
    // 初始化最大距离为 0
    double max_distance = 0.0;

    // 创建点云迭代器，遍历 x, y, z 坐标
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    // 遍历点云数据
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // 计算点到原点的欧几里得距离
        double distance = std::sqrt((*iter_x) * (*iter_x) + (*iter_y) * (*iter_y) + (*iter_z) * (*iter_z));
        // 更新最大距离
        if (distance > max_distance) {
            max_distance = distance;
        }
    }
    return max_distance;
}

PlannerNode::PlannerNode()
    : Node("planner")
{
    RCLCPP_INFO(this->get_logger(), "planner node init");
    is_initialized = false;
    is_navigating = false;
    have_target = false;
    sensor_state = SENSOR_STATE::OFFLINE;
    node_init();
    sensorStateTimerInit();
}

PlannerNode::~PlannerNode()
{
    if(mainprocess_thread.joinable())
    {
        mainprocess_thread.join();
    }
}

void PlannerNode::EstimaterInit()
{
    //init estimator
    planner_estimator = std::make_shared<PlannerEstimator>(this);
}

void PlannerNode::node_init()
{
    package_path_root = ament_index_cpp::get_package_share_directory("planner");

    odom_topic_name = this->declare_parameter("odom_topic_name", "/fastlio2/lio_odom");
    room_pos_topic_name = this->declare_parameter("room_pos_topic_name", "/room_pos");
    gps_topic_name = this->declare_parameter("gps_topic_name", "/gps");
    cloud_topic_name = this->declare_parameter("cloud_topic_name", "/fastlio2/world_cloud");
    target_pos_name = this->declare_parameter("target_pos_name", "/target_pos");
    target_pos_vec_name = this->declare_parameter("target_pos_vec_name", "/path4local");
    imu_topic_name = this->declare_parameter("imu_topic_name", "/livox/imu");
    depth_topic_name = this->declare_parameter("depth_topic_name", "camera/depth/image_rect_raw");
    //image_topic_name

    las_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_name, 10, std::bind(&PlannerNode::lioOdometryCallback, this, std::placeholders::_1));
    _room_pos_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(room_pos_topic_name, 10, std::bind(&PlannerNode::roomPosCallback, this, std::placeholders::_1));
    gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic_name, 10, std::bind(&PlannerNode::gpsCallback, this, std::placeholders::_1));
    cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_name, 10, std::bind(&PlannerNode::mid360PointCloudCallback, this, std::placeholders::_1));
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_name, 10, std::bind(&PlannerNode::imuCallback, this, std::placeholders::_1));

    goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(target_pos_name, 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    goal_vec_sub = this->create_subscription<nav_msgs::msg::Path>(target_pos_vec_name, 10, std::bind(&PlannerNode::goalvecCallback, this, std::placeholders::_1));
    depth_sub = this->create_subscription<sensor_msgs::msg::Image>(depth_topic_name, 10, std::bind(&PlannerNode::depthCallback, this, std::placeholders::_1));


    bspline_topic_name = this->declare_parameter("bspline_topic_name", "/bspline");

    bspline_pub = this->create_publisher<planner::msg::Bspline>(bspline_topic_name, 10);
    odom_now_pub = this->create_publisher<nav_msgs::msg::Odometry>("/mid_odom", 10);

    _sensor_state_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PlannerNode::sensorStateCheckerCallback, this));
    _odom_state_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PlannerNode::odomTimerCallback, this));
    mainprocess_thread = std::thread(&PlannerNode::mainProcess, this);
    RCLCPP_INFO(this->get_logger(), "planner node init success");
}

void PlannerNode::initial_pos_transform()
{
    /*
    RCLCPP_INFO(this->get_logger(),"initlal GPS and MID360 transform, please Don't Move, Maybe take for seconds");
    std::unique_lock<std::mutex> lock(sensor_mutex);
    sensor_cv.wait(lock, [&]() {
        return gps_pos_vec.size() == 50 && mid360_pos_vec.size() == 50 && imu_quat_vec.size() == 1000;
    });
    */
}

void PlannerNode::mid360PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    mid_cloud = *msg;
    if(!(sensor_state & SENSOR_STATE::MID360_ONLINE))
    {
        sensor_state |= SENSOR_STATE::MID360_ONLINE;
        last_mid360_pointcloud_time = std::chrono::steady_clock::now();
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr mid360_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(mid_cloud, *mid360_cloud);
    planner_estimator->sdf_map_->updateMapWithCloud(mid360_cloud);
}

void PlannerNode::lioOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    mid_odom = *msg;
    if(!(sensor_state & SENSOR_STATE::MID360_ONLINE))
    {
        sensor_state |= SENSOR_STATE::MID360_ONLINE;
        last_las_odom_time = std::chrono::steady_clock::now();
    }
    double sensor_offset_x = -0.12;  
    double sensor_offset_y = -0.07; 
    Eigen::Vector3d pos{mid_odom.pose.pose.position.x, mid_odom.pose.pose.position.y, mid_odom.pose.pose.position.z};
    Eigen::Quaterniond quat{mid_odom.pose.pose.orientation.w, mid_odom.pose.pose.orientation.x, mid_odom.pose.pose.orientation.y, mid_odom.pose.pose.orientation.z};
    Eigen::Vector3d ypr = quat.toRotationMatrix().eulerAngles(2, 1, 0);
    double yaw = ypr(0);
    Eigen::Vector3d car_pos = pos - (Eigen::Vector3d(cos(yaw) * sensor_offset_x - sin(yaw) * sensor_offset_y, sin(yaw) * sensor_offset_x + cos(yaw) * sensor_offset_y, 0));
    planner_estimator->setOdomstate(true);
    planner_estimator->sdf_map_->setOdomPos(msg);
    planner_estimator->updateCurrentPos(car_pos);
    planner_estimator->updateCurrentVel(Eigen::Vector3d(mid_odom.twist.twist.linear.x, mid_odom.twist.twist.linear.y, mid_odom.twist.twist.linear.z));
    planner_estimator->updateCurrentQuat(Eigen::Quaterniond(mid_odom.pose.pose.orientation.w, mid_odom.pose.pose.orientation.x, mid_odom.pose.pose.orientation.y, mid_odom.pose.pose.orientation.z));
    RCLCPP_INFO(this->get_logger(), "%f %f %f", car_pos(0), car_pos(1), car_pos(2));
}

void PlannerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_msg = *msg;
    if(!(sensor_state & SENSOR_STATE::IMU_ONLINE))
    {
        sensor_state |= SENSOR_STATE::IMU_ONLINE;
        last_imu_time = std::chrono::steady_clock::now();
    }
}

void PlannerNode::roomPosCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    /*
    room_pos = *msg;
    if(!(sensor_state & SENSOR_STATE::AMCL_NODE_ONLINE))
    {
        sensor_state |= SENSOR_STATE::AMCL_NODE_ONLINE;
        last_amcl_room_time = std::chrono::steady_clock::now();
    }
    */
}

void PlannerNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    gps_pos = *msg;
    if(!(sensor_state & SENSOR_STATE::GPS_ONLINE))
    {
        sensor_state |= SENSOR_STATE::GPS_ONLINE;
        last_gps_time = std::chrono::steady_clock::now();
    }
    if(gps_pos_vec.size() < 50)
    {
        Eigen::Vector3d gps_pos_eigen(gps_pos.latitude, gps_pos.longitude, gps_pos.altitude);
        gps_pos_vec.push_back(gps_pos_eigen);
        if(gps_pos_vec.size() == 50)
        sensor_cv.notify_one();//notify gps data
    }
}

void PlannerNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    depth_msg = *msg;
    if(!(sensor_state & SENSOR_STATE::CAMERA_ONLINE))
    {
        sensor_state |= SENSOR_STATE::CAMERA_ONLINE;
        last_depth_time = std::chrono::steady_clock::now();
    }
    planner_estimator->sdf_map_->depthhandle(msg);
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    target_pos_vec.poses.clear();
    target_pos_vec.poses.push_back(*msg);
    have_target = true;
    planner_estimator->OnWayPointCallback(target_pos_vec);
}

void PlannerNode::goalvecCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    target_pos_vec = *msg;
    have_target = true;
    planner_estimator->OnWayPointCallback(target_pos_vec);
}

void PlannerNode::sensorStateTimerInit()
{
    last_mid360_pointcloud_time = std::chrono::steady_clock::now();
    last_las_odom_time = std::chrono::steady_clock::now();
    last_gps_time = std::chrono::steady_clock::now();
    last_amcl_room_time = std::chrono::steady_clock::now();
    last_imu_time = std::chrono::steady_clock::now();
    last_depth_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "sensor state timer init success");
}

void PlannerNode::sensorStateCheckerCallback()
{
    auto now = std::chrono::steady_clock::now();
    double mid360_diff = std::chrono::duration_cast<std::chrono::seconds>(now - last_mid360_pointcloud_time).count();
    double odom_diff = std::chrono::duration_cast<std::chrono::seconds>(now - last_las_odom_time).count();
    double gps_diff = std::chrono::duration_cast<std::chrono::seconds>(now - last_gps_time).count();
    double amcl_diff = std::chrono::duration_cast<std::chrono::seconds>(now - last_amcl_room_time).count();
    double imu_diff = std::chrono::duration_cast<std::chrono::seconds>(now - last_imu_time).count();
    double depth_diff = std::chrono::duration_cast<std::chrono::seconds>(now - last_depth_time).count();
    if(mid360_diff > 5 && !(sensor_state & SENSOR_STATE::MID360_ONLINE))
    {
        sensor_state &= ~SENSOR_STATE::MID360_ONLINE;
        RCLCPP_WARN(this->get_logger(), "mid360 offline");
    }
    if(odom_diff > 5 && !(sensor_state & SENSOR_STATE::MID360_ONLINE))
    {
        sensor_state &= ~SENSOR_STATE::MID360_ONLINE;
        RCLCPP_WARN(this->get_logger(), "odom offline");
    }
    if(gps_diff > 5 && !(sensor_state & SENSOR_STATE::GPS_ONLINE))
    {
        sensor_state &= ~SENSOR_STATE::GPS_ONLINE;
        RCLCPP_WARN(this->get_logger(), "gps offline");
    }
    if(amcl_diff > 5 && !(sensor_state & SENSOR_STATE::AMCL_NODE_ONLINE))
    {
        sensor_state &= ~SENSOR_STATE::AMCL_NODE_ONLINE;
        RCLCPP_WARN(this->get_logger(), "amcl node offline");
    }
    if(imu_diff > 5 && !(sensor_state & SENSOR_STATE::IMU_ONLINE))
    {
        sensor_state &= ~SENSOR_STATE::IMU_ONLINE;
        RCLCPP_WARN(this->get_logger(), "imu offline");
    }
    if(depth_diff > 5 && !(sensor_state & SENSOR_STATE::CAMERA_ONLINE))
    {
        sensor_state &= ~SENSOR_STATE::CAMERA_ONLINE;
        RCLCPP_WARN(this->get_logger(), "camera offline");
    }
}

void PlannerNode::publishTwist(const geometry_msgs::msg::TwistStamped& twist_msg)
{
    twist_pub->publish(twist_msg);
}

void PlannerNode::publishBspline(const planner::msg::Bspline& bspline_msg)
{
    bspline_pub->publish(bspline_msg);
}

void PlannerNode::mainProcess()
{
    /*
    while(rclcpp::ok())
    {
        std::unique_lock<std::mutex> lock(target_mutex);
        target_cv.wait(lock, [this](){ return have_target; });
        lock.unlock();
        planner_estimator->setSensorState(sensor_state);
        planner_estimator->updateCurrentPos(Eigen::Vector3d(mid_odom.pose.pose.position.x, mid_odom.pose.pose.position.y, mid_odom.pose.pose.position.z));
        planner_estimator->updateCurrentVel(Eigen::Vector3d(mid_odom.twist.twist.linear.x, mid_odom.twist.twist.linear.y, mid_odom.twist.twist.linear.z));
        planner_estimator->updateCurrentQuat(Eigen::Quaterniond(mid_odom.pose.pose.orientation.w, mid_odom.pose.pose.orientation.x, mid_odom.pose.pose.orientation.y, mid_odom.pose.pose.orientation.z));
        planner_estimator->OnWayPointCallback(target_pos_vec);
        have_target = false;
    }
    */
}

void PlannerNode::odomTimerCallback()
{
    if(!(sensor_state & SENSOR_STATE::MID360_ONLINE))
    {
        return;
    }
    // 发布里程计数据
    nav_msgs::msg::Odometry odom_state = planner_estimator->getOdomState();
    odom_now_pub->publish(odom_state);
}

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PlannerNode>();
    node->EstimaterInit();
    try
    {
        rclcpp::spin(node);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Exception caught: " << e.what() << std::endl;

        // 使用 backward-cpp 捕获和打印堆栈跟踪信息
        backward::StackTrace st;
        st.load_here(32);

        backward::TraceResolver tr;
        tr.load_stacktrace(st);

        for (size_t i = 0; i < st.size(); ++i) {
            backward::ResolvedTrace trace = tr.resolve(st[i]);
            std::cout << "#" << i
                      << " " << trace.object_filename
                      << " " << trace.object_function
                      << " [" << trace.addr << "]"
                      << std::endl;
        }
    }
    rclcpp::shutdown();
    return 0;
}
