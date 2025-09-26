#pragma once

#include<Eigen/Eigen>

#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/nav_sat_fix.hpp>   
#include<sensor_msgs/msg/imu.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include<nav_msgs/msg/path.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<planner/msg/bspline.hpp>
#include<std_msgs/msg/empty.hpp>    
#include<std_msgs/msg/int32.hpp>

#include<ament_index_cpp/get_package_share_directory.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<geometry_msgs/msg/twist_stamped.hpp>
#include<geometry_msgs/msg/pose_array.hpp>
#include<geometry_msgs/msg/transform_stamped.hpp>
#include<geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include<geometry_msgs/msg/point32.hpp>
#include<builtin_interfaces/msg/time.hpp>

enum SENSOR_STATE
{
    OFFLINE = 0,
    MID360_ONLINE = 1,
    GPS_ONLINE = 2,
    CAMERA_ONLINE = 4,
    AMCL_NODE_ONLINE = 8,
    RADAR_ONLINE = 16,
    IMU_ONLINE = 32,
};

enum GLOBAL_TYPE
{
    NONE = 0,
    INROOM,
    OUTSIDE,
    EXPLORE,
};

enum TRAJ_OPTIONS
{
    STRAIGHT = 0,
    POLY,
    BSPLINE,
};

enum GRAPH_STATE
{
    STAND_BY = 0,
    FIND_LATEST_AREA, //寻找最近的区域
    ON_TRAJECTORY,  //沿预定轨迹行驶
    BACK_TO_POINT,  //返回指定点
    
};

enum GLOBAL_FSM_STATE
{
    INIT = 0,
    WAIT_TARGET = 1,
    GEN_NEW_TRAJ = 2,
    REPLAN_TRAJ = 3,
    EXEC_TRAJ = 4,
    REPLAN_NEW = 5,
};

