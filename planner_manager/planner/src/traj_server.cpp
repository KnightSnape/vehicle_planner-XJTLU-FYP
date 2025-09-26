#include<traj_server.hpp>

TrajectoryServer::TrajectoryServer():Node("traj_server")
{
    traj_id = 0;
    traj_duration_ = 0.0;
    last_yaw_ = 0.0;

    odom_topic_ = this->declare_parameter("odom_topic", "/fastlio2/lio_odom");
    bspline_topic_ = this->declare_parameter("bspline_topic", "/bspline");

    new_sub_topic_ = this->declare_parameter("new_sub_topic", "/new");
    replan_sub_topic_ = this->declare_parameter("replan_sub_topic", "/replan");

    new_sub_ = this->create_subscription<std_msgs::msg::Empty>(new_sub_topic_, 10, std::bind(&TrajectoryServer::newSubCallback, this, std::placeholders::_1));
    replan_sub_ = this->create_subscription<std_msgs::msg::Empty>(replan_sub_topic_, 10, std::bind(&TrajectoryServer::replanSubCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10, std::bind(&TrajectoryServer::odomCallback, this, std::placeholders::_1));
    bspline_sub_ = this->create_subscription<planner::msg::Bspline>(bspline_topic_, 10, std::bind(&TrajectoryServer::bsplineCallback, this, std::placeholders::_1));

    traj_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/traj", 10);

    cmd_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TrajectoryServer::cmdTimerCallback, this));
    receive_traj_ = false;
}

void TrajectoryServer::newSubCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
    traj_cmd_.clear();
    traj_real_.clear();
}

void TrajectoryServer::replanSubCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
    const double timeout = 0.01;
    auto t = std::chrono::system_clock::now();
    double t_stop = std::chrono::duration<double>(t - start_time_).count();
    traj_duration_ = min(traj_duration_, t_stop);
}

void TrajectoryServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if(msg->child_frame_id == "X" || msg->child_frame_id == "O")
        return;
    odom = *msg;
    traj_real_.push_back(Eigen::Vector3d(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z));
    if (traj_real_.size() > 10000) 
        traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);//erase the first 1000 points
}

void TrajectoryServer::bsplineCallback(const planner::msg::Bspline::SharedPtr msg)
{
    Eigen::MatrixXd pos_pts(msg->pos_pts.size(),3);
    Eigen::VectorXd knots(msg->knots.size());

    for(int i = 0; i < msg->pos_pts.size(); i++)
    {
        pos_pts(i,0) = msg->pos_pts[i].x;
        pos_pts(i,1) = msg->pos_pts[i].y;
        pos_pts(i,2) = msg->pos_pts[i].z;
    }
    for(int i = 0; i < msg->knots.size(); i++)
    {
        knots(i) = msg->knots[i];
    }

    NonUniformBspline pos_traj(pos_pts,msg->order,0.1);
    pos_traj.setKnot(knots);

    Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(),1);
    for(int i = 0; i < msg->yaw_pts.size(); i++)
    {
        yaw_pts(i,0) = msg->yaw_pts[i];
    }

    NonUniformBspline yaw_traj(yaw_pts,msg->order,msg->yaw_dt);
    start_time_ = std::chrono::system_clock::now(); 
    traj_id = msg->traj_id;
    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());
    traj_.push_back(yaw_traj);
    traj_.push_back(yaw_traj.getDerivative());

    traj_duration_ = traj_[0].getTimeSum();
    receive_traj_ = true;
}

void TrajectoryServer::cmdTimerCallback()
{
    if(!receive_traj_)
    {
        return;
    }
    auto time = std::chrono::system_clock::now();
    double t_diff = (time - start_time_) / std::chrono::milliseconds(1) / 1000.0;
    Eigen::Vector3d pos, vel, acc, pos_f;
    double yaw, yaw_dot;
    if(t_diff < traj_duration_ && t_diff >= 0.0)
    {
        pos = traj_[0].evaluateDeBoor(t_diff);
        vel = traj_[1].evaluateDeBoor(t_diff);
        acc = traj_[2].evaluateDeBoor(t_diff);
        yaw = traj_[3].evaluateDeBoor(t_diff)(0);
        yaw_dot = traj_[4].evaluateDeBoor(t_diff)(0);
        double tf = min(traj_duration_, t_diff + 2.0);//get the final time
        pos_f = traj_[0].evaluateDeBoor(tf);//get the final position
    }
    else if(t_diff >= traj_duration_)
    {
        pos = traj_[0].evaluateDeBoor(traj_duration_);
        vel = Eigen::Vector3d::Zero();
        acc = Eigen::Vector3d::Zero();
        yaw = traj_[3].evaluateDeBoor(traj_duration_)(0);
        yaw_dot = traj_[4].evaluateDeBoor(traj_duration_)(0);
        pos_f = pos;
    }

    geometry_msgs::msg::Twist traj_msg;
    //TODO: check the publish value
    traj_msg.linear.x = vel(0);
    traj_msg.linear.y = vel(1);
    traj_msg.linear.z = vel(2);
    traj_msg.angular.x = 0.0;
    traj_msg.angular.y = 0.0;
    if(abs(yaw_dot > M_PI)) yaw_dot = 0;
    traj_msg.angular.z = yaw_dot;
    traj_pub_->publish(traj_msg);

    last_yaw_ = yaw;
    auto pos_err = pos_f - Eigen::Vector3d(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
    Eigen::Vector3d dir(cos(yaw),sin(yaw),0.0);
    traj_cmd_.push_back(pos);
    if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TrajectoryServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}