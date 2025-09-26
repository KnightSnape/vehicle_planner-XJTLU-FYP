#include<planner_process.hpp>
#include<planner_node.hpp>

using namespace std::chrono_literals;

//#define AUTO
//#define USE_3D

PlannerEstimator::PlannerEstimator(PlannerNode* node): planner_node_(node)
{
    have_target_ = false;
    trigger_ = false;
    have_odom_ = false;
    global_type = GLOBAL_TYPE::NONE;
    traj_options = TRAJ_OPTIONS::BSPLINE;
    sensor_state = SENSOR_STATE::OFFLINE;

    std::string package_path_root = ament_index_cpp::get_package_share_directory("planner");
    std::string room_graph_path = package_path_root + "/config/test_config.yaml";
    std::string initial_path = package_path_root + "/config/initial.yaml";
    sdf_map_ = std::make_shared<SDFMap>(node, package_path_root);
    paraminit(initial_path);
    initModules(package_path_root);
    room_graph_ = std::make_shared<RoomGraph>(package_path_root);
    visualization_ = std::make_shared<PlanningVisualization>(planner_node_);

    new_pub = planner_node_->create_publisher<std_msgs::msg::Empty>("/new_traj", 10);
    replan_pub = planner_node_->create_publisher<std_msgs::msg::Empty>("/replan_traj", 10);
    fsm_pub = planner_node_->create_publisher<std_msgs::msg::Int32>("/fsm_state", 10);

    cycle_cnt = 0;
    machine_changer = planner_node_->create_wall_timer(0.01s, std::bind(&PlannerEstimator::PlannerProcess, this));
    if(traj_options != TRAJ_OPTIONS::STRAIGHT)
    {
        collision_checker = planner_node_->create_wall_timer(0.01s, std::bind(&PlannerEstimator::CollisionCheckProcess, this));
    }

    RCLCPP_INFO(planner_node_->get_logger(), "planner estimator init success");
}

PlannerEstimator::~PlannerEstimator()
{
    delete planner_node_;
}

void PlannerEstimator::paraminit(std::string package_path)
{
    YAML::Node config = YAML::LoadFile(package_path);

    pp_.max_vel_ = config["max_vel"].as<double>();
    pp_.max_acc_ = config["max_acc"].as<double>();
    pp_.max_jerk_ = config["max_jerk"].as<double>();
    pp_.dynamic_ = config["dynamic_environment"].as<int>();
    pp_.clearance_ = config["clearance_threshold"].as<double>();
    pp_.local_traj_len_ = config["local_segment_length"].as<double>();
    pp_.ctrl_pt_dist = config["control_points_distance"].as<double>();

    use_geometric_path = config["use_geometric_path"].as<bool>();
    use_topo_path = config["use_topo_path"].as<bool>();
    use_optimization = config["use_optimization"].as<bool>();
    use_active_perception_ = config["use_active_perception"].as<bool>();

    global_type = static_cast<GLOBAL_TYPE>(config["global_type"].as<int>());
    target_type = static_cast<TARGET_TYPE>(config["target_type"].as<int>());
    replan_time_threshold_ = config["thresh_replan"].as<double>();
    replan_distance_threshold_ = config["thresh_no_replan"].as<double>();
    waypoint_num_ = config["waypoint_num"].as<int>();
    for(int i = 0; i < waypoint_num_; i++)
    {
        waypoints_[i][0] = config["waypoints"][i][0].as<double>();
        waypoints_[i][1] = config["waypoints"][i][1].as<double>();
        waypoints_[i][2] = config["waypoints"][i][2].as<double>();
    }

    RCLCPP_INFO(planner_node_->get_logger(), "param init success");
    printGlobalType(global_type);
    RCLCPP_INFO(planner_node_->get_logger(), "target_type: %d", target_type);
}

void PlannerEstimator::initModules(std::string package_path)
{
    edt_environment_ = std::make_shared<EDTEnvironment>();
    edt_environment_->setMap(sdf_map_);

    std::cout<<"EDT Environment init success"<<std::endl;

    if(use_optimization)
    {
        bspline_optimizers_.resize(10);
        for(int i = 0; i < 10; i++)
        {
            bspline_optimizers_[i].reset(new BsplineOptimizer);
            bspline_optimizers_[i]->setParams(package_path);
            bspline_optimizers_[i]->setEnvironment(edt_environment_);
        }

        bspline_optimizers_2d_.resize(10);
        for(int i = 0; i < 10; i++)
        {
            bspline_optimizers_2d_[i].reset(new BsplineOptimizer2d);
            bspline_optimizers_2d_[i]->setParams(package_path);
            bspline_optimizers_2d_[i]->setEnvironment(edt_environment_);
        }
    }
    std::cout<<"Bspline Optimization init success"<<std::endl;

    if(use_geometric_path)
    {
        geo_path_finder_.reset(new Astar);
        geo_path_finder_->setEnvironment(edt_environment_);
        geo_path_finder_->init();
    }

    std::cout<<"Geometric Path Finder init success"<<std::endl;

    if(use_topo_path)
    {
        topo_prm_.reset(new TopologyPRM);
        topo_prm_->setEnvironment(edt_environment_);
        topo_prm_->init(package_path);
    }

    std::cout<<"Topology Path Finder init success"<<std::endl;
}

void PlannerEstimator::resetGlobalWayPoints()
{
    global_way_points_.clear();
    have_target_ = false;
    trigger_ = false;
}

void PlannerEstimator::setOdomstate(bool state)
{
    have_odom_ = state;
}

void PlannerEstimator::updateCurrentPos(const Eigen::Vector3d& pos)
{
    RCLCPP_INFO(planner_node_->get_logger(), "Current Pos: %f %f %f", pos(0), pos(1), pos(2));
    current_pos = pos;
}

void PlannerEstimator::updateCurrentVel(const Eigen::Vector3d& vel)
{
    current_vel = vel;
}

void PlannerEstimator::updateCurrentQuat(const Eigen::Quaterniond& quat)
{
    current_quat = quat;
}

void PlannerEstimator::setSensorState(int state)
{
    sensor_state = state;
}

nav_msgs::msg::Odometry PlannerEstimator::getOdomState()
{
    nav_msgs::msg::Odometry odom_state;
    odom_state.header.frame_id = "lidar";
    odom_state.header.stamp = planner_node_->now();
    odom_state.pose.pose.position.x = current_pos(0);
    odom_state.pose.pose.position.y = current_pos(1);
    odom_state.pose.pose.position.z = current_pos(2);
    odom_state.twist.twist.linear.x = current_vel(0);
    odom_state.twist.twist.linear.y = current_vel(1);
    odom_state.twist.twist.linear.z = current_vel(2);
    odom_state.pose.pose.orientation.w = current_quat.w();
    odom_state.pose.pose.orientation.x = current_quat.x();
    odom_state.pose.pose.orientation.y = current_quat.y();
    odom_state.pose.pose.orientation.z = current_quat.z();
    return odom_state;
}

void PlannerEstimator::printGlobalType(GLOBAL_TYPE type)
{
    std::string type_str[4] = {"NONE", "INROOM", "OUTSIDE", "EXPLORE"};
    RCLCPP_INFO(planner_node_->get_logger(), "Global Type: %s", type_str[int(type)].c_str());
}

void PlannerEstimator::CheckSensorState()
{
#ifdef AUTO
    if((sensor_state & SENSOR_STATE::MID360_ONLINE) && (sensor_state & SENSOR_STATE::GPS_ONLINE))
    {
        global_type = GLOBAL_TYPE::OUTSIDE;
        have_odom_ = true;
    }
    else if((sensor_state & SENSOR_STATE::MID360_ONLINE) && (sensor_state & SENSOR_STATE::AMCL_NODE_ONLINE))
    {
        global_type = GLOBAL_TYPE::INROOM;
        have_odom_ = true;
    }
    else
    {
        global_type = GLOBAL_TYPE::NONE;
    }
#endif
#ifndef AUTO
    if(global_type == GLOBAL_TYPE::OUTSIDE)
    {
        if((sensor_state & SENSOR_STATE::MID360_ONLINE) && (sensor_state & SENSOR_STATE::GPS_ONLINE))
        {
            have_odom_ = true;
        }
    }
    else if(global_type == GLOBAL_TYPE::INROOM)
    {
        if((sensor_state & SENSOR_STATE::MID360_ONLINE) && (sensor_state & SENSOR_STATE::AMCL_NODE_ONLINE))
        {
            have_odom_ = true;
        }
    }
    else if(global_type == GLOBAL_TYPE::EXPLORE)
    {
        if(sensor_state & SENSOR_STATE::MID360_ONLINE) 
        {
            have_odom_ = true;
        }
    }
    else
    {
        have_odom_ = false;
    }
#endif
}

bool PlannerEstimator::ReachTarget(const Eigen::Vector3d& target_pos_now)
{
    return (current_pos - target_pos_now).norm() < 0.2;
}

void PlannerEstimator::OnWayPointCallback(const nav_msgs::msg::Path& msg)
{
    if(!have_target_)
    {
        way_point = msg;
        HandlerGoal();
    }
    else
    {
        RCLCPP_WARN(planner_node_->get_logger(), "Already have target, ignore new target");
    }
}

void PlannerEstimator::HandlerGoal()
{
    std::deque<Eigen::Vector3d> global_up;
    CheckSensorState();
    if(target_type == TARGET_TYPE::REFENCE_PATH)
    {
        for(int i = 0; i < waypoint_num_; i++)
        {
            Eigen::Vector3d pt;
            pt(0) = waypoints_[i][0];
            pt(1) = waypoints_[i][1];
            pt(2) = waypoints_[i][2];
            global_up.push_back(pt);
        }
    }
    else
    {
        if(target_type == TARGET_TYPE::MANUAL_TARGET)
        {
            for(int i = 0; i < way_point.poses.size(); i++)
            {
                Eigen::Vector3d pt;
                pt(0) = way_point.poses[i].pose.position.x;
                pt(1) = way_point.poses[i].pose.position.y;
                pt(2) = way_point.poses[i].pose.position.z;
                global_up.push_back(pt);
            }
            target_point_(0) = way_point.poses[way_point.poses.size() - 1].pose.position.x;
            target_point_(1) = way_point.poses[way_point.poses.size() - 1].pose.position.y;
            target_point_(2) = way_point.poses[way_point.poses.size() - 1].pose.position.z;
        }
        else if(target_type == TARGET_TYPE::PRESET_TARGET)
        {
            target_point_(0) = waypoints_[current_wp_][0];
            target_point_(1) = waypoints_[current_wp_][1];
            target_point_(2) = waypoints_[current_wp_][2];
            current_wp_ = ( current_wp_ + 1) % waypoint_num_;
            global_up.push_back(target_point_);
        }
        visualization_->drawGoal(target_point_, 0.1, Eigen::Vector4d(1, 0, 0, 1), 1);
    }

    end_vel.setZero();
    plan_data_.global_waypoints_ = global_up;
    global_way_points_ = global_up;
    have_target_ = true;
    trigger_ = true;
    if(fsm_state == GLOBAL_FSM_STATE::WAIT_TARGET)
        ChangeGlobalFSMState(GLOBAL_FSM_STATE::GEN_NEW_TRAJ, "HandlerGoal");
}

void PlannerEstimator::ChangeGlobalFSMState(GLOBAL_FSM_STATE fsm_state_update, std::string from)
{
    std::string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "REPLAN_NEW"};

    int pre_state = int(fsm_state);
    fsm_state = fsm_state_update;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "From [%s] Places, FSM state change from %s to %s",from,state_str[pre_state].c_str(), state_str[int(fsm_state)].c_str());
}

void PlannerEstimator::printFSMExecState() {
  string state_str[6] = { "INIT",
                          "WAIT_TARGET",
                          "GEN_NEW_TRAJ",
                          "REPLAN_TRAJ",
                          "EXEC_TRAJ",
                          "REPLAN_"
                          "NEW" };
  cout << "state: " + state_str[int(fsm_state)] << endl;
}

void PlannerEstimator::PlannerProcess()
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "PlannerProcess");
    cycle_cnt++;
    if(cycle_cnt % 200 == 0)
    {
        printFSMExecState();
        if(!have_odom_)
        {
            RCLCPP_WARN(planner_node_->get_logger(), "No odom data");
        }
        if(!trigger_)
        {
            RCLCPP_WARN(planner_node_->get_logger(), "No target data");
        }
        cycle_cnt = 0;
    }
    publishState();
    switch(fsm_state)
    {
        case INIT:
        {
            if(!have_odom_) return;
            if(!trigger_) return;
            ChangeGlobalFSMState(WAIT_TARGET, "PlannerProcess");
            break;
        }

        case WAIT_TARGET:
        {
            if(!have_target_)
                return;
            else
                ChangeGlobalFSMState(GEN_NEW_TRAJ, "PlannerProcess");
            break;
        }

        case GEN_NEW_TRAJ:
        {
            start_pt_ = current_pos;
            start_vel_ = current_vel;
            start_acc_.setZero();

            Eigen::Vector3d rot_x = current_quat.toRotationMatrix().block(0, 0, 3, 1);
            start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
            start_yaw_(1) =         start_yaw_(2) = 0.0;

            if(ReachTarget(target_point_))//near_target
            {
                ChangeGlobalFSMState(WAIT_TARGET, "PlannerProcess");
                have_target_ = false;
                break;
            }

            bool success = callTopologicalTraj(1);
            if(success)
            {
                ChangeGlobalFSMState(EXEC_TRAJ, "PlannerProcess");
            }
            else
                ChangeGlobalFSMState(GEN_NEW_TRAJ, "PlannerProcess");
            break;
        }

        case EXEC_TRAJ:
        {
            if(traj_options != TRAJ_OPTIONS::STRAIGHT)
            {
                GlobalTrajData* global_data = &global_data_;
                auto time_now = std::chrono::steady_clock::now();
                double t_cur = std::chrono::duration<double>(time_now - global_data->global_start_time_).count();
                if(t_cur > global_data->global_duration_ && ReachTarget(target_point_)) //到达目标点
                {
                    have_target_ = false;
                    ChangeGlobalFSMState(WAIT_TARGET, "PlannerProcess");
                    return;
                }
                else
                {
                    LocalTrajData* info = &local_data_;
                    Eigen::Vector3d current_real_pos = current_pos;
                    Eigen::Vector3d start_pos = info->start_pos_;
                    t_cur = std::chrono::duration<double>(time_now - info->start_time_).count();
                    Eigen::Vector3d cur_pos = info->position_traj_.evaluateDeBoorT(t_cur);
                    // if((current_real_pos - cur_pos).norm() > 1.5) //车偏离轨迹
                    // {
                    //    RCLCPP_WARN(planner_node_->get_logger(), "Vehicle deviate from the traj, disable local_traj and replan");
                    //    ChangeGlobalFSMState(GEN_NEW_TRAJ, "PlannerProcess");//不用local_traj规划，直接重新规划global_traj
                    //     break;
                    // }
                    if(t_cur > replan_time_threshold_)//到达重新规划时间阈值
                    {
                        if(!global_data->localTrajReachTarget())
                        {
                            ChangeGlobalFSMState(REPLAN_TRAJ, "PlannerProcess");
                        }
                        else
                        {
                            Eigen::Vector3d end_pos = info->position_traj_.evaluateDeBoorT(info->duration_);
                            if((cur_pos - end_pos).norm() > replan_distance_threshold_)
                            {
                                ChangeGlobalFSMState(REPLAN_TRAJ, "PlannerProcess");
                            } 
                        }
                    }
                }
                break;
            }
            break;
        }

        case REPLAN_TRAJ:
        {
            LocalTrajData* info = &local_data_;
            auto time_now = std::chrono::steady_clock::now();
            double t_cur = std::chrono::duration<double>(time_now - info->start_time_).count();

            start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
            start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
            start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

            start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)(0);
            start_yaw_(1) = start_yaw_(2) = 0.0;//车辆不会翻滚

            bool success = callTopologicalTraj(2);
            if(success)
            {
                ChangeGlobalFSMState(EXEC_TRAJ, "PlannerProcess");
            }
            else
            {
                RCLCPP_WARN(planner_node_->get_logger(), "Replan failed, go back to GEN_NEW_TRAJ");
            }
            break;
        }
        case REPLAN_NEW:
        {

            LocalTrajData* info = &local_data_;
            auto time_now = std::chrono::steady_clock::now();
            double t_cur = std::chrono::duration<double>(time_now - info->start_time_).count();

            start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
            start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
            start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

            new_pub->publish(std_msgs::msg::Empty());
            bool success = callTopologicalTraj(2);
            if(success)
            {
                ChangeGlobalFSMState(EXEC_TRAJ, "PlannerProcess");
            }
            else
            {
                ChangeGlobalFSMState(GEN_NEW_TRAJ, "PlannerProcess");
            }
            break;
        }
    }

}

void PlannerEstimator::updateTrajInfo()
{
    local_data_.velocity_traj_       = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_   = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_           = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_            = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
}

void PlannerEstimator::refineTraj(NonUniformBspline& best_traj, double& time_inc)
{
    auto t1 = std::chrono::steady_clock::now();
    time_inc = 0.0;
    double dt, t_inc;
    const int max_iter = 1;

    Eigen::MatrixXd ctrl_pts = best_traj.getControlPoints();
    int cost_function = BsplineOptimizer::NORMAL_PHASE;
    best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
    double ratio = best_traj.checkRatio();
    reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
}

void PlannerEstimator::reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                      double& time_inc)
{
    int prev_num         = bspline.getControlPoints().rows();
    double time_origin   = bspline.getTimeSum();
    int seg_num          = bspline.getControlPoints().rows() - 3;

    ratio = min(1.01, ratio);
    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;
    std::vector<Eigen::Vector3d> point_set;
    for(double time = 0.0; time <= duration + 1e-4; time += dt)
    {
        point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_, ctrl_pts);
}

void PlannerEstimator::findCollisionRange(std::vector<Eigen::Vector3d>& colli_start,
                                std::vector<Eigen::Vector3d>& colli_end,
                                std::vector<Eigen::Vector3d>& start_pts,
                                std::vector<Eigen::Vector3d>& end_pts){
    bool               last_safe = true, safe;
    double             t_m, t_mp;
    NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
    initial_traj->getTimeSpan(t_m, t_mp);
    
    /* find range of collision */
    double t_s = -1.0, t_e;
    for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {
    
        Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
        safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;
    
        if (last_safe && !safe) {
        colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
        if (t_s < 0.0) t_s = tc - 0.05;
        } else if (!last_safe && safe) {
        colli_end.push_back(ptc);
        t_e = tc;
        }
    
        last_safe = safe;
    }
    
    if (colli_start.size() == 0) return;
    
    if (colli_start.size() == 1 && colli_end.size() == 0) return;
    
    /* find start and end safe segment */
    double dt = initial_traj->getInterval();
    int    sn = ceil((t_s - t_m) / dt);
    dt        = (t_s - t_m) / sn;
    
    for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
        start_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
    
    dt = initial_traj->getInterval();
    sn = ceil((t_mp - t_e) / dt);
    dt = (t_mp - t_e) / sn;
    // std::cout << "dt: " << dt << std::endl;
    // std::cout << "sn: " << sn << std::endl;
    // std::cout << "t_m: " << t_m << std::endl;
    // std::cout << "t_mp: " << t_mp << std::endl;
    // std::cout << "t_s: " << t_s << std::endl;
    // std::cout << "t_e: " << t_e << std::endl;
    
    if (dt > 1e-4) {
        for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
        end_pts.push_back(initial_traj->evaluateDeBoor(tc));
        }
    } else {
        end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
    }
}

void PlannerEstimator::CollisionCheckProcess()
{
    LocalTrajData* info = &local_data_;
    if(false)
    {

    }

    if(fsm_state == EXEC_TRAJ || fsm_state == REPLAN_TRAJ)
    {
        double dist; 
        bool safe = checkTrajCollision(dist);
        if(!safe)
        {
            if(dist > 0.3)
            {
                collide_ = true;
                ChangeGlobalFSMState(REPLAN_TRAJ, "CollisionCheckProcess");
            }
            else
            {
                RCLCPP_ERROR(planner_node_->get_logger(), "Collision happened, stop the vehicle");
                replan_pub->publish(std_msgs::msg::Empty()); 
                have_target_ = false;
                ChangeGlobalFSMState(WAIT_TARGET, "CollisionCheckProcess");
            }
        }
        else
        {
            collide_ = false;
        }
    }
}

Eigen::MatrixXd PlannerEstimator::reparamLocalTraj(double start_t, double& dt, double& duration)
{
    std::vector<Eigen::Vector3d> point_set;
    std::vector<Eigen::Vector3d> start_end_derivative;
    global_data_.getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set, start_end_derivative, dt, duration);
    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    plan_data_.local_start_end_derivative_ = start_end_derivative; 
    return ctrl_pts;
}

Eigen::MatrixXd PlannerEstimator::reparamLocalTraj(double start_t, double duration, int seg_num, double& dt)
{
    std::vector<Eigen::Vector3d> point_set;
    std::vector<Eigen::Vector3d> start_end_derivative;
    global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
    plan_data_.local_start_end_derivative_ = start_end_derivative;

    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    return ctrl_pts;
}

EDTEnvironment::Ptr PlannerEstimator::getEDTEnvironment()
{
    return edt_environment_;
}

bool PlannerEstimator::callTopologicalTraj(int step)
{
    bool plan_success;
    if(step == 1)
    {
        switch(global_type)
        {
            case GLOBAL_TYPE::INROOM:
                plan_success = RoomPlannerProcess();
                break;
            case GLOBAL_TYPE::OUTSIDE:
                OutSidePlannerProcess();
                break;
            case GLOBAL_TYPE::EXPLORE:
                plan_success = ExplorePlannerProcess();
                break;
            default:
                break;
        }
    }
    else
    {
        switch(global_type)
        {
            case GLOBAL_TYPE::INROOM:
                break;
            case GLOBAL_TYPE::OUTSIDE:
                break;
            case GLOBAL_TYPE::EXPLORE:
                plan_success = topoReplan();
                break;
        }
    }

    if(plan_success)
    {
        std::cout<<"Start Plan Yaw"<<std::endl;
        planYaw(start_yaw_);
        LocalTrajData* localdat = &local_data_;
        planner::msg::Bspline bspline_msg;
        bspline_msg.start_time = toROSTime(localdat->start_time_);
        bspline_msg.order = 3;
        bspline_msg.traj_id = localdat->traj_id_;
        Eigen::MatrixXd pos_pts = localdat->position_traj_.getControlPoints();
        for(int i = 0;i < pos_pts.rows();i++)
        {
            geometry_msgs::msg::Point pt;
            pt.x = pos_pts(i, 0);
            pt.y = pos_pts(i, 1);
            pt.z = pos_pts(i, 2);
            bspline_msg.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = localdat->position_traj_.getKnot();
        for(int i = 0;i < knots.rows();i++)
        {
            bspline_msg.knots.push_back(knots(i));
        }
        bool checkEnd = checkEndTraj(pos_pts, target_point_);
        
        Eigen::MatrixXd yaw_pts = localdat->yaw_traj_.getControlPoints();
        for(int i = 0;i < yaw_pts.rows();i++)
        {
            double yaw = yaw_pts(i, 0);
            bspline_msg.yaw_pts.push_back(yaw);
        }

        bspline_msg.yaw_dt = localdat->yaw_traj_.getInterval();
        planner_node_->publishBspline(bspline_msg);

        MidPlanData* plan_data = &plan_data_;
        visualization_->drawPolynomialTraj(global_data_.global_traj_, 0.05, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), 0);
        visualization_->drawBspline(localdat->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), false, 0.15, Eigen::Vector4d(1.0, 1.0, 1.0, 1),99,99);
        visualization_->drawBsplinesPhase2(plan_data->topo_traj_pos2_, 0.075);
        return true;
    }
    else
    {
        return false;
    }
}

bool PlannerEstimator::RoomPlannerProcess()
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RoomPlannerProcess");
    int idx = room_graph_->CheckInPoint(current_pos);
    if(idx == -1)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can't find the point in the room graph");
        return false;
    }
    int target_idx = room_graph_->CheckInPoint(global_way_points_.front());
    if(target_idx == -1)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can't find the target point in the room graph");
        return false;
    }
    int first_id = room_graph_->get_first_point(idx, target_idx);
    Eigen::Vector3d target_pos = room_graph_->getNodePlace(target_idx);
    Eigen::Vector3d first_pos = room_graph_->getNodePlace(first_id);
    int dotvalue = room_graph_->CheckAngleWeight(first_pos, target_pos, current_pos);
    //Will be writen in a struct
    std::vector<int> path_sequence = room_graph_->get_shortest_path_sequence(idx, target_idx);
    std::deque<Eigen::Vector3d> local_path;
    for(auto &&id : path_sequence)
    {
        local_path.push_front(room_graph_->getNodePlace(id));
    }
    if(dotvalue < 0)
    {
        local_path.push_front(current_pos);
    }
    std::vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;
    if(traj_options != TRAJ_OPTIONS::STRAIGHT)//优化策略
    {
        for(int i = 0;i < local_path.size() - 1; i++)
        {
            inter_points.push_back(local_path[i]);
            double dist = (local_path[i] - local_path[i + 1]).norm(); // 欧式距离
            if(dist > dist_thresh)
            {
                int id_num = floor(dist / dist_thresh) + 1;
                for(int j = 1;j < id_num;j++)
                {
                    Eigen::Vector3d inter_point = local_path[i] + (local_path[i + 1] - local_path[i]) * (double)j / (double)id_num;//线性插值TODO
                    inter_points.push_back(inter_point);
                }
            }
        }
        inter_points.push_back(local_path[local_path.size() - 1]);//最后一个点
        if(inter_points.size() == 2)
        {
            Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) / 2;
            inter_points.insert(inter_points.begin() + 1, mid);
        }

        int pt_num = inter_points.size();
        Eigen::MatrixXd pos(pt_num, 3);
        for(int i = 0;i < pt_num;i++)
        {
            pos.row(i) = inter_points[i];
        }
        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        for(int i = 0;i < pt_num - 1;i++)
        {
            time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
        }
        time(0) *= 2.0;
        time(0) = max(1.0, time(0));
        time(time.rows() - 1) *= 2.0;
        time(time.rows() - 1) = max(1.0, time(time.rows() - 1));
        PolynomialTraj gl_traj = PolynomialTraj::minSnapTraj(pos, zero, zero, zero, zero, time);
        auto time_now = std::chrono::steady_clock::now();
        global_data_.setGlobalTraj(gl_traj, time_now);

        double dt, duration;
        Eigen::MatrixXd ctrl_pts = reparamLocalTraj(0.0, dt, duration);
        NonUniformBspline bspline(ctrl_pts, 3, dt);

        global_data_.setLocalTraj(bspline, 0.0, duration, 0.0); 
        local_data_.position_traj_   = bspline;
        local_data_.start_time_      = time_now;
        updateTrajInfo();

    }
    else//直线策略
    {
        
    }   

}

bool PlannerEstimator::topoReplan()
{
    std::chrono::steady_clock::time_point t1,t2,time_now;
    time_now = std::chrono::steady_clock::now();
    double t_now = std::chrono::duration<double>(time_now - global_data_.global_start_time_).count();
    double local_traj_dt, local_traj_duration;
    double time_inc = 0.0;
    Eigen::MatrixXd ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
    NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);

    local_data_.start_time_ = time_now;
    if(!collide_)//simply truncate the segment and do nothing
    {   
        refineTraj(init_traj, time_inc);
        local_data_.position_traj_ = init_traj;
        global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);
    }
    else
    {
        plan_data_.initial_local_segment_ = init_traj;
        std::vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
        findCollisionRange(colli_start, colli_end, start_pts, end_pts);
        if(colli_start.size() == 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No collision found, maybe some bugs in the code");
            return false;
        }
        if(colli_start.size() == 1 && colli_end.size() == 0)
        {
            local_data_.position_traj_ = init_traj;
            global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);
        }
        else
        {
            NonUniformBspline best_traj;
            plan_data_.clearTopoPaths();
            std::list<GraphNode::Ptr> graph;
            std::vector<std::vector<Eigen::Vector3d> > raw_paths, filtered_paths, selected_paths;
            topo_prm_->findTopoPath(colli_start.front(), colli_end.back(), start_pts, end_pts, graph,
                               raw_paths, filtered_paths, selected_paths);
            //visualization topo path
            visualization_->drawTopoGraph(graph, 0.1, 0.08, 
                            Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), Eigen::Vector4d(0.0, 0.5, 1.0, 1.0), 
                            Eigen::Vector4d(0.0, 1.0, 1.0, 1.0), 50);
            if(selected_paths.size() == 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "No topological path found");
                return false;
            }
            visualization_paths(selected_paths);
            plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, selected_paths);
            auto t1 = std::chrono::steady_clock::now();
            plan_data_.topo_traj_pos1_.resize(selected_paths.size());
            plan_data_.topo_traj_pos2_.resize(selected_paths.size());
            std::vector<std::thread> optimize_threads;
            for(int i = 0;i < selected_paths.size();i++)
            {
                optimize_threads.push_back(std::thread(&PlannerEstimator::optimizeTopoBspline, this, t_now, local_traj_duration, selected_paths[i], i));
            }
            for(int i = 0;i < optimize_threads.size();i++)
            {
                optimize_threads[i].join();
            }
            auto t2 = std::chrono::steady_clock::now();
            double t_opt = std::chrono::duration<double>(t2 - t1).count();
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Optimization time: %f", t_opt);
            selectBestTraj(best_traj);
            refineTraj(best_traj, time_inc);
            local_data_.position_traj_ = best_traj;
            global_data_.setLocalTraj(best_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);
        }
    }
    updateTrajInfo();
    return true;
}

void PlannerEstimator::selectBestTraj(NonUniformBspline& traj)
{
    std::vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
    sort(trajs.begin(), trajs.end(),
        [&](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
    traj = trajs[0];
}

bool PlannerEstimator::OutSidePlannerProcess()
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "OutSidePlannerProcess");
}

bool PlannerEstimator::ExplorePlannerProcess()
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "ExplorePlannerProcess");
    plan_data_.clearTopoPaths();
    std::vector<Eigen::Vector3d> points;
    for(auto &&pt : plan_data_.global_waypoints_)
    {
        points.push_back(pt);
    }
    if(points.size() == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "No global waypoints");
    }
    points.insert(points.begin(), start_pt_);
    std::vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ExplorePlannerProcess");
    for(int i = 0;i < points.size() - 1; i++)
    {
        inter_points.push_back(points[i]);
        double dist = (points[i] - points[i + 1]).norm(); // 欧式距离
        if(dist > dist_thresh)
        {
            int id_num = floor(dist / dist_thresh) + 1;
            for(int j = 1;j < id_num;j++)
            {
                Eigen::Vector3d inter_point = points[i] + (points[i + 1] - points[i]) * (double)j / (double)id_num;//线性插值TODO
                inter_points.push_back(inter_point);
            }
        }
    }
    inter_points.push_back(points[points.size() - 1]);//最后一个点
    if(inter_points.size() == 2)
    {
        Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) / 2;
        inter_points.insert(inter_points.begin() + 1, mid);
    }

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(pt_num, 3);
    for(int i = 0;i < pt_num;i++)
    {
        pos.row(i) = inter_points[i];
    }
    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for(int i = 0;i < pt_num - 1;i++)
    {
        time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
    }
    time(0) *= 2.0;
    time(0) = max(1.0, time(0));
    time(time.rows() - 1) *= 2.0;
    time(time.rows() - 1) = max(1.0, time(time.rows() - 1));
    PolynomialTraj gl_traj = PolynomialTraj::minSnapTraj(pos, zero, zero, zero, zero, time);
    auto time_now = std::chrono::steady_clock::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    double dt, duration;
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "ExplorePlannerProcess");
    Eigen::MatrixXd ctrl_pts = reparamLocalTraj(0.0, dt, duration);
    NonUniformBspline bspline(ctrl_pts, 3, dt);

    global_data_.setLocalTraj(bspline, 0.0, duration, 0.0); 
    local_data_.position_traj_   = bspline;
    local_data_.start_time_      = time_now;
    updateTrajInfo();

    return true;
}   

int PlannerEstimator::planGlobalTraj()
{
    if(traj_options == TRAJ_OPTIONS::STRAIGHT)
    {
        global_start_time = std::chrono::steady_clock::now();
        return 1;//不考虑
    }
    std::deque<Eigen::Vector3d> local_path = local_way_points_;
    std::deque<Eigen::Vector3d> inter_points;//插值点
    const double dist_thresh = 4.0;
    for(int i = 0;i < local_path.size() - 1; i++)
    {
        inter_points.push_back(local_path[i]);
        double dist = (local_path[i] - local_path[i + 1]).norm();
        if(dist > dist_thresh)
        {
            int id_num = floor(dist / dist_thresh) + 1;
            for(int j = 1;j < id_num;j++)
            {
                Eigen::Vector3d inter_point = local_path[i] + (local_path[i + 1] - local_path[i]) * (double)j / (double)id_num;//线性插值TODO
                inter_points.push_back(inter_point);
            }
        }
    }
    inter_points.push_back(local_path[local_path.size() - 1]);
    if(inter_points.size() == 2)
    {
        Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) / 2;
        inter_points.insert(inter_points.begin() + 1, mid);
    }
    for(int i = 0;i < local_path.size() - 1; i++)
    {
        inter_points.push_back(local_path[i]);
        double dist = (local_path[i] - local_path[i + 1]).norm(); // 欧式距离
        if(dist > dist_thresh)
        {
            int id_num = floor(dist / dist_thresh) + 1;
            for(int j = 1;j < id_num;j++)
            {
                Eigen::Vector3d inter_point = local_path[i] + (local_path[i + 1] - local_path[i]) * (double)j / (double)id_num;//线性插值TODO
                inter_points.push_back(inter_point);
            }
        }
    }
    inter_points.push_back(local_path[local_path.size() - 1]);//最后一个点
    if(inter_points.size() == 2)
    {
        Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) / 2;
        inter_points.insert(inter_points.begin() + 1, mid);
    }

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(pt_num, 3);
    for(int i = 0;i < pt_num;i++)
    {
        pos.row(i) = inter_points[i];
    }
    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for(int i = 0;i < pt_num - 1;i++)
    {
        time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
    }
    time(0) *= 2.0;
    time(0) = max(1.0, time(0));
    time(time.rows() - 1) *= 2.0;
    time(time.rows() - 1) = max(1.0, time(time.rows() - 1));
    PolynomialTraj gl_traj = PolynomialTraj::minSnapTraj(pos, zero, zero, zero, zero, time);
    auto time_now = std::chrono::steady_clock::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    double dt, duration;
    Eigen::MatrixXd ctrl_pts = reparamLocalTraj(0.0, dt, duration);
    NonUniformBspline bspline(ctrl_pts, 3, dt);

    global_data_.setLocalTraj(bspline, 0.0, duration, 0.0); 
    local_data_.position_traj_   = bspline;
    local_data_.start_time_      = time_now;
    updateTrajInfo();

    return true;
}

void PlannerEstimator::visualization_paths(std::vector<std::vector<Eigen::Vector3d > >& paths)
{
    visualization_->drawTopoPathsPhase2(paths, 0.1);
}

void PlannerEstimator::publishTwist(const Eigen::Vector3d& target)
{
    
}

void PlannerEstimator::executeStraight()
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "executeStraight");

    Eigen::Vector3d target = global_way_points_.front();
    if(ReachTarget(target))
    {
        global_way_points_.pop_front();
        if(global_way_points_.empty())
        {
            ChangeGlobalFSMState(WAIT_TARGET, "executeStraight");
            return;
        }
        else
            target = global_way_points_.front();
    }
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = planner_node_->now();

    Eigen::Vector2d target_pos_2d(target(0), target(1));
    Eigen::Vector2d cur_pos_2d(current_pos(0), current_pos(1));
    Eigen::Vector2d dir = target_pos_2d - cur_pos_2d;
    Eigen::Vector2d x_pos{1,0};
    double norm = dir.norm();

    double roll, pitch, yaw;
    Eigen::Quaterniond quat(current_quat.w(), current_quat.x(), current_quat.y(), current_quat.z()); //maybe TF
    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    yaw = euler(2);

    //
    //yaw = yaw + (0.5 * M_PI);
    //if(yaw > M_PI)
    //    yaw -= 2 * M_PI;
    //
    double dot = (dir.x() * x_pos.x() + dir.y() * x_pos.y());
    double cross = (dir.x() * x_pos.y() - dir.y() * x_pos.x());
    double angle = atan2(cross, dot);
    double delta_angle = yaw - angle;
    if(delta_angle > M_PI)
        delta_angle -= 2 * M_PI;
    if(delta_angle < -M_PI)
        delta_angle += 2 * M_PI;
    bool use_final_angle = false;
    double final_angle = 0.0;
    Eigen::Vector2d e_pos2d{cos(yaw), sin(yaw)};
    double cos_angle = dir.dot(e_pos2d) / (e_pos2d.norm() * dir.norm());
    if(delta_angle > 5.0 / 180 * M_PI)
    {
        final_angle = 50 / 180 * M_PI;
        use_final_angle = true;
    }
    else if(delta_angle < -5.0 / 180 * M_PI)
    {
        final_angle = -5.0 / 180 * M_PI;
        use_final_angle = true;
    }
    Eigen::Vector3d final_twist = Eigen::Vector3d(0.0, 0.0, 0.0);
    if(!use_final_angle)
    {
        double final_speed = 0;
        if(norm > 0.05)
            final_speed = 2.0;
        else
            final_speed = 0.0;
        final_twist = Eigen::Vector3d(final_speed, 0.0, 0.0);
    }

    twist_msg.twist.linear.x = final_twist(0);
    twist_msg.twist.linear.y = final_twist(1);
    twist_msg.twist.linear.z = final_twist(2);
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = final_angle;
    //final
    planner_node_->publishTwist(twist_msg);
}

bool PlannerEstimator::checkTrajCollision(double& dist)
{
    auto now = std::chrono::steady_clock::now();
    double t_now = std::chrono::duration<double>(now - local_data_.start_time_).count();
    double tm, tmp;
    local_data_.position_traj_.getTimeSpan(tm, tmp);
    Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);
    double radi = 0.0;
    Eigen::Vector3d fut_pt;
    double fut_t = 0.02;
    while(radi < 6.0 && t_now + fut_t < local_data_.duration_)
    {
        fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);
        double distance = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
        if(distance < 0.1)
        {
            dist = radi;
            return false;
        }
        radi = (fut_pt - cur_pt).norm();
        fut_t += 0.02;
    }
    return true;
}

void PlannerEstimator::planYaw(const Eigen::Vector3d& start_yaw)
{
    auto t1 = std::chrono::steady_clock::now();
    auto& pos = local_data_.position_traj_;
    double duration = pos.getTimeSum();

    double dt_yaw = 0.3;
    int seg_num = ceil(duration / dt_yaw);
    dt_yaw = duration / seg_num;
    const double forward_t = 2.0; //前向时间
    double last_yaw = start_yaw(0);

    std::vector<Eigen::Vector3d> waypts;
    std::vector<int> waypt_idx;

    for(int i = 0;i < seg_num;i++)
    {
        double tc = i * dt_yaw;
        Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
        double tf = min(duration, tc + forward_t);
        Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
        Eigen::Vector3d pd = pf - pc;

        Eigen::Vector3d waypt;
        if(pd.norm() > 1e-6)
        {
            waypt(0) = atan2(pd(1), pd(0));
            waypt(1) = waypt(2) = 0.0;
            calcNextYaw(last_yaw, waypt(0));
        }
        else
        {
            waypt = waypts.back();
        }
        waypts.push_back(waypt);
        waypt_idx.push_back(i);
    }

    Eigen::MatrixXd yaw(seg_num + 3, 1);
    yaw.setZero();

    Eigen::Matrix3d state2pts;
    state2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
        dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
    yaw.block(0, 0, 3, 1) = state2pts * start_yaw;
    Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
    Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0.0, 0.0);

    calcNextYaw(last_yaw, end_yaw(0));
    yaw.block(seg_num, 0, 3, 1) = state2pts * end_yaw;

    bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
    int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINT;

    yaw = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);
    
    local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
    local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
    local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

    std::vector<double> path_yaw;
    for(int i = 0;i < waypts.size();i++)
    {
        path_yaw.push_back(waypts[i](0));
    }
    plan_data_.path_yaw_ = path_yaw;
    plan_data_.dt_yaw_ = dt_yaw;
    plan_data_.dt_yaw_path_ = dt_yaw;
}

void PlannerEstimator::optimizeTopoBspline(double start_t, double duration,
                                             vector<Eigen::Vector3d> guide_path, int traj_id) 
{
    std::chrono::steady_clock::time_point t1;
    double tm1, tm2, tm3;
    t1 = std::chrono::steady_clock::now();

    int seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
    Eigen::MatrixXd ctrl_pts;
    double dt;
    ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);

    std::vector<Eigen::Vector3d> guide_pt;
    guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2);
    guide_pt.pop_back();
    guide_pt.pop_back();

    guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);
    tm1 = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
    t1 = std::chrono::steady_clock::now();
#ifdef USE_3D
    bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
    Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);
    plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts1, 3, dt);

    tm2 = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
    t1 = std::chrono::steady_clock::now();

    Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(ctrl_pts, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);
    plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts2, 3, dt);
#else 
    double z = guide_pt[0](2);
    std::vector<Eigen::Vector2d> guide_pt2d;
    Eigen::MatrixXd ctrl_pts2d(ctrl_pts.rows(), 2);
    for(int i = 0;i < ctrl_pts.rows();i++)
    {
        ctrl_pts2d(i, 0) = ctrl_pts(i, 0);
        ctrl_pts2d(i, 1) = ctrl_pts(i, 1);
    }
    for(auto &&pt : guide_pt)
    {
        guide_pt2d.push_back(Eigen::Vector2d(pt(0), pt(1)));
    }
    bspline_optimizers_2d_[traj_id]->setGuidePath(guide_pt2d);
    bspline_optimizers_2d_[traj_id]->setZ(0.0);
    Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_2d_[traj_id]->BsplineOptimizeTraj(ctrl_pts2d, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);
    Eigen::MatrixXd opt_ctrl_pts3d(opt_ctrl_pts1.rows(), 3);
    for(int i = 0;i < opt_ctrl_pts1.rows();i++)
    {
        opt_ctrl_pts3d(i, 0) = opt_ctrl_pts1(i, 0);
        opt_ctrl_pts3d(i, 1) = opt_ctrl_pts1(i, 1);
        opt_ctrl_pts3d(i, 2) = 0.0;
    }
    plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts3d, 3, dt);
    tm2 = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
    t1 = std::chrono::steady_clock::now();
    Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_2d_[traj_id]->BsplineOptimizeTraj(ctrl_pts2d, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);
    Eigen::MatrixXd opt_ctrl_pts3d2(opt_ctrl_pts2.rows(), 3);
    for(int i = 0;i < opt_ctrl_pts2.rows();i++)
    {
        opt_ctrl_pts3d2(i, 0) = opt_ctrl_pts2(i, 0);
        opt_ctrl_pts3d2(i, 1) = opt_ctrl_pts2(i, 1);
        opt_ctrl_pts3d2(i, 2) = 0.0;
    }
    plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts3d2, 3, dt);
#endif
    tm3 = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Optimization time: %f, %f, %f", tm1, tm2, tm3);
}

void PlannerEstimator::calcNextYaw(const double& last_yaw,double& yaw)
{
    double round_last = last_yaw;
    while(round_last < -M_PI)
    {
        round_last += 2 * M_PI;
    }
    while(round_last > M_PI)
    {
        round_last -= 2 * M_PI;
    }
    double diff = yaw - round_last;
    if(fabs(diff) <= M_PI)
    {
        yaw = last_yaw + diff;
    }
    else if(diff > M_PI)
    {
        yaw = last_yaw + diff - 2 * M_PI;
    }
    else if(diff < -M_PI)
    {
        yaw = last_yaw + diff + 2 * M_PI;
    }
}

void PlannerEstimator::publishState()
{
    std_msgs::msg::Int32 state_msg;
    state_msg.data = fsm_state;
    fsm_pub->publish(state_msg);
}

bool PlannerEstimator::checkStartTraj(const Eigen::MatrixXd& ctrl_pts, const Eigen::Vector3d& start_pos)
{
    Eigen::Vector3d start_pt = ctrl_pts.row(0);
    double dist = (start_pt - start_pos).norm();
    if(dist < 0.5)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool PlannerEstimator::checkEndTraj(const Eigen::MatrixXd& ctrl_pts, const Eigen::Vector3d& end_pos)
{
    Eigen::Vector3d end_pt = ctrl_pts.row(ctrl_pts.rows() - 1);
    double dist = (end_pt - end_pos).norm();
    if(dist < 0.5)
    {
        return true;
    }
    else
    {
        return false;
    }
}