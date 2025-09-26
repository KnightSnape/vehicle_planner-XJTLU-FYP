#pragma once

#include<utils/utility.hpp>
#include<plugin/astar.hpp>
#include<plugin/topo_prm.hpp>
#include<plugin/planning_visualization.hpp>
#include<plugin/polynormial_traj.hpp>
#include<plugin/bspline_optimizer.hpp>
#include<plugin/bspline_optimizer_2d.hpp>
#include<plugin/non_uniform_bspline.hpp>
#include<plugin/sdfMap.hpp>
#include<room_graph.hpp>
#include<thread>
#include<mutex>
#include<condition_variable>

class PlannerNode;

using namespace planner;

struct PlanParameters
{
    double max_vel_, max_acc_, max_jerk_;  // physical limits
    double local_traj_len_;                // local replanning trajectory length
    double ctrl_pt_dist;                   // distance between adjacient B-spline
                                            // control points
    double clearance_;
    int dynamic_;
    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
};

class GlobalTrajData
{
    public:
        PolynomialTraj global_traj_;
        std::vector<NonUniformBspline> local_traj_;

        double global_duration_;
        std::chrono::steady_clock::time_point global_start_time_;
        double local_start_time_;
        double local_end_time_;
        double time_increase_;
        double last_time_inc_;

        GlobalTrajData(){}
        ~GlobalTrajData(){}

        bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

        void setGlobalTraj(const PolynomialTraj& traj, const std::chrono::steady_clock::time_point& start_time)
        {
            global_traj_ = traj;
            global_traj_.init();
            global_duration_ = global_traj_.getTimeSum();
            global_start_time_ = start_time;

            local_traj_.clear();
            local_start_time_ = -1;
            local_end_time_ = -1;
            time_increase_ = 0.0;
            last_time_inc_ = 0.0;
        }

        void setLocalTraj(NonUniformBspline traj, double local_ts, double local_te, double time_inc)
        {
            local_traj_.resize(3);
            local_traj_[0] = traj;
            local_traj_[1] = local_traj_[0].getDerivative();
            local_traj_[2] = local_traj_[1].getDerivative();
            local_start_time_ = local_ts;
            local_end_time_ = local_te;
            global_duration_ += time_inc;
            time_increase_ += time_inc;
            last_time_inc_ = time_inc;
        }

        Eigen::Vector3d getPosition(double t)
        {
            if (t >= -1e-3 && t <= local_start_time_) 
            {
                return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
            } 
            else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) 
            {
                return global_traj_.evaluate(t - time_increase_);
            } 
            else 
            {
                double tm, tmp;
                local_traj_[0].getTimeSpan(tm, tmp);
                return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
            }
        }

        Eigen::Vector3d getVelocity(double t)
        {
            if (t >= -1e-3 && t <= local_start_time_) 
            {
                return global_traj_.evaluateVel(t);
            } 
            else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) 
            {
                return global_traj_.evaluateVel(t - time_increase_);
            } 
            else 
            {
                double tm, tmp;
                local_traj_[0].getTimeSpan(tm, tmp);
                return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
            }
        }

        Eigen::Vector3d getAcceleration(double t)
        {
            if (t >= -1e-3 && t <= local_start_time_) 
            {
                return global_traj_.evaluateAcc(t);
            } 
            else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) 
            {
                return global_traj_.evaluateAcc(t - time_increase_);
            } 
            else 
            {
                double tm, tmp;
                local_traj_[0].getTimeSpan(tm, tmp);
                return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
            }
        }

        void getTrajByRadius(const double& start_t, const double& des_radius, const double& dist_pt,
                            vector<Eigen::Vector3d>& point_set, vector<Eigen::Vector3d>& start_end_derivative,
                            double& dt, double& seg_duration) {
            double seg_length = 0.0;  // length of the truncated segment
            double seg_time = 0.0;    // duration of the truncated segment
            double radius = 0.0;      // distance to the first point of the segment

            double delt = 0.2;
            Eigen::Vector3d first_pt = getPosition(start_t);  // first point of the segment
            Eigen::Vector3d prev_pt = first_pt;               // previous point
            Eigen::Vector3d cur_pt;                           // current point

            // go forward until the traj exceed radius or global time

            while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3) {
                seg_time += delt;
                seg_time = min(seg_time, global_duration_ - start_t);

                cur_pt = getPosition(start_t + seg_time);
                seg_length += (cur_pt - prev_pt).norm();
                prev_pt = cur_pt;
                radius = (cur_pt - first_pt).norm();
            }

            // get parameterization dt by desired density of points
            int seg_num = floor(seg_length / dist_pt);
            // get outputs

            seg_duration = seg_time;  // duration of the truncated segment
            dt = seg_time / seg_num;  // time difference between to points

            for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt) {
                cur_pt = getPosition(start_t + tp);
                point_set.push_back(cur_pt);
            }
            start_end_derivative.push_back(getVelocity(start_t));
            start_end_derivative.push_back(getVelocity(start_t + seg_time));
            start_end_derivative.push_back(getAcceleration(start_t));
            start_end_derivative.push_back(getAcceleration(start_t + seg_time));         

        }


        void getTrajByDuration(double start_t, double duration, int seg_num,
                         vector<Eigen::Vector3d>& point_set,
                         vector<Eigen::Vector3d>& start_end_derivative, double& dt)
        {
            dt = duration / seg_num;
            Eigen::Vector3d cur_pt;
            for (double tp = 0.0; tp <= duration + 1e-4; tp += dt) {
                cur_pt = getPosition(start_t + tp);
                point_set.push_back(cur_pt);
            }

            start_end_derivative.push_back(getVelocity(start_t));
            start_end_derivative.push_back(getVelocity(start_t + duration));
            start_end_derivative.push_back(getAcceleration(start_t));
            start_end_derivative.push_back(getAcceleration(start_t + duration));
        }

};

struct LocalTrajData {
  /* info of generated traj */

  int traj_id_;
  double duration_;
  std::chrono::steady_clock::time_point start_time_;
  Eigen::Vector3d start_pos_;
  NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_, yaw_traj_, yawdot_traj_,
      yawdotdot_traj_;
};

class MidPlanData
{
    public:
        MidPlanData(){}
        ~MidPlanData(){}

        std::deque<Eigen::Vector3d> global_waypoints_;
        NonUniformBspline initial_local_segment_;
        std::vector<Eigen::Vector3d> local_start_end_derivative_;

        // kinodynamic path
        std::vector<Eigen::Vector3d> kino_path_;

        // topological paths
        list<GraphNode::Ptr> topo_graph_;
        std::vector<std::vector<Eigen::Vector3d>> topo_paths_;
        std::vector<std::vector<Eigen::Vector3d>> topo_filtered_paths_;
        std::vector<std::vector<Eigen::Vector3d>> topo_select_paths_;

        // multiple topological trajectories
        std::vector<NonUniformBspline> topo_traj_pos1_;
        std::vector<NonUniformBspline> topo_traj_pos2_;
        std::vector<NonUniformBspline> refines_;

        // visibility constraint
        std::vector<Eigen::Vector3d> block_pts_;
        Eigen::MatrixXd ctrl_pts_;

        // heading planning
        std::vector<double> path_yaw_;
        double dt_yaw_;
        double dt_yaw_path_;
        void clearTopoPaths() {
            topo_traj_pos1_.clear();
            topo_traj_pos2_.clear();
            topo_graph_.clear();
            topo_paths_.clear();
            topo_filtered_paths_.clear();
            topo_select_paths_.clear();
        }

        
        void addTopoPaths(list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector3d>>& paths,
                            vector<vector<Eigen::Vector3d>>& filtered_paths,
                            vector<vector<Eigen::Vector3d>>& selected_paths) {
            topo_graph_ = graph;
            topo_paths_ = paths;
            topo_filtered_paths_ = filtered_paths;
            topo_select_paths_ = selected_paths;
        }
        
};

class PlannerEstimator
{
    public:
        enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };
        typedef std::shared_ptr<PlannerEstimator> Ptr;
        PlannerEstimator(){};
        PlannerEstimator(PlannerNode* node);
        ~PlannerEstimator();

        void paraminit(std::string package_path);
        void initModules(std::string package_path);
        void resetGlobalWayPoints();

        void printGlobalType(GLOBAL_TYPE type);
        void setOdomstate(bool state);
        bool ReachTarget(const Eigen::Vector3d& target_pos_now);
        void publishTwist(const Eigen::Vector3d& target);
        void publishState();

        void updateCurrentPos(const Eigen::Vector3d& pos);
        void updateCurrentVel(const Eigen::Vector3d& vel);
        void updateCurrentQuat(const Eigen::Quaterniond& quat);

        void setSensorState(int state);
        void CheckSensorState();

        void OnWayPointCallback(const nav_msgs::msg::Path& msg);
        void HandlerGoal();

        EDTEnvironment::Ptr getEDTEnvironment();
        void ChangeGlobalFSMState(GLOBAL_FSM_STATE fsm_state_update, std::string from);
        void printFSMExecState();
        void executeStraight();

        void PlannerProcess();
        void CollisionCheckProcess();
        bool callTopologicalTraj(int step);
        bool RoomPlannerProcess();
        bool OutSidePlannerProcess();
        bool ExplorePlannerProcess();


        bool checkStartTraj(const Eigen::MatrixXd& ctrl_pts, const Eigen::Vector3d& start_pos);
        bool checkEndTraj(const Eigen::MatrixXd& ctrl_pts, const Eigen::Vector3d& end_pos);
        int planGlobalTraj();
        bool topoReplan();
        void selectBestTraj(NonUniformBspline& traj);
        void updateTrajInfo();
        void refineTraj(NonUniformBspline& best_traj, double& time_inc);

        Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);
        Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double& dt);
        void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                      double& time_inc);
        void findCollisionRange(std::vector<Eigen::Vector3d>& colli_start,
                                std::vector<Eigen::Vector3d>& colli_end,
                                std::vector<Eigen::Vector3d>& start_pts,
                                std::vector<Eigen::Vector3d>& end_pts);
        bool checkTrajCollision(double& distance);
        void planYaw(const Eigen::Vector3d& start_yaw);
        void calcNextYaw(const double& last_yaw,double& yaw);
        void visualization_paths(std::vector<std::vector<Eigen::Vector3d > >& paths);
        void optimizeTopoBspline(double start_t, double duration,
                                vector<Eigen::Vector3d> guide_path, int traj_id);

        nav_msgs::msg::Odometry getOdomState();
        
        SDFMap::Ptr sdf_map_;
        GLOBAL_FSM_STATE fsm_state;
        TARGET_TYPE target_type;
        GLOBAL_TYPE global_type;

    private:
        int cycle_cnt;
        PlannerNode* planner_node_;

        Eigen::Vector3d current_pos, current_vel;
        Eigen::Quaterniond current_quat;

        Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;
        Eigen::Vector3d target_point_, end_vel;
        Eigen::Vector3d temp_target_point_;

        rclcpp::TimerBase::SharedPtr machine_changer;
        rclcpp::TimerBase::SharedPtr collision_checker;

        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr new_pub;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_pub;

        std::shared_ptr<RoomGraph> room_graph_;

        std::deque<Eigen::Vector3d> global_way_points_;
        std::deque<Eigen::Vector3d> local_way_points_;

        PlanParameters pp_;
        LocalTrajData local_data_;
        GlobalTrajData global_data_;
        MidPlanData plan_data_;
        EDTEnvironment::Ptr edt_environment_;

        PlanningVisualization::Ptr visualization_;

        std::vector<BsplineOptimizer::Ptr> bspline_optimizers_;
        std::vector<BsplineOptimizer2d::Ptr> bspline_optimizers_2d_;

        std::unique_ptr<Astar> geo_path_finder_;
        std::unique_ptr<TopologyPRM> topo_prm_;

        double waypoints_[50][3];
        int current_wp_;
        nav_msgs::msg::Path way_point;

        TRAJ_OPTIONS traj_options;
        int sensor_state;

        int waypoint_num_;

        bool have_odom_;
        bool have_target_;
        bool trigger_;
        bool collide_;

        double replan_distance_threshold_, replan_time_threshold_;
        bool use_geometric_path, use_topo_path,use_optimization, use_active_perception_;

        builtin_interfaces::msg::Time toROSTime(const std::chrono::steady_clock::time_point& tp) {
            builtin_interfaces::msg::Time ros_time;
            
            // 获取当前时间距离 UNIX 时间（自 1970-01-01 起的时间）的 duration
            auto now = std::chrono::time_point_cast<std::chrono::nanoseconds>(tp);
            auto epoch = now.time_since_epoch();
            
            // 获取秒数和纳秒数
            ros_time.sec = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();
            ros_time.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count() % 1000000000;

            return ros_time;
        }

        std::chrono::steady_clock::time_point global_start_time;

};

typedef std::shared_ptr<PlannerEstimator> PlannerEstimatorPtr;
typedef std::shared_ptr<PlannerEstimator const> PlannerEstimatorConstPtr;