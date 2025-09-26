#include<ament_index_cpp/get_package_share_directory.hpp>
#include<plugin/non_uniform_bspline.hpp>
#include<plugin/bspline_optimizer.hpp>
#include<plugin/edt_environment.h>
#include<plugin/sdfMap.hpp>
#include<rclcpp/rclcpp.hpp>
#include<planner/msg/bspline.hpp>
#include<plugin/polynormial_traj.hpp>

using namespace planner;


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

class TestBspline : public rclcpp::Node
{
    public:
        TestBspline():Node("test_bspline")
        {
            std::string package_path_root = ament_index_cpp::get_package_share_directory("planner");
            sdf_map_ = std::make_shared<SDFMap>(package_path_root);

            edt_environment_ = std::make_shared<EDTEnvironment>();
            edt_environment_->setMap(sdf_map_);
            bspline_optimizers_.resize(10);
            for(int i = 0; i < 10; i++)
            {
                bspline_optimizers_[i].reset(new BsplineOptimizer);
                bspline_optimizers_[i]->setParams(package_path_root);
                bspline_optimizers_[i]->setEnvironment(edt_environment_);
            }
            bspline_pub_ = this->create_publisher<planner::msg::Bspline>("/bspline",10);
            std::vector<Eigen::Vector3d> points;
            //需要调整途径路径的位置
            // Eigen::Vector3d p1(0,0,0);
            // Eigen::Vector3d p2(1.5,1,0);
            // Eigen::Vector3d p3(4.8,-0.3,0);
            // Eigen::Vector3d p4(9.5,0.5,0);
            // Eigen::Vector3d p5(11.9,0.3,0);
            // Eigen::Vector3d p6(15.3,0.56,0);
            // Eigen::Vector3d p7(18.6,0.59,0);
            // Eigen::Vector3d p8(22,0.74,0);
            // Eigen::Vector3d p9(26,0.7,0);

            // Eigen::Vector3d p1(0,0,0);
            // Eigen::Vector3d p2(2,-0.3,0);
            // Eigen::Vector3d p3(3,-0.5,0);
            // Eigen::Vector3d p4(4,-0.8,0);
            // Eigen::Vector3d p5(5,-0.5,0);
            // Eigen::Vector3d p6(6,-0.3,0);
            // Eigen::Vector3d p7(7,-0.15,0);
            // Eigen::Vector3d p8(9,0,0);
            // Eigen::Vector3d p9(10,0,0);

            // Eigen::Vector3d p1(0,0,0);
            // Eigen::Vector3d p2(2,-0.3,0);
            // Eigen::Vector3d p3(3,-0.4,0);
            // Eigen::Vector3d p4(4,-3.0,0);
            // Eigen::Vector3d p5(5,-0.6,0);
            // Eigen::Vector3d p6(6,-0.3,0);
            // Eigen::Vector3d p7(7,-0.15,0);
            // Eigen::Vector3d p8(9,0,0);
            // Eigen::Vector3d p9(10,0,0);

            // Eigen::Vector3d p1(0,0,0);
            // Eigen::Vector3d p2(0.5,0,0);
            // Eigen::Vector3d p3(1.25,0.2,0);
            // Eigen::Vector3d p4(3.25,1.1,0);
            // Eigen::Vector3d p5(3.6,1.75,0);
            // Eigen::Vector3d p6(3.75,2.5,0);
            // Eigen::Vector3d p7(3.9,3,0);
            // Eigen::Vector3d p8(3.95,3.5,0);
            // Eigen::Vector3d p9(4,4,0);

            Eigen::Vector3d p1(0,0,0);
            Eigen::Vector3d p2(2,0,0);
            Eigen::Vector3d p3(4,0,0);
            Eigen::Vector3d p4(10,0,0);
            Eigen::Vector3d p5(15,0,0);
            Eigen::Vector3d p6(30,0,0);
            Eigen::Vector3d p7(35,0,0);
            Eigen::Vector3d p8(44,0,0);
            Eigen::Vector3d p9(50,0,0);

            Eigen::MatrixXd control_points(9, 3);
            control_points.row(0) = p1;
            control_points.row(1) = p2;
            control_points.row(2) = p3;
            control_points.row(3) = p4;
            control_points.row(4) = p5;
            control_points.row(5) = p6;
            control_points.row(6) = p7;
            control_points.row(7) = p8;
            control_points.row(8) = p9;
            //time的数量为control_points的数量减1
            Eigen::Vector3d zero(0, 0, 0);
            Eigen::VectorXd time(8);//num - 1
            for(int i = 0;i < 8;i++)
            {
                time(i) = (control_points.row(i + 1) - control_points.row(i)).norm() / 0.5;//max vec
            }
            time(0) *= 2.0;
            time(0) = max(1.0, time(0));
            time(time.rows() - 1) *= 2.0;
            time(time.rows() - 1) = max(1.0, time(time.rows() - 1));
            PolynomialTraj gl_traj = PolynomialTraj::minSnapTraj(control_points, zero, zero, zero, zero, time);
            auto time_now = std::chrono::steady_clock::now();
            global_data_.setGlobalTraj(gl_traj, time_now);
            double dt, duration;
            Eigen::MatrixXd ctrl_pts = reparamLocalTraj(0.0, dt, duration);
            std::cout<<ctrl_pts<<std::endl;
            bspline_.setUniformBspline(ctrl_pts, 3, dt);
            bspline_velocity = bspline_.getDerivative();
            Eigen::Vector3d start_yaw = Eigen::Vector3d::Zero();
            
            planYaw(start_yaw);
            publishBspline();
        }

        void planYaw(const Eigen::Vector3d& start_yaw)
        {
            auto t1 = std::chrono::steady_clock::now();
            auto& pos = bspline_;
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
            Eigen::Vector3d end_v = bspline_velocity.evaluateDeBoorT(duration - 0.1);
            Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0.0, 0.0);

            calcNextYaw(last_yaw, end_yaw(0));
            yaw.block(seg_num, 0, 3, 1) = state2pts * end_yaw;

            bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
            int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINT;

            yaw = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);
            bspline_yaw_.setUniformBspline(yaw, 3, dt_yaw);
        }

        void calcNextYaw(const double& last_yaw,double& yaw)
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

        void publishBspline()
        {
            std::cout<<"Start Publish"<<std::endl;
            planner::msg::Bspline bspline_msg;
            bspline_msg.start_time = rclcpp::Time(0,0);
            bspline_msg.order = 3;
            bspline_msg.traj_id = 0;
            Eigen::MatrixXd pos_pts = bspline_.getControlPoints();
            for(int i = 0;i < pos_pts.rows();i++)
            {
                geometry_msgs::msg::Point pt;
                pt.x = pos_pts(i, 0);
                pt.y = pos_pts(i, 1);
                pt.z = pos_pts(i, 2);
                bspline_msg.pos_pts.push_back(pt);
            }
            Eigen::VectorXd knots = bspline_.getKnot();
            for(int i = 0;i < knots.rows();i++)
            {
                bspline_msg.knots.push_back(knots(i));
            }

            Eigen::MatrixXd yaw_pts = bspline_yaw_.getControlPoints();
            std::cout<<yaw_pts<<std::endl;
            for(int i = 0;i < yaw_pts.rows();i++)
            {
                double yaw = yaw_pts(i, 0);
                bspline_msg.yaw_pts.push_back(yaw);
            }
            bspline_msg.yaw_dt = bspline_yaw_.getInterval();
            bspline_pub_->publish(bspline_msg);
            std::cout<<"Publish Done"<<std::endl;
            exit(0);
        }

        Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration)
        {
            std::vector<Eigen::Vector3d> point_set;
            std::vector<Eigen::Vector3d> start_end_derivative;

            //距离阈值
            global_data_.getTrajByRadius(start_t, 30.0, 0.3, point_set, start_end_derivative, dt, duration);
            Eigen::MatrixXd ctrl_pts;
            NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
            local_start_end_derivative = start_end_derivative; 
            return ctrl_pts;
        }


    private:
        NonUniformBspline bspline_, bspline_yaw_, bspline_velocity;
        SDFMap::Ptr sdf_map_;
        EDTEnvironment::Ptr edt_environment_;
        rclcpp::Publisher<planner::msg::Bspline>::SharedPtr bspline_pub_;
        std::vector<BsplineOptimizer::Ptr> bspline_optimizers_;
        std::vector<Eigen::Vector3d> local_start_end_derivative;
        std::vector<double> path_yaw_;
        GlobalTrajData global_data_;
        double dt_yaw_;
        double dt_yaw_path_;
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TestBspline>());
    rclcpp::shutdown();
    return 0;
}
