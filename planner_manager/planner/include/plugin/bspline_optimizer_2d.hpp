#pragma once

#include<Eigen/Eigen>
#include<edt_environment.h>
#include<memory>

class BsplineOptimizer2d
{
    public:
        static const int SMOOTHNESS;
        static const int DISTANCE;
        static const int FEASIBILITY;
        static const int ENDPOINT;
        static const int GUIDE;
        static const int WAYPOINT;

        static const int GUIDE_PHASE;
        static const int NORMAL_PHASE;

        BsplineOptimizer2d() {}
        ~BsplineOptimizer2d() {}
        void setEnvironment(const EDTEnvironment::Ptr& env);
        void setParams(std::string config_path);
        Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts, const int& cost_function, int max_num_id, int max_time_id);

        void setControlPoints(const Eigen::MatrixXd& points);
        void setBsplineInterval(const double& ts);
        void setCostFunction(const int& cost_function);
        void setTerminateCond(const int& max_num_id, const int max_time_id);

        void setZ(const double& z);
        void setGuidePath(const std::vector<Eigen::Vector2d>& guide_pts);
        void setWaypoints(const std::vector<Eigen::Vector2d>& waypts,
                          const std::vector<int>& waypt_idx);

        void optimize();
        Eigen::MatrixXd getControlPoints();
        std::vector<Eigen::Vector2d> matrixToVector(const Eigen::MatrixXd& ctrl_pts);
    private:
        EDTEnvironment::Ptr edt_environment_;

        Eigen::MatrixXd control_points;      //B样条控制点
        double          bspline_interval;    //B样条间隔
        double          z_;                  //高度
        Eigen::Vector2d end_pt;              //终点
        int dim_;                            //维度
        std::vector<Eigen::Vector2d>    guide_pts_;  //几何引导路径点 N-6
        std::vector<Eigen::Vector2d>    waypoints_;  //路径点 N-6
        std::vector<int>                waypt_idx_;  //路径点索引
        int max_num_id_, max_time_id_; //最大迭代次数和最大时间优化次数

        int cost_function_;            //成本函数
        bool dynamic_;
        double start_time_;

        int order_; //阶数
        double lambda1_;                // 冲击平滑度权重
        double lambda2_;                // 距离权重
        double lambda3_;                // 可行性权重
        double lambda4_;                // 终点重量
        double lambda5_;                // 指导成本重量
        double lambda6_;                // 可见性成本权重
        double lambda7_;                // 航点成本权重
        double lambda8_;                // 加速平滑度

        double dist0_;
        double max_vel, max_acc;
        double visib_min_;
        double wnl_;
        double dlmin_;

        int algorithm1_;                //优化算法1：
        int algorithm2_;                //优化算法2：

        int max_iteration_num_[4];
        double max_iteration_time_[4];

        std::vector<Eigen::Vector2d> g_q_;//引导路径点
        std::vector<Eigen::Vector2d> g_smoothness_;//平滑度
        std::vector<Eigen::Vector2d> g_distance_;//距离
        std::vector<Eigen::Vector2d> g_feasibility_;//可行性
        std::vector<Eigen::Vector2d> g_endpoint_;//终点
        std::vector<Eigen::Vector2d> g_guide_;//引导
        std::vector<Eigen::Vector2d> g_waypoints_;//航点

        int                 variable_num_;   // optimization variables
        int                 iter_num_;       // iteration of the solver
        std::vector<double> best_variable_;  //
        double              min_cost_;       //

        static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
        void combineCost(const std::vector<double>& x,std::vector<double>& grad, double& cost);

        void calcSmoothnessCost(const std::vector<Eigen::Vector2d>& q,double& cost,std::vector<Eigen::Vector2d>& gradient);
        void calcDistanceCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient);
        void calcFeasibilityCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient);
        void calcEndpointCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient);
        void calcGuideCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient);
        void calcVisibilityCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient);
        void calcWaypointsCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient);
        void calcViewCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient);
        bool isQuadratic();
    public:
        std::vector<double> vec_cost_;
        std::vector<double> vec_time_;

        std::chrono::steady_clock::time_point time_start_;

        void getCostCurve(std::vector<double>& cost, std::vector<double>& time) {
            cost = vec_cost_;
            time = vec_time_;
        }

        typedef std::unique_ptr<BsplineOptimizer2d> Ptr;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};