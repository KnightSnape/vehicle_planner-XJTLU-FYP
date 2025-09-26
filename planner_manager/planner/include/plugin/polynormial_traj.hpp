#pragma once

#include<Eigen/Eigen>
#include<vector>

using std::vector;

class PolynomialTraj
{
    public:
        PolynomialTraj(){}
        ~PolynomialTraj(){}

        void reset();
        void addSegment(std::vector<double> cx, std::vector<double> cy, std::vector<double> cz, double time);
        void init();
        Eigen::Vector3d evaluate(double t);
        Eigen::Vector3d evaluateVel(double t);
        Eigen::Vector3d evaluateAcc(double t);
        double getTimeSum();
        vector<Eigen::Vector3d> getTraj();
        double getLength();
        double getMeanVel();
        double getAccCost();
        double getJerk();
        void getMeanAndMaxVel(double& mean_v,double& max_v);
        void getMeanAndMaxAcc(double& mean_a,double& max_a);

        static PolynomialTraj minSnapTraj(const Eigen::MatrixXd& Pos, const Eigen::Vector3d& start_vel,
                                   const Eigen::Vector3d& end_vel, const Eigen::Vector3d& start_acc,
                                   const Eigen::Vector3d& end_acc, const Eigen::VectorXd& Time);
        
        PolynomialTraj fastLine4deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc,
                            double max_jerk);

        PolynomialTraj fastLine3deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc);

        
        

    private:
        vector<double> times;
        vector<vector<double>> cxs;
        vector<vector<double>> cys;
        vector<vector<double>> czs;

        double time_sum;
        int num_seg;

        vector<Eigen::Vector3d> traj_vec3d;
        double length;
};