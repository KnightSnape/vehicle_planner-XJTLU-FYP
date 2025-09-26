#pragma once

#include<Eigen/Eigen>
#include<algorithm>
#include<iostream>

using namespace std;

namespace planner
{

class NonUniformBspline
{
    private:
        Eigen::MatrixXd control_points_;//control points
        int p_,n_,m_;//degree, number of control points, number of knots
        Eigen::VectorXd u_;//knots vector
        double interval_;//interval of the knots

        Eigen::MatrixXd getDerivativeControlPoints();//get the derivative of the control points
        double limit_vel_,limit_acc_,limit_ratio_;

    public:
        NonUniformBspline(){};
        NonUniformBspline(const Eigen::MatrixXd& points,const int& order,const double& interval);
        ~NonUniformBspline();

        void setUniformBspline(const Eigen::MatrixXd& points,const int& order,const double& interval);

        void setKnot(const Eigen::VectorXd& knots);
        Eigen::VectorXd getKnot();
        Eigen::MatrixXd getControlPoints();
        double getInterval();
        void getTimeSpan(double& um,double& um_p);
        pair<Eigen::VectorXd,Eigen::VectorXd> getHeadTailPts();

        Eigen::VectorXd evaluateDeBoor(const double& u) const;//evaluate the point at u
        Eigen::VectorXd evaluateDeBoorT(const double& u) const;//evaluate the tangent at u

        NonUniformBspline getDerivative();//get the derivative of the bspline
        static void parameterizeToBspline(const double& ts,
                                          const vector<Eigen::Vector3d>& point_set,
                                          const vector<Eigen::Vector3d>& start_end_derivative,
                                          Eigen::MatrixXd& ctrl_pts);

        void setPhysicalLimits(const double& vel, const double& acc);
        bool   checkFeasibility(bool show = false);
        double checkRatio();
        void   lengthenTime(const double& ratio);
        bool   reallocateTime(bool show = false);

        /* for performance evaluation */

        double getTimeSum();
        double getLength(const double& res = 0.01);
        double getJerk();
        void   getMeanAndMaxVel(double& mean_v, double& max_v);
        void   getMeanAndMaxAcc(double& mean_a, double& max_a);

        void recomputeInit();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}