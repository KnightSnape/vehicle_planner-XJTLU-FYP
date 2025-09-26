#pragma once

#include<Eigen/Eigen>
#include<algorithm>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<iostream>
#include<list>
#include<chrono>

class PolynomialPrediction;
typedef std::shared_ptr<std::vector<PolynomialPrediction> > ObjPrediction;
typedef std::shared_ptr<std::vector<Eigen::Vector3d> > ObjScale;

class PolynomialPrediction
{
    public:
        PolynomialPrediction(/* args */) {
        }
        ~PolynomialPrediction() {
        }

        void setPolynomial(const std::vector<Eigen::Matrix<double, 6, 1>>& pls) {
            polys = pls;
        }

        void setTime(double t1,double t2)
        {
            this->t1 = t1;
            this->t2 = t2;
        }

        bool valid(){
            return polys.size() == 2;
        }

        Eigen::Vector3d evaluate(double t) {
            Eigen::Matrix<double, 6, 1> tv;
            tv << 1.0, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

            Eigen::Vector3d pt;
            pt(0) = tv.dot(polys[0]), pt(1) = tv.dot(polys[1]), pt(2) = tv.dot(polys[2]);

            return pt;
        }

        Eigen::Vector3d evaluateConstVel(double t) {
            Eigen::Matrix<double, 2, 1> tv;
            tv << 1.0, pow(t, 1);

            Eigen::Vector3d pt;
            pt(0) = tv.dot(polys[0].head(2)), pt(1) = tv.dot(polys[1].head(2)), pt(2) = tv.dot(polys[2].head(2));

            return pt;
        }

    private:
        std::vector<Eigen::Matrix<double, 6, 1>> polys;
        double t1, t2;//start, end
};

class ObjHistory
{
    public:
        static int skip_num_;
        static int queue_size_;
        static std::chrono::steady_clock::time_point global_start_time;

        ObjHistory(){}
        ~ObjHistory(){}

        void init(int id);
        //void handlerPose(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
        void clear(){history_.clear();}
        void getHistory(std::list<Eigen::Vector4d>& his){ his = history_; }

    private:
        std::list<Eigen::Vector4d> history_;
        int skip_;
        int obj_idx_;
        Eigen::Vector3d scale_;
};