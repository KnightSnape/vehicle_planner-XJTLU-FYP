#pragma once

#include<Eigen/Eigen>
#include<sdfMap.hpp>
#include<iostream>

#include<obj_predictor.hpp>

class EDTEnvironment
{
    public:
        EDTEnvironment(){}
        ~EDTEnvironment(){}

        void init();
        void setMap(SDFMap::Ptr map);
        void setObjPrediction(ObjPrediction obj_prediction);
        void setObjScale(ObjScale scale);
        void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dist[2][2][2]);

        void interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d& diff, double& value, Eigen::Vector3d& grad);
        void evaluateEDTWithGrad(const Eigen::Vector3d& pos, double time,double& dist, Eigen::Vector3d& grad);

        double evaluateCoarseEDT(Eigen::Vector3d& pos, double time);
        void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size){
            sdf_map_->getRegion(ori,size);
        }

        SDFMap::Ptr sdf_map_;

        typedef std::shared_ptr<EDTEnvironment> Ptr;

    private:
        ObjPrediction obj_prediction_;
        ObjScale obj_scale_;
        double resolution_inv_;
        double distToBox(int idx,const Eigen::Vector3d& pos,const double& time);
        double minDistToAllBox(const Eigen::Vector3d& pos,const double& time);
        

};


