#pragma once

#include<vector>
#include<string>
#include<Eigen/Eigen>
#include<yaml-cpp/yaml.h>
#include<deque>
#include<OctomapManager.hpp>

class RoomGraph
{
    public:
        struct RoomNode
        {
            int node_id;//编号
            std::string node_name;//名字
            Eigen::Vector3d node_place;//所在位置
            std::vector<Eigen::Vector3d> node_area;//node对应的区域
            std::vector<std::pair<int,double> > next_node;//储存下一个node的map
            std::vector<std::pair<int,Eigen::Vector3d> > outside_node;//储存连接的点id和位置(目前暂时由Eigen::Vector3d代替)
        };  
        RoomGraph(std::string package_path);
        void setid(std::string id);
        std::string getRoomID();
        void load_params();
        void add_node(int idx,
                      std::string node_name,
                      const Eigen::Vector3d& node_place,
                      const std::vector<Eigen::Vector3d> node_areas);
        void add_node(int idx,
                      std::string node_name,
                      const Eigen::Vector3d& node_place,
                      const std::vector<Eigen::Vector3d> node_areas,
                      const std::vector<std::pair<int,Eigen::Vector3d> >& outdoor_targets);
        void add_edge(int first,int next,double weight);
        void build_path();
        void printShortestPath();
        void printEdge();
        int CheckAngleWeight(const Eigen::Vector3d& first, const Eigen::Vector3d& next,const Eigen::Vector3d& pos);
        double GetAngle(const Eigen::Vector3d& first, const Eigen::Vector3d& next);
        int CheckInPoint(const Eigen::Vector3d& point);
        int get_first_point(int start_id,int final_id);
        std::vector<int> get_shortest_path_sequence(int start_id,int final_id);
        Eigen::Vector3d getNodePlace(int node_id);
        std::vector<Eigen::Vector3d> getNodeArea(int node_id);
        //TODO: 室内室外交互部分
        bool HaveOutSideConnect(int node_id);

    private:
        std::string RoomID;
        std::vector<RoomNode> nodes;
        std::vector<std::vector<std::vector<int> > > shortest_path_node;
        std::vector<int> shortest_path_sequence; 
        std::string package_path;
        OctoMapManagerPtr manager;

        inline double Distance(const Eigen::Vector3d& A,const Eigen::Vector3d& B)
        {
            return sqrt((B.x() - A.x()) * (B.x() - A.x()) + (B.y() - A.y()) * (B.y() - A.y()) + (B.z() - A.z()) * (B.z() - A.z()));
        }

        inline Eigen::Vector3d pos_vec_2_eigen(const std::vector<double>& vec)
        {
            Eigen::Vector3d pos(vec[0],vec[1],vec[2]);
            return pos;
        }

        inline std::vector<Eigen::Vector3d> pos_vec_2_eigen(const std::vector<std::vector<double> >& vec)
        {
            std::vector<Eigen::Vector3d> pos_eigen;
            for(int i = 0;i < vec.size();i++)
            {
                pos_eigen.push_back(pos_vec_2_eigen(vec[i]));
            }
            return pos_eigen;
        }
        //判断点是否在多边形内部
        inline bool pointPolygon(const std::vector<Eigen::Vector3d>& node_areas,const Eigen::Vector3d& point) 
        {
            std::vector<Eigen::Vector3d> node_areas_2d;
            for(auto &&area : node_areas)
            {
                node_areas_2d.push_back(Eigen::Vector3d(area.x(),area.y(),0));
            }
            Eigen::Vector3d point_2d(point.x(),point.y(),0);
            int nCross = 0;
            for(int i = 0;i < node_areas_2d.size();i++)
            {
                Eigen::Vector3d p1 = node_areas_2d[i];
                Eigen::Vector3d p2 = node_areas_2d[(i + 1) % node_areas_2d.size()];
                if(p1.y() == p2.y())
                    continue;
                if(point_2d.y() < std::min(p1.y(),p2.y()))
                    continue;
                if(point_2d.y() >= std::max(p1.y(),p2.y()))
                    continue;
                double x = (double)(point_2d.y() - p1.y()) * (double)(p2.x() - p1.x()) / (double)(p2.y() - p1.y()) + p1.x();
                if(x > point_2d.x())
                    nCross++;
            }

            return (nCross % 2 == 1);
        }





};