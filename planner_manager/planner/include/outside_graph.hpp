#pragma once

#include<string>
#include<Eigen/Eigen>
#include<yaml-cpp/yaml.h>
#include<deque>
#include<OctomapManager.hpp>

struct Node
{
    int idx;
    std::string name;
    Eigen::Vector3d pos;
    std::vector<Eigen::Vector3d> node_area;
    std::vector<std::pair<int,double> > next_node;
    std::map<int, std::vector<std::pair<Node, int>> > connect_outside_node;//储存哪个连接其他区块的节点以及其ID
};

class BlockNode
{
    public:
        BlockNode();
        void loadBlock(int idx, const std::string& package_path);
        bool is_inBlock(const Eigen::Vector3d& pos); //判断点是否在区块内
        void loadCloud(const std::string& cloud_path); //加载点云
        void unregisterCloud(); //注销点云
        void add_node(int idx, const std::string& name, const Eigen::Vector3d& pos, const std::vector<Eigen::Vector3d>& node_area);//添加节点
        void add_edge(int first,int next,double weight); //添加边
        void add_outside_node_connection(int node_idx,std::vector<std::pair<Node, int>> connect_node); //添加外部节点连接
        void build_path(); //建立路径
        void printShortestPath(int start_idx, int end_idx); //打印最短路径
        void printEdge(); //打印边
        int get_first_point_inBlockNode(int start_id,int final_id); //获取第一个点
        Eigen::Vector3d getNodePlace(int node_id); //获取节点位置
        std::vector<Eigen::Vector3d> getNodeArea(int node_id); //获取节点区域
        bool HaveOutSideConnect(int node_id); //是否有外部连接
        std::vector<Node> getNodesFromConnectIdx(int idx); //获取有外部连接的连接节点


    private:
        std::vector<std::vector<std::vector<int> > > shortest_path_node;//从i到j的最短路径，存储路径上的节点
        int idx; //节点索引
        std::string name; //节点名称
        Eigen::Vector3d anchor_pos; //锚点位置
        std::vector<Eigen::Vector3d> Block; //节点整个区域
        OctoMapManagerPtr cloud_manager; //点云管理
        std::vector<Node> nodes; //节点
};

class OutSideGraph
{
    public:
        OutSideGraph(std::string package_path);


    private:
        std::vector<BlockNode> blocks;

};