#pragma once

#include<rclcpp/rclcpp.hpp>
#include<Eigen/Eigen>
#include<vector>
#include<topo_prm.hpp>
#include<polynormial_traj.hpp>
#include<non_uniform_bspline.hpp>
#include<algorithm>
#include<iostream>
#include<obj_predictor.hpp>
#include<visualization_msgs/msg/marker_array.hpp>
#include<visualization_msgs/msg/marker.hpp>

using namespace planner;

class PlannerNode;

class PlanningVisualization
{
    public:
        typedef std::shared_ptr<PlanningVisualization> Ptr;
        PlanningVisualization(PlannerNode* planner_node);

        void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);
        void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id, int pub_id = 0);
        void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                       double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);
        
        void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                   int id2 = 0);

        // draw a set of bspline trajectories generated in different phases
        void drawPolynomialTraj(PolynomialTraj traj,double resolution, const Eigen::Vector4d& color,int id = 0);
        void drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size);
        void drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size);

        void drawTopoGraph(list<GraphNode::Ptr>& graph, double point_size, double line_width,
                            const Eigen::Vector4d& color1, const Eigen::Vector4d& color2,
                            const Eigen::Vector4d& color3, int id = 0);

        void drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths, double line_width);
        void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
        void drawPrediction(ObjPrediction pred,double resolution, const Eigen::Vector4d& color, int id = 0);
    private:
        enum TRAJECTORY_PLANNING_ID{
            GOAL = 1,
            PATH = 200,
            BSPLINE = 300,
            BSPLINE_CTRL_PT = 400,
            POLY_TRAJ = 500,
        };

        enum TOPOLOGICAL_PATH_PLANNING_ID{
            GRAPH_NODE = 1,
            GRAPH_EDGE = 100,
            RAW_PATH = 200,
            FILTERED_PATH = 300,
            SELECT_PATH = 400,
        };

        Eigen::Vector4d getColor(double h, double alpha = 1.0);
        
        int last_topo_path1_num_;
        int last_bspline_phase2_num_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_;
        PlannerNode* planner_node_;
};