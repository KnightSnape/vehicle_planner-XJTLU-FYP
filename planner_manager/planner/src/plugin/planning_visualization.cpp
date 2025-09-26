#include<planning_visualization.hpp>
#include<planner_node.hpp>

PlanningVisualization::PlanningVisualization(PlannerNode* planner_node): planner_node_(planner_node)
{
    traj_pub_ = planner_node_->create_publisher<visualization_msgs::msg::MarkerArray>("/planning_visualization", 10);

    last_topo_path1_num_ = 0;
    last_bspline_phase2_num_ = 0;
}

void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "lidar";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "planning_visualization";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(marker);
    traj_pub_->publish(marker_array);
    marker_array.markers.clear();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.color.a = color(3);
    //marker.lifetime = rclcpp::Duration(0.1);
    marker.frame_locked = true;

    for (int i = 0; i < list.size(); ++i)
    {
        geometry_msgs::msg::Point pt;
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    traj_pub_->publish(marker_array);
}

void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id, int pub_id)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "lidar";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "planning_visualization";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(marker);
    traj_pub_->publish(marker_array);
    marker_array.markers.clear();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.color.a = color(3);
    //marker.lifetime = rclcpp::Duration(0.1);
    marker.frame_locked = true;

    for (int i = 0; i < list.size(); ++i)
    {
        geometry_msgs::msg::Point pt;
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    traj_pub_->publish(marker_array);
}

void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                       double line_width, const Eigen::Vector4d& color, int id, int pub_id)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "lidar";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "planning_visualization";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(marker);
    traj_pub_->publish(marker_array);
    marker_array.markers.clear();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = line_width;
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.color.a = color(3);
    //marker.lifetime = rclcpp::Duration(0.1);

    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    for (int i = 0; i < list1.size(); ++i)
    {
        geometry_msgs::msg::Point pt;
        pt.x = list1[i](0);
        pt.y = list1[i](1);
        pt.z = list1[i](2);
        marker.points.push_back(pt);

        pt.x = list2[i](0);
        pt.y = list2[i](1);
        pt.z = list2[i](2);
        marker.points.push_back(pt);
    }
    marker_array.markers.push_back(marker);
    traj_pub_->publish(marker_array);
}

void PlanningVisualization::drawBspline(NonUniformBspline& bspline, double size,
                                        const Eigen::Vector4d& color, bool show_ctrl_pts, double size2,
                                        const Eigen::Vector4d& color2, int id1, int id2) {
    if(bspline.getControlPoints().size() == 0) return;
    vector<Eigen::Vector3d> traj_pts;
    double tm, tmp;

    bspline.getTimeSpan(tm, tmp);
    for (double t = tm; t <= tmp; t += 0.01) {
        Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
        traj_pts.push_back(pt);
    }
    displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);

    // draw the control point
    if (!show_ctrl_pts) return;

    Eigen::MatrixXd         ctrl_pts = bspline.getControlPoints();
    vector<Eigen::Vector3d> ctp;

    for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
        Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
        ctp.push_back(pt);
    }

    displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
}

void PlanningVisualization::drawPolynomialTraj(PolynomialTraj traj,double resolution, const Eigen::Vector4d& color,int id)
{
    traj.init();
    std::vector<Eigen::Vector3d> poly_pts = traj.getTraj();
    displaySphereList(poly_pts, resolution, color, POLY_TRAJ + id % 100);
}

void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id)
{
    std::vector<Eigen::Vector3d> goal_vec = { goal };
    displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}
 
void PlanningVisualization::drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size)
{
    std::vector<Eigen::Vector3d> empty;
    for(int i = 0; i < last_bspline_phase2_num_; i++)
    {
        displaySphereList(empty, size, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), BSPLINE + (50 + i) % 100);
        displaySphereList(empty, size, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), BSPLINE_CTRL_PT + (50 + i) % 100);
    }
    last_bspline_phase2_num_ = bsplines.size();
    for (int i = 0; i < bsplines.size(); ++i)
    {
        drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.3), false, 1.5 * size,
        getColor(double(i) / bsplines.size()), 50 + i, 50 + i);
    }
}

void PlanningVisualization::drawTopoGraph(list<GraphNode::Ptr>& graph, double point_size, double line_width,
                                         const Eigen::Vector4d& color1, const Eigen::Vector4d& color2,
                                         const Eigen::Vector4d& color3, int id)
{
    std::vector<Eigen::Vector3d> empty;
    displaySphereList(empty, point_size, color1, GRAPH_NODE, 1);
    displaySphereList(empty, point_size, color1, GRAPH_NODE + 50, 1);
    displayLineList(empty, empty, line_width, color3, GRAPH_EDGE, 1);

    std::vector<Eigen::Vector3d> guards, connectors;
    for(std::list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter)
    {
        if ((*iter)->type_ == GraphNode::Guard) {
            guards.push_back((*iter)->pos_);
        } else if ((*iter)->type_ == GraphNode::Connector) {
            connectors.push_back((*iter)->pos_);
        }
    }

    displaySphereList(guards, point_size, color1, GRAPH_NODE, 1);
    displaySphereList(connectors, point_size, color2, GRAPH_NODE + 50, 1);
    std::vector<Eigen::Vector3d> edge_pt1, edge_pt2;

    for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter)
    {
        for (int k = 0; k < (*iter)->neighbors_.size(); ++k) 
        {
            edge_pt1.push_back((*iter)->pos_);
            edge_pt2.push_back((*iter)->neighbors_[k]->pos_);
        }
    }
    displayLineList(edge_pt1, edge_pt2, line_width, color3, GRAPH_EDGE, 1);
}

void PlanningVisualization::drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths, double line_width)
{
    Eigen::Vector4d color1(1, 1, 1, 1);
    for (int i = 0; i < last_topo_path1_num_; ++i) {
        vector<Eigen::Vector3d> empty;
        displayLineList(empty, empty, line_width, color1, SELECT_PATH + i % 100, 1);
        displaySphereList(empty, line_width, color1, PATH + i % 100, 1);
    }
  
    last_topo_path1_num_ = paths.size();
  
    for (int i = 0; i < paths.size(); ++i) {
        vector<Eigen::Vector3d> edge_pt1, edge_pt2;
    
        for (int j = 0; j < paths[i].size() - 1; ++j) {
            edge_pt1.push_back(paths[i][j]);
            edge_pt2.push_back(paths[i][j + 1]);
        }
    
        displayLineList(edge_pt1, edge_pt2, line_width, getColor(double(i) / (last_topo_path1_num_)),
                        SELECT_PATH + i % 100, 1);
    }
}

Eigen::Vector4d PlanningVisualization::getColor(double h, double alpha)
{
    if (h < 0.0 || h > 1.0) {
        std::cout << "h out of range" << std::endl;
        h = 0.0;
    }

    double          lambda;
    Eigen::Vector4d color1, color2;
    if (h >= -1e-4 && h < 1.0 / 6) {
    lambda = (h - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);

    } else if (h >= 1.0 / 6 && h < 2.0 / 6) {
    lambda = (h - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);

    } else if (h >= 2.0 / 6 && h < 3.0 / 6) {
    lambda = (h - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);

    } else if (h >= 3.0 / 6 && h < 4.0 / 6) {
    lambda = (h - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);

    } else if (h >= 4.0 / 6 && h < 5.0 / 6) {
    lambda = (h - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);

    } else if (h >= 5.0 / 6 && h <= 1.0 + 1e-4) {
    lambda = (h - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
    }

    Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
    fcolor(3)              = alpha;

    return fcolor;
}