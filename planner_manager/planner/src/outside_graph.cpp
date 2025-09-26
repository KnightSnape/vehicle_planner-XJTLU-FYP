#include<outside_graph.hpp>

BlockNode::BlockNode()
{
    
}

void BlockNode::loadBlock(int idx, const std::string& package_path)
{
    
}

bool BlockNode::is_inBlock(const Eigen::Vector3d& pos)
{
    
}

void BlockNode::loadCloud(const std::string& cloud_path)
{
    if(!boost::filesystem::exists(cloud_path))
    {
        throw std::runtime_error(std::string("Cannot find file ") + cloud_path);
    }
    cloud_manager.reset(new OctoMapManager(cloud_path));
}

void BlockNode::unregisterCloud()
{
    cloud_manager.reset();//注销点云
}

void BlockNode::add_node(int idx, const std::string& name, const Eigen::Vector3d& pos, const std::vector<Eigen::Vector3d>& node_area)
{
    Node node;
    node.idx = idx;
    node.name = name;
    node.pos = pos;
    node.node_area = node_area;
    nodes.push_back(node);
}

void BlockNode::add_edge(int first,int next,double weight)
{
    nodes[first].next_node.push_back(std::make_pair(next,weight));
}

void BlockNode::add_outside_node_connection(int node_idx,std::vector<std::pair<Node, int>> connect_node)
{
    nodes[node_idx].connect_outside_node.insert(std::make_pair(node_idx,connect_node));
}

void BlockNode::build_path()
{

}

void BlockNode::printShortestPath(int start_idx, int end_idx)
{

}

void BlockNode::printEdge()
{
    
}

int BlockNode::get_first_point_inBlockNode(int start_id,int final_id)
{

}

Eigen::Vector3d BlockNode::getNodePlace(int node_id)
{

}

std::vector<Eigen::Vector3d> BlockNode::getNodeArea(int node_id)
{

}

bool BlockNode::HaveOutSideConnect(int node_id)
{

}

std::vector<Node> BlockNode::getNodesFromConnectIdx(int idx)
{

}

OutSideGraph::OutSideGraph(std::string package_path)
{

}