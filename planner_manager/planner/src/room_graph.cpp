#include<room_graph.hpp>

RoomGraph::RoomGraph(std::string package_path)
{
    this->package_path = package_path;
}

void RoomGraph::load_params()
{
    std::string test_config = this->package_path;
    YAML::Node Config = YAML::LoadFile(test_config);
    std::string idx = Config["room_name"].as<std::string>();
    int num = Config["points"].as<int>();
    for(int i = 1;i <= num; i++)
    {
        std::vector<double> pos_vec = Config["point"]["idx" + std::to_string(i)]["pos"].as<std::vector<double>>();
        Eigen::Vector3d pos = pos_vec_2_eigen(pos_vec);
        int level = Config["point"]["idx" + std::to_string(i)]["level"].as<int>();
        std::vector<std::vector<double> > area = Config["point"]["idx" + std::to_string(i)]["area"].as<std::vector<std::vector<double>>>();
        std::vector<Eigen::Vector3d> area_eigen = pos_vec_2_eigen(area);
        int types = Config["point"]["idx" + std::to_string(i)]["type"].as<int>();
        
        for(int j = 1;j <= types;j++)
        {
            std::vector<double> outpos = Config["point"]["idx" + std::to_string(i)]["outside" + std::to_string(j)]["exitpoint"].as<std::vector<double>>();
            Eigen::Vector3d outpos_eigen = pos_vec_2_eigen(outpos);
            int outpos_idx = Config["point"]["idx" + std::to_string(i)]["outside" + std::to_string(j)]["outidx"].as<int>();
        }
        add_node(i - 1,idx,pos,area_eigen);
    }

    for(int i = 0;i < num;i++)
    {
        auto connections = Config["connection"]["idx" + std::to_string(i + 1)]["points"].as<std::vector<std::vector<int>>>();
        for(auto &&connection : connections)
        {
            add_edge(i,connection[0] - 1,connection[1]);
        }
    }

    build_path();
}

void RoomGraph::add_node(int idx,
                        std::string node_name,
                        const Eigen::Vector3d& node_place,
                        const std::vector<Eigen::Vector3d> node_areas){
    RoomNode node;
    node.node_id = idx;
    node.node_name = node_name;
    node.node_place = node_place;
    node.node_area = node_areas;
    nodes.push_back(node);
}

void RoomGraph::add_node(int idx,
                        std::string node_name,
                        const Eigen::Vector3d& node_place,
                        const std::vector<Eigen::Vector3d> node_areas,
                        const std::vector<std::pair<int,Eigen::Vector3d> >& outdoor_targets){
    RoomNode node;
    node.node_id = idx;
    node.node_name = node_name;
    node.node_place = node_place;
    node.node_area = node_areas;
    node.outside_node = outdoor_targets;
    nodes.push_back(node);
}

void RoomGraph::add_edge(int first,int next,double weight)
{
    nodes[first].next_node.push_back(std::make_pair(next,weight));
}

void RoomGraph::build_path()
{
    shortest_path_node.resize(nodes.size());
    for(int i = 0;i < shortest_path_node.size();i++)
    {
        shortest_path_node[i].resize(nodes.size());
    }
    for(auto &&start_node : nodes)
    {
        int pre[nodes.size()];
        int vis[nodes.size()];
        memset(vis,0,sizeof(vis));
        int dis[nodes.size()];
        memset(dis,0x3f,sizeof(dis));
        dis[start_node.node_id] = 0;
        for(int i = 0;i < nodes.size();i++)
        {
            int u = 0, mind = 0x3f3f3f3f;
            for(int j = 0;j < nodes.size();j++)
            {
                if(!vis[j] && dis[j] < mind)
                {
                    u = j;
                    mind = dis[j];
                }
            }
            vis[u] = true;
            for(auto ed : nodes[u].next_node)
            {
                int v = ed.first;
                double w = ed.second;
                if(dis[v] > dis[u] + w)
                {
                    dis[v] = dis[u] + w;
                    pre[v] = u;
                }
            }
        }

        for(int i = 0;i < nodes.size();i++)
        {
            std::vector<int> temp_vector;
            int temp = i;
            while(temp != start_node.node_id)
            {
                temp_vector.push_back(temp);
                temp = pre[temp];
            }
            temp_vector.push_back(start_node.node_id);
            shortest_path_node[start_node.node_id][i] = temp_vector;
        }
    }
}

void RoomGraph::printShortestPath()
{
    for(int i = 0;i < shortest_path_node.size();i++)
    {
        for(int j = 0;j < shortest_path_node[i].size();j++)
        {
            std::cout << "From " << i << " to " << j << " : ";
            for(auto &&k : shortest_path_node[i][j])
            {
                std::cout << k << " ";
            }
            std::cout << std::endl;
        }
    }
}

void RoomGraph::printEdge()
{
    for(auto &&node : nodes)
    {
        std::cout << "Node " << node.node_id << " : ";
        for(auto &&next : node.next_node)
        {
            std::cout << "Next " << next.first << " Weight " << next.second << " ";
        }
        std::cout << std::endl;
    }
}

int RoomGraph::CheckAngleWeight(const Eigen::Vector3d& first, const Eigen::Vector3d& next,const Eigen::Vector3d& pos)
{
    Eigen::Vector2d first_plant(first.x(),first.y());
    Eigen::Vector2d next_plant(next.x(),next.y());

    Eigen::Vector2d pos_plant(pos.x(),pos.y());
    Eigen::Vector2d first_vec = pos_plant - first_plant;
    Eigen::Vector2d next_vec = next_plant - first_plant;
    double dot = first_vec.dot(next_vec);

    if(dot < 0)
    {
        return -1;
    }
    else if(dot > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int RoomGraph::CheckInPoint(const Eigen::Vector3d& point)
{
    for(auto &&node : nodes)
    {
        if(pointPolygon(node.node_area,point))
        {
            return node.node_id;
        }
    }
    return -1;
}

int RoomGraph::get_first_point(int start_id,int final_id)
{
    int first_id;
    if(shortest_path_node[start_id][final_id].size() == 1)
    {
        first_id = start_id;
    }
    else
    {
        first_id = shortest_path_node[start_id][final_id][shortest_path_node[start_id][final_id].size() - 2];
    }
    return first_id;
}

std::vector<int> RoomGraph::get_shortest_path_sequence(int start_id,int final_id)
{
    return shortest_path_node[start_id][final_id];
}

Eigen::Vector3d RoomGraph::getNodePlace(int node_id)
{
    return nodes[node_id].node_place;
}

std::vector<Eigen::Vector3d> RoomGraph::getNodeArea(int node_id)
{
    return nodes[node_id].node_area;
}