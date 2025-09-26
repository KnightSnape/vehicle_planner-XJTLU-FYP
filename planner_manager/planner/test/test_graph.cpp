#include<room_graph.hpp>
#include<ament_index_cpp/get_package_share_directory.hpp>
#include<yaml-cpp/yaml.h>

int main(int argc,char **argv)
{
    std::string package_path = ament_index_cpp::get_package_share_directory("planner") + "/config/test_config2.yaml";
    RoomGraph room_graph(package_path);
    room_graph.load_params();

    room_graph.build_path();
    room_graph.printShortestPath();

    return 0;
}