#include<OctomapManager.hpp>
#include<ament_index_cpp/get_package_share_directory.hpp>
#include<pcl/io/pcd_io.h>
int main(int argc,char **argv)
{
    std::string package_path = ament_index_cpp::get_package_share_directory("planner") + "/map/octomap.bt";
    OctoMapManager octo_map_manager(package_path);
    octo_map_manager.computePointCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = octo_map_manager.getCloud();
    auto cloudinfo = octo_map_manager.getCloudInfo();
    std::cout<< "Cloud info: " << cloudinfo->octo_min_x << 
                           " " << cloudinfo->octo_min_y << 
                           " " << cloudinfo->octo_min_z << 
                           " " << cloudinfo->octo_max_x << 
                           " " << cloudinfo->octo_max_y << 
                           " " << cloudinfo->octo_max_z << 
                           " " << cloudinfo->octo_resol << std::endl;
    
    
    
    std::string write_path = ament_index_cpp::get_package_share_directory("planner") + "/map/output.pcd";
    pcl::io::savePCDFileASCII(write_path, *cloud);
    
    return 0;
}