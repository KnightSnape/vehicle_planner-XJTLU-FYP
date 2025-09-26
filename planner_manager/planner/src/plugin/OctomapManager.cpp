#include<OctomapManager.hpp>

OctoMapManager::OctoMapManager(std::string package_path)
{
    octo_tree = openOcTree(package_path);
}

std::shared_ptr<octomap::OcTree> OctoMapManager::openOcTree(const std::string& package_path)
{
    /*
    if(!boost::filesystem::exists(package_path))
    {
        throw std::runtime_error(std::string("Cannot find file ") + package_path);
    }
    std::shared_ptr<octomap::OcTree> octo_tree;
    if(package_path.compare(package_path.length() - 3,3,".bt") == 0)
    {
        octo_tree.reset(new octomap::OcTree(package_path));
        if(!octo_tree->readBinary(package_path))
            throw std::runtime_error("OcTree cannot be read");
    }
    else if(package_path.compare(package_path.length() - 3,3,".ot") == 0)
    {
        octo_tree.reset(new octomap::OcTree(package_path));
        if(!octo_tree->read(package_path))
            throw std::runtime_error("OcTree cannot be read");
    }
    else
    {
        throw std::runtime_error(std::string("OcTree cannot be created from file ") + package_path);
    }
    */
    return nullptr;

}

void OctoMapManager::computePointCloud()
{
     if(!octo_tree)
        throw std::runtime_error("OcTree is NULL");
    const uint32_t octo_size = octo_tree->size();
    if(octo_size <= 1)
        throw std::runtime_error("OcTree is empty");
    cloud_info = std::make_shared<PointCloudInfo>();
    octo_tree->getMetricMin(cloud_info->octo_min_x,cloud_info->octo_min_y,cloud_info->octo_min_z);
    octo_tree->getMetricMax(cloud_info->octo_max_x,cloud_info->octo_max_y,cloud_info->octo_max_z);
    cloud_info->octo_resol = octo_tree->getResolution();
    cloud_info->cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointXYZ point;
    for(octomap::OcTree::leaf_iterator it = octo_tree->begin_leafs(); it != octo_tree->end_leafs(); ++it)
    {
        if(octo_tree->isNodeOccupied(*it) && it != nullptr)
        {
            point.x = static_cast<float>(it.getX());
            point.y = static_cast<float>(it.getY());
            point.z = static_cast<float>(it.getZ());
            cloud_info->cloud->push_back(point);
        }
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OctoMapManager::getCloud()
{
    return cloud_info->cloud;
}

PointCloudInfo::Ptr OctoMapManager::getCloudInfo()
{
    return cloud_info;
}