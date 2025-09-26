#pragma once

#include<octomap/OcTree.h>
#include<octomap/octomap.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<boost/filesystem.hpp>

class PointCloudInfo
{
    public:
        typedef std::shared_ptr<PointCloudInfo> Ptr;
        typedef std::shared_ptr<const PointCloudInfo> ConstPtr;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        double octo_min_x{0};
        double octo_min_y{0};
        double octo_min_z{0};
        double octo_max_x{0};
        double octo_max_y{0};
        double octo_max_z{0};
        double octo_resol{0};
};
//管理每一个导入的Octomap
class OctoMapManager
{
    public:
        OctoMapManager() = default;
        OctoMapManager(std::string package_path);
        std::shared_ptr<octomap::OcTree> openOcTree(const std::string& package_path);
        void computePointCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
        PointCloudInfo::Ptr getCloudInfo();
    private:

        std::shared_ptr<octomap::OcTree> octo_tree;
        PointCloudInfo::Ptr cloud_info;
};

typedef std::shared_ptr<OctoMapManager> OctoMapManagerPtr;
typedef std::vector<OctoMapManagerPtr> OctoMapManagerVec;
