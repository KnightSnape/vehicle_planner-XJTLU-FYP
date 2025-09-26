#pragma once

#include<Eigen/Eigen>
#include<iostream>
#include<random>
#include<queue>
#include<rclcpp/rclcpp.hpp>
#include<tuple>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<pcl_conversions/pcl_conversions.h>
#include<functional>
#include<chrono>
#include<yaml-cpp/yaml.h>
#include<opencv2/opencv.hpp>
#include<OctomapManager.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include<raycast.hpp>
#include<cv_bridge/cv_bridge.h>
#include<future>
#include<omp.h>

using namespace std;

class PlannerNode;

#define logit(x) (log((x) / (1 - (x))))

template<typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
    std::size_t operator()(T const &matrix) const
    {
        size_t seed = 0;
        for(size_t i = 0; i < matrix.size(); ++i)
        {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::value_type>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

struct MappingParameters
{
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;
    Eigen::Vector3i map_voxel_num_; // number of voxels in each dimension
    Eigen::Vector3i map_min_idx_, map_max_idx_; // min and max index of the map
    Eigen::Vector3d local_update_range_; // local update range

    double resolution, resolution_inv;
    double obstacle_inflation_;// obstacle inflation radius
    std::string map_input_;
    std::string frame_id_;
    int pose_type;

    double fx_, fy_, cx_, cy_; 
    double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
    int depth_filter_margin_;
    bool use_depth_filter_;
    double k_depth_scaling_factor_;
    int skip_pixel_;

    double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;
    double min_ray_length_, max_ray_length_; // min and max ray length

    double local_bound_inflate_;
    int local_map_margin_;

    double esdf_slice_height_, visualization_truncate_height_, virtual_ceil_height_, ground_height_;
    bool show_esdf_time_, show_occ_time_;

    /* active mapping */
    double unknown_flag_;  
};

struct MappingData
{
    std::vector<double> occupancy_buffer_; //占据地图
    std::vector<char> occupancy_buffer_neg; //负占据地图
    std::vector<char> occupancy_buffer_inflate_; //膨胀
    std::vector<double> distance_buffer_;
    std::vector<double> distance_buffer_neg_;
    std::vector<double> distance_buffer_all_;
    std::vector<double> tmp_buffer1_;
    std::vector<double> tmp_buffer2_;  

    Eigen::Vector3d camera_pos_, last_camera_pos_;
    Eigen::Quaterniond camera_q_, last_camera_q;
    Eigen::Matrix4d camera_2_car;

    Eigen::Vector3d car_pos_, last_car_pos_;
    Eigen::Quaterniond car_quat_, last_car_quat_;

    bool occ_need_update_;     //是否需要更新占据地图
    bool local_updated_;       //是否局部更新
    bool esdf_need_update_;    //是否需要更新ESDF
    bool has_first_cloud_;     //是否有第一帧点云
    bool has_first_depth_;
    bool has_odom_;            //是否有里程计信息
    bool has_cloud_;           //是否有点云信息

    cv::Mat depth_image_, last_depth_image_;
    int image_cnt_;

    std::vector<Eigen::Vector3d> proj_points_;
    int proj_points_cnt;
    std::vector<short> count_hit_, count_hit_and_miss;
    std::vector<char> flag_traverse_, flag_rayend_;
    char raycast_num_;
    std::queue<Eigen::Vector3i> cache_voxel_;

    std::vector<Eigen::Vector3d> incremental_obstacles_; //增量障碍物
    bool has_incremental_obstacles_; //是否有增量障碍物

    Eigen::Vector3i local_bound_min_, local_bound_max_; //局部地图边界
    double fuse_time_; //融合时间
    double esdf_time_; //ESDF时间
    double max_fuse_time_; //最大融合时间
    double max_esdf_time_; //最大ESDF时间
    int update_num_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


class SDFMap
{
    public:
        enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 };
        SDFMap(std::string package_path);
        SDFMap(PlannerNode* node,std::string package_path);
        ~SDFMap();

        void resetBuffer();
        void resetBuffer(Eigen::Vector3d min,Eigen::Vector3d max);

        void setOdomPos(const nav_msgs::msg::Odometry::SharedPtr msg);
        inline void pushOctoMap2SDF(const PointCloudInfo::Ptr &cloud_info);
        void updateMapWithCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

        inline void posToIndex(const Eigen::Vector3d& pos,Eigen::Vector3i& id);//将三维坐标转换为三维索引
        inline void indexToPos(const Eigen::Vector3i& id,Eigen::Vector3d& pos);
        inline int toAddress(const Eigen::Vector3i &id);//将三维坐标转换为一维地址
        inline int toAddress(int& x,int& y,int& z);
        inline bool isInMap(const Eigen::Vector3d& pos);
        inline bool isInMap(const Eigen::Vector3i& idx);

        inline void boundIndex(Eigen::Vector3i& idx);//边界检查
        inline bool isUnknown(const Eigen::Vector3d& pos);//是否未知
        inline bool isUnknown(const Eigen::Vector3i& idx);//是否未知
        inline bool isKnownFree(const Eigen::Vector3d& pos);//是否空闲
        inline bool isKnownFree(const Eigen::Vector3i& idx);//是否空闲

        inline void setOccupancy(Eigen::Vector3d pos,double occ = 1);
        inline void setOccupied(Eigen::Vector3d pos);
        inline int getOccupancy(const Eigen::Vector3d& pos);
        inline int getOccupancy(const Eigen::Vector3i& idx);

        double getDistance(const Eigen::Vector3d& pos);
        double getDistance(const Eigen::Vector3i& idx);
        inline double getDistWidthGradTrilinear(const Eigen::Vector3d& pos,Eigen::Vector3d& grad);//三线性插值
        void getSurroundPts(const Eigen::Vector3d &pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d &diff);

        void updateESDF3d();
        void getSliceESDF(const double height,const double res,const Eigen::Vector4d& range,
                          std::vector<Eigen::Vector3d>& slice, std::vector<Eigen::Vector3d>& grad,int sign = 1);

        void DownSampleToCV(cv::Mat& img_src,int sample_rate);
        void depthhandle(const sensor_msgs::msg::Image::SharedPtr msg);

        Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
        int setCacheOccupancy(Eigen::Vector3d pos, int occ);
        void projectDepthImage();
        void raycastProcess();
        void clearAndInflateLocalMap();
        
        void initMap(std::string package_path);
        void checkDist();
        bool hasDepthObservation();
        bool odomValid();
        double getResolution();
        void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
        Eigen::Vector3d getOrigin();
        int getVoxelNum();

        typedef std::shared_ptr<SDFMap> Ptr;

    private:
        MappingParameters mp_;
        MappingData md_;

        template<typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val get_val, F_set_val set_val,int start,int end,int dim);
        
        void updateOccupancycallback();
        void updateESDFcallback();
        void updateViscallback();

        void publishCloud();
        void publishESDF();

        inline void inflatePoint(const Eigen::Vector3i& pt,int step,std::vector<Eigen::Vector3i>& pts);

        bool use_rclcpp;

        std::uniform_real_distribution<double> rand_noise;
        std::normal_distribution<double> rand_noise2_;
        std::default_random_engine eng_;

        PlannerNode* planner_node_;
        rclcpp::TimerBase::SharedPtr update_esdf_timer_;
        rclcpp::TimerBase::SharedPtr update_occ_timer_;
        rclcpp::TimerBase::SharedPtr update_vis_timer_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sdf_cloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sdf_esdf_cloud_pub_;

        
};