#include<sdfMap.hpp>
#include<planner_node.hpp>

SDFMap::SDFMap(std::string package_path):planner_node_(nullptr)
{
    use_rclcpp = false;
    initMap(package_path);
}

SDFMap::SDFMap(PlannerNode* node,std::string package_path):planner_node_(node)
{
    use_rclcpp = true;
    initMap(package_path);
}

SDFMap::~SDFMap()
{
    if(use_rclcpp = true)
    {
        if(update_esdf_timer_)
        {
            update_esdf_timer_->cancel();
        }
        delete planner_node_;
    }
}

void SDFMap::resetBuffer()
{
    Eigen::Vector3d min_pos = mp_.map_min_boundary_;
    Eigen::Vector3d max_pos = mp_.map_max_boundary_;

    resetBuffer(min_pos, max_pos);

    md_.local_bound_min_ = Eigen::Vector3i::Zero();
    md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(Eigen::Vector3d min,Eigen::Vector3d max)
{
    Eigen::Vector3i min_id, max_id;
    posToIndex(min, min_id);
    posToIndex(max, max_id);

    boundIndex(min_id);
    boundIndex(max_id);

    for(int x = min_id(0); x <= max_id(0); ++x)
        for(int y = min_id(1); y <= max_id(1); ++y)
            for(int z = min_id(2); z <= max_id(2); ++z)
            {
                md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
                md_.distance_buffer_[toAddress(x, y, z)] = 1 << 30;
            }
}

void SDFMap::setOdomPos(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    md_.has_odom_ = true;
    md_.car_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    md_.car_quat_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    //需要旋转
    md_.camera_pos_ = md_.car_pos_;
    md_.camera_q_ = md_.car_quat_;
}

inline void SDFMap::pushOctoMap2SDF(const PointCloudInfo::Ptr &cloud_info)
{
    md_.has_cloud_ = true;
    if(!md_.has_odom_)  return;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud_info->cloud;
    if(cloud->empty())
    {
        return;
    }
    pcl::PointXYZ pt;
    Eigen::Vector3d p3d, p3d_inf;
    int inf_step = ceil(mp_.obstacle_inflation_ / mp_.resolution);
    int inf_step_z = 1;

    double max_x, max_y, max_z, min_x, min_y, min_z;
    min_x = mp_.map_min_boundary_(0);
    min_y = mp_.map_min_boundary_(1);
    min_z = mp_.map_min_boundary_(2);

    max_x = mp_.map_max_boundary_(0);
    max_y = mp_.map_max_boundary_(1);
    max_z = mp_.map_max_boundary_(2);

    for(size_t i = 0; i < cloud->points.size();i++)
    {
        pt = cloud->points[i];
        p3d = Eigen::Vector3d(pt.x, pt.y, pt.z);

        Eigen::Vector3d devi = p3d - md_.car_pos_;
        Eigen::Vector3i inf_pt;

        if(fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) && fabs(devi(2)) < mp_.local_update_range_(2))
        {
            for(int x = -inf_step; x <= inf_step; x++)
            {
                for(int y = -inf_step; y <= inf_step; y++)
                {
                    for(int z = -inf_step_z; z <= inf_step_z; z++)
                    {
                        p3d_inf(0) = pt.x + x * mp_.resolution;
                        p3d_inf(1) = pt.y + y * mp_.resolution;
                        p3d_inf(2) = pt.z + z * mp_.resolution;

                        max_x = max(max_x, p3d_inf(0));
                        max_y = max(max_y, p3d_inf(1));
                        max_z = max(max_z, p3d_inf(2));

                        min_x = min(min_x, p3d_inf(0));
                        min_y = min(min_y, p3d_inf(1));
                        min_z = min(min_z, p3d_inf(2));

                        posToIndex(p3d_inf, inf_pt);
                        if(!isInMap(inf_pt))
                            continue;

                        int idx_inf = toAddress(inf_pt);
                        md_.occupancy_buffer_inflate_[idx_inf] = 1;
                    }
                }
            }
        }
    }

    min_x = min(min_x, md_.car_pos_(0));
    min_y = min(min_y, md_.car_pos_(1));
    min_z = min(min_z, md_.car_pos_(2));

    max_x = max(max_x, md_.car_pos_(0));
    max_y = max(max_y, md_.car_pos_(1));
    max_z = max(max_z, md_.car_pos_(2));

    max_z = max(max_z, mp_.ground_height_);

    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    md_.esdf_need_update_ = true;
}

void SDFMap::updateMapWithCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    md_.has_cloud_ = true;
    if(!md_.has_odom_)
        return;
    if(cloud->empty())
    {
        return;
    }

    if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))) return;
    this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);
    pcl::PointXYZ pt;
    Eigen::Vector3d p3d, p3d_inf;
    int inf_step = ceil(mp_.obstacle_inflation_ / mp_.resolution);
    int inf_step_z = 1;

    double max_x, max_y, max_z, min_x, min_y, min_z;
    min_x = mp_.map_max_boundary_(0);
    min_y = mp_.map_max_boundary_(1);
    min_z = mp_.map_max_boundary_(2);

    max_x = mp_.map_min_boundary_(0);
    max_y = mp_.map_min_boundary_(1);
    max_z = mp_.map_min_boundary_(2);

    for(size_t i = 0; i < cloud->points.size();i++)
    {
        pt = cloud->points[i];
        p3d = Eigen::Vector3d(pt.x, pt.y, pt.z);

        Eigen::Vector3d devi = p3d - md_.car_pos_;
        Eigen::Vector3i inf_pt;

        if(fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) && fabs(devi(2)) < mp_.local_update_range_(2))
        {
            for(int x = -inf_step; x <= inf_step; x++)
            {
                for(int y = -inf_step; y <= inf_step; y++)
                {
                    for(int z = -inf_step_z; z <= inf_step_z; z++)
                    {
                        p3d_inf(0) = pt.x + x * mp_.resolution;
                        p3d_inf(1) = pt.y + y * mp_.resolution;
                        p3d_inf(2) = pt.z + z * mp_.resolution;

                        max_x = max(max_x, p3d_inf(0));
                        max_y = max(max_y, p3d_inf(1));
                        max_z = max(max_z, p3d_inf(2));

                        min_x = min(min_x, p3d_inf(0));
                        min_y = min(min_y, p3d_inf(1));
                        min_z = min(min_z, p3d_inf(2));

                        posToIndex(p3d_inf, inf_pt);
                        if(!isInMap(inf_pt))
                            continue;

                        int idx_inf = toAddress(inf_pt);
                        md_.occupancy_buffer_inflate_[idx_inf] = 1;
                    }
                }
            }
        }
    }

    min_x = min(min_x, md_.car_pos_(0));
    min_y = min(min_y, md_.car_pos_(1));
    min_z = min(min_z, md_.car_pos_(2));

    max_x = max(max_x, md_.car_pos_(0));
    max_y = max(max_y, md_.car_pos_(1));
    max_z = max(max_z, md_.car_pos_(2));

    max_z = max(max_z, mp_.ground_height_);

    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    md_.esdf_need_update_ = true;
}

inline void SDFMap::posToIndex(const Eigen::Vector3d& pos,Eigen::Vector3i& id)
{
    for (int i = 0; i < 3; ++i)
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv);
}

inline void SDFMap::indexToPos(const Eigen::Vector3i& id,Eigen::Vector3d& pos)
{
    for(int i = 0; i < 3; i++)
        pos(i) = (id(i) + 0.5)*mp_.resolution + mp_.map_origin_(i);
}

inline int SDFMap::toAddress(const Eigen::Vector3i &id)
{
    assert(id(0) >= 0 && id(0) < mp_.map_voxel_num_(0));
    assert(id(1) >= 0 && id(1) < mp_.map_voxel_num_(1));
    assert(id(2) >= 0 && id(2) < mp_.map_voxel_num_(2));
    return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int SDFMap::toAddress(int& x,int& y,int& z)
{
    assert(x >= 0 && x < mp_.map_voxel_num_(0));
    assert(y >= 0 && y < mp_.map_voxel_num_(1));
    assert(z >= 0 && z < mp_.map_voxel_num_(2));
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline bool SDFMap::isInMap(const Eigen::Vector3d& pos)
{
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
        pos(2) < mp_.map_min_boundary_(2) + 1e-4)
        return false;
    if(pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 || pos(2) > mp_.map_max_boundary_(2) - 1e-4)
        return false;
    return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i& idx)
{
    if(idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
        return false;
    if(idx(0) > mp_.map_voxel_num_(0) - 1|| idx(1) >= mp_.map_voxel_num_(1) - 1 || idx(2) >= mp_.map_voxel_num_(2) - 1)
        return false;
    return true;
}

double SDFMap::getDistance(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    boundIndex(id);

    return md_.distance_buffer_all_[toAddress(id)];
}

double SDFMap::getDistance(const Eigen::Vector3i& idx)
{
    Eigen::Vector3i id1 = idx;
    boundIndex(id1);
    return md_.distance_buffer_all_[toAddress(id1)];
}

inline double SDFMap::getDistWidthGradTrilinear(const Eigen::Vector3d& pos, Eigen::Vector3d& grad)
{
    if(!isInMap(pos))
    {
        grad.setZero();
        return 0;
    }

    Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution * Eigen::Vector3d::Ones();
}

template<typename F_get_val,typename F_set_val>
void SDFMap::fillESDF(F_get_val get_val, F_set_val set_val,int start,int end,int dim)
{
    int v[mp_.map_voxel_num_(dim)];
    double z[mp_.map_voxel_num_(dim) + 1];

    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();

    for(int q = start + 1; q <= end; q++)
    {
        k++;
        double s;
        do
        {
            k--;
            s = ((get_val(q) + q * q) - (get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
        } while(s <= z[k]);
        k++;
        v[k] = q;
        z[k] = s;

        z[k + 1] = std::numeric_limits<double>::max();
    }
    k = start;
    for(int q = start; q <= end; q++)
    {
        while(z[k + 1] < q)
            k++;
        double val = (q - v[k]) * (q - v[k]) + get_val(v[k]);
        set_val(q, val);
    }

}

void SDFMap::getSurroundPts(const Eigen::Vector3d &pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d &diff)
{
    if (!isInMap(pos)) {
        // cout << "pos invalid for interpolation." << endl;
    }
    Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution * Eigen::Vector3d::Ones(); //中心点
    Eigen::Vector3i idx;
    Eigen::Vector3d idx_pos;
    posToIndex(pos_m, idx);
    indexToPos(idx, idx_pos);

    diff = (pos - idx_pos) * mp_.resolution_inv;

    for (int x = 0; x < 2; x++) 
    {
        for (int y = 0; y < 2; y++) 
        {
            for (int z = 0; z < 2; z++) 
            {
                Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                Eigen::Vector3d current_pos;
                indexToPos(current_idx, current_pos);
                pts[x][y][z] = current_pos;
            }
        }
    }
}

void SDFMap::updateESDF3d()
{
    Eigen::Vector3i min_esdf = md_.local_bound_min_;
    Eigen::Vector3i max_esdf = md_.local_bound_max_;

    #pragma omp parallel for collapse(2) schedule(dynamic)
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
                return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                    0 :
                    std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
        }
    }

    #pragma omp parallel for collapse(2) schedule(dynamic)
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
                [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
                max_esdf[1], 1);
        }
    }

    #pragma omp parallel for collapse(2) schedule(dynamic)
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
                [&](int x, double val) {
                    md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution * std::sqrt(val);
                    //  min(mp_.resolution_ * std::sqrt(val),
                    //      md_.distance_buffer_[toAddress(x, y, z)]);
                },
                min_esdf[0], max_esdf[0], 0);
        }
    }

    #pragma omp parallel for collapse(3) schedule(dynamic)
    for(int x = min_esdf[0]; x <= max_esdf[0]; x++)
    {
        for(int y = min_esdf[1]; y <= max_esdf[1]; y++)
        {
            for(int z = min_esdf[2]; z <= max_esdf[2]; z++)
            {
                int idx = toAddress(x, y, z);
                if(md_.occupancy_buffer_inflate_[idx] == 1)
                {
                    md_.occupancy_buffer_neg[idx] = 0;
                }
                else if(md_.occupancy_buffer_inflate_[idx] == 0)
                {
                    md_.occupancy_buffer_neg[idx] = 1;
                }
                else
                {
                    RCLCPP_WARN(planner_node_->get_logger(), "what?");
                }
            }
        }
    }

    #pragma omp parallel for collapse(2) schedule(dynamic)
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
                return md_.occupancy_buffer_neg[x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                                y * mp_.map_voxel_num_(2) + z] == 1 ?
                    0 :
                    std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
        }
    }

    #pragma omp parallel for collapse(2) schedule(dynamic)
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
                [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
                max_esdf[1], 1);
        }
    }

    #pragma omp parallel for collapse(2) schedule(dynamic)
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
                [&](int x, double val) {
                    md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution * std::sqrt(val);
                },
                min_esdf[0], max_esdf[0], 0);
        }
    }

    //Now combine the positive and negative distance fields
    #pragma omp parallel for collapse(3) schedule(dynamic)
    for(int x = min_esdf[0]; x <= max_esdf[0]; x++)
    {
        for(int y = min_esdf[1]; y <= max_esdf[1]; y++)
        {
            for(int z = min_esdf[2]; z <= max_esdf[2]; z++)
            {
                int idx = toAddress(x, y, z);
                md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];
                if(md_.distance_buffer_neg_[idx] > 0.0)//如果负占据地图有值
                    md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution);
            }
        }
    }

}

void SDFMap::initMap(std::string package_path)
{
    //parameters init
    double x_size, y_size, z_size;

    YAML::Node config = YAML::LoadFile(package_path + "/config/sdf_map_param.yaml");
    mp_.resolution = config["map_resolution"].as<double>();
    x_size = config["map_size_x"].as<double>();
    y_size = config["map_size_y"].as<double>();
    z_size = config["map_size_z"].as<double>();

    mp_.fx_ = config["fx"].as<double>();
    mp_.fy_ = config["fy"].as<double>();
    mp_.cx_ = config["cx"].as<double>();
    mp_.cy_ = config["cy"].as<double>();

    mp_.local_update_range_(0) = config["local_update_range_x"].as<double>();
    mp_.local_update_range_(1) = config["local_update_range_y"].as<double>();
    mp_.local_update_range_(2) = config["local_update_range_z"].as<double>();

    mp_.obstacle_inflation_ = config["obstacle_inflation"].as<double>();
    mp_.use_depth_filter_ = config["use_depth_filter"].as<bool>();
    mp_.depth_filter_tolerance_ = config["depth_filter_tolerance"].as<double>();
    mp_.depth_filter_maxdist_ = config["depth_filter_maxdist"].as<double>();
    mp_.depth_filter_mindist_ = config["depth_filter_mindist"].as<double>();
    mp_.depth_filter_margin_ = config["depth_filter_margin"].as<int>();
    mp_.k_depth_scaling_factor_ = config["k_depth_scaling_factor"].as<double>();
    mp_.skip_pixel_ = config["skip_pixel"].as<int>();

    mp_.p_hit_ = config["p_hit"].as<double>();
    mp_.p_miss_ = config["p_miss"].as<double>();
    mp_.p_min_ = config["p_min_"].as<double>();
    mp_.p_max_ = config["p_max_"].as<double>();
    mp_.p_occ_ = config["p_occ_"].as<double>();
    mp_.min_ray_length_ = config["min_ray_length"].as<double>();
    mp_.max_ray_length_ = config["max_ray_length"].as<double>();

    mp_.esdf_slice_height_ = config["esdf_slice_height"].as<double>();
    mp_.visualization_truncate_height_ = config["visualization_truncate_height"].as<double>();
    mp_.virtual_ceil_height_ = config["virtual_ceil_height"].as<double>();

    mp_.show_occ_time_ = config["show_occ_time"].as<bool>();
    mp_.show_esdf_time_ = config["show_esdf_time"].as<bool>();
    mp_.pose_type = config["pose_type_"].as<int>();

    mp_.frame_id_ = config["frame_id"].as<std::string>();
    mp_.local_bound_inflate_ = config["local_bound_inflate"].as<double>();
    mp_.local_map_margin_ = config["local_map_margin"].as<int>();
    mp_.ground_height_ = config["ground_height"].as<double>();

    mp_.local_bound_inflate_ = max(mp_.resolution, mp_.local_bound_inflate_);
    mp_.resolution_inv = 1.0 / mp_.resolution;
    mp_.map_origin_ = Eigen::Vector3d(-x_size / 2, -y_size / 2, mp_.ground_height_);
    mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

    mp_.prob_hit_log_ = logit(mp_.p_hit_);
    mp_.prob_miss_log_ = logit(mp_.p_miss_);
    mp_.clamp_min_log_ = logit(mp_.p_min_);
    mp_.clamp_max_log_ = logit(mp_.p_max_);
    mp_.min_occupancy_log_ = logit(mp_.p_occ_);
    mp_.unknown_flag_ = 0.01;

    std::cout << "hit: " << mp_.prob_hit_log_ << std::endl;
    std::cout << "miss: " << mp_.prob_miss_log_ << std::endl;
    std::cout << "min log: " << mp_.clamp_min_log_ << std::endl;
    std::cout << "max: " << mp_.clamp_max_log_ << std::endl;
    std::cout << "thresh log: " << mp_.min_occupancy_log_ << std::endl;

    for(int i = 0;i < 3;i++)
    {
        mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution);
    }

    mp_.map_min_boundary_ = mp_.map_origin_;
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

    mp_.map_min_idx_ = Eigen::Vector3i::Zero();
    mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

    int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

    md_.occupancy_buffer_ = std::vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
    md_.occupancy_buffer_neg = std::vector<char>(buffer_size, 0);
    md_.occupancy_buffer_inflate_ = std::vector<char>(buffer_size, 0);

    md_.distance_buffer_ = std::vector<double>(buffer_size, 10000);
    md_.distance_buffer_neg_ = std::vector<double>(buffer_size, 10000);
    md_.distance_buffer_all_ = std::vector<double>(buffer_size, 10000);

    md_.count_hit_and_miss = vector<short>(buffer_size, 0);
    md_.count_hit_ = vector<short>(buffer_size, 0);
    md_.flag_rayend_ = vector<char>(buffer_size, -1);
    md_.flag_traverse_ = vector<char>(buffer_size, -1);

    md_.tmp_buffer1_ = std::vector<double>(buffer_size, 0);
    md_.tmp_buffer2_ = std::vector<double>(buffer_size, 0);
    md_.raycast_num_ = 0;

    md_.proj_points_.resize(1280 * 720 / mp_.skip_pixel_ / mp_.skip_pixel_);
    md_.proj_points_cnt = 0;

    if(use_rclcpp)
    {
        sdf_cloud_pub_ = planner_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy",100000);
        sdf_esdf_cloud_pub_ = planner_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf",100000);
        update_vis_timer_ = planner_node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SDFMap::updateViscallback, this));
        update_esdf_timer_ = planner_node_->create_wall_timer(std::chrono::milliseconds(5), std::bind(&SDFMap::updateESDFcallback, this)); 
        //std::async::launch(std::async(&SDFMap::updateESDFcallback, this));
        update_occ_timer_ = planner_node_->create_wall_timer(std::chrono::milliseconds(30), std::bind(&SDFMap::updateOccupancycallback, this));
    }
    
    md_.has_first_cloud_ = false;
    md_.occ_need_update_ = false;
    md_.has_odom_ = false;
    md_.local_updated_ = false;
    md_.has_cloud_ = false;
    md_.esdf_need_update_ = false;
    md_.image_cnt_ = 0;

    md_.esdf_time_ = 0.0;
    md_.fuse_time_ = 0.0;
    md_.update_num_ = 0;
    md_.max_esdf_time_ = 0.0;
    md_.max_fuse_time_ = 0.0;

    rand_noise = uniform_real_distribution<double>(-0.2, 0.2);
    rand_noise2_ = normal_distribution<double>(0, 0.2);
    random_device rd;
    eng_ = default_random_engine(rd());

    if(use_rclcpp)
        RCLCPP_INFO(planner_node_->get_logger(), "SDFMap init success!");
}

void SDFMap::boundIndex(Eigen::Vector3i& id)
{
    Eigen::Vector3i id1;
    id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
    id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
    id = id1;
}

bool SDFMap::isUnknown(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return isUnknown(id);
}

bool SDFMap::isUnknown(const Eigen::Vector3i& idx)
{
    Eigen::Vector3i id1 = idx;
    boundIndex(id1);
    return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

bool SDFMap::isKnownFree(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return isKnownFree(id);
}

bool SDFMap::isKnownFree(const Eigen::Vector3i& idx)
{
    Eigen::Vector3i id1 = idx;
    boundIndex(id1);
    int adr = toAddress(id1);
    return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;//未膨胀
}

inline void SDFMap::setOccupied(Eigen::Vector3d pos)
{
    if(!isInMap(pos))
    {
        return;
    }
    Eigen::Vector3i id;
    posToIndex(pos, id);
    md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                  id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

inline void SDFMap::setOccupancy(Eigen::Vector3d pos,double occ)
{
    if(occ != 1 && occ != 0)
    {
        std::cout<<"occ must be 0 or 1"<<std::endl;
        return;
    }
    if(!isInMap(pos))
    {
        return;
    }
    Eigen::Vector3i id;
    posToIndex(pos, id);

    md_.occupancy_buffer_[toAddress(id)] = occ;
}

void SDFMap::updateOccupancycallback()
{
    if(!md_.occ_need_update_)
    {
        return;
    } 
    std::chrono::steady_clock::time_point t1 , t2;
    t1 = std::chrono::steady_clock::now();
    projectDepthImage();
    raycastProcess();
    if(md_.local_updated_)
    {
        clearAndInflateLocalMap();
    }
    t2 = std::chrono::steady_clock::now();
    md_.fuse_time_ = std::chrono::duration<double>(t2 - t1).count();
    md_.max_fuse_time_ = std::max(md_.max_fuse_time_, md_.fuse_time_);
    if(mp_.show_occ_time_)
        std::cout<<"Fuse time: "<<md_.fuse_time_<<std::endl;

    md_.occ_need_update_ = false;
    if(md_.local_updated_)
    {
        md_.esdf_need_update_ = true;
    }
    md_.local_updated_ = false;
}

void SDFMap::updateESDFcallback()
{
    if(!md_.esdf_need_update_)
    {
        return;
    }
    auto t1 = std::chrono::steady_clock::now();
    updateESDF3d();

    auto t2 = std::chrono::steady_clock::now();
    md_.esdf_time_ = std::chrono::duration<double>(t2 - t1).count();

    md_.max_esdf_time_ = std::max(md_.max_esdf_time_, md_.esdf_time_);
    if(mp_.show_esdf_time_)
        std::cout<<"ESDF time: "<<md_.esdf_time_<<std::endl;

    md_.esdf_need_update_ = false;
}

void SDFMap::updateViscallback()
{
    publishCloud();
    publishESDF();
}

void SDFMap::publishCloud()
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;

    int lmm = mp_.local_map_margin_ / 2;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);

    boundIndex(min_cut);
    boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = min_cut(2); z <= max_cut(2); ++z) {
            if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

            Eigen::Vector3d pos;
            indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > mp_.visualization_truncate_height_) continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
        }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud,cloud_msg);
    cloud_msg.header.stamp = planner_node_->get_clock()->now();
    sdf_cloud_pub_->publish(cloud_msg);
}

void SDFMap::publishESDF()
{
    double dist;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI pt;

    const double min_dist = 0.0;
    const double max_dist = 3.0;

    Eigen::Vector3i min_cut = md_.local_bound_min_ -
    Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    Eigen::Vector3i max_cut = md_.local_bound_max_ +
    Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    boundIndex(min_cut);
    boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y) {

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, 1), pos);
        pos(2) = mp_.esdf_slice_height_;

        dist = getDistance(pos);
        dist = min(dist, max_dist);
        dist = max(dist, min_dist);

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = -0.2;
        pt.intensity = (dist - min_dist) / (max_dist - min_dist);
        cloud.push_back(pt);
        } 

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    sdf_esdf_cloud_pub_->publish(cloud_msg);
}

void SDFMap::inflatePoint(const Eigen::Vector3i& pt,int step,std::vector<Eigen::Vector3i>& pts)
{
    int num = 0;
    for(int x = -step;x <= step; ++x)
    {
        for(int y = -step; y <= step; ++y)
        {
            for(int z = -step; z <= step; ++z)
            {
                pts[++num] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);//膨胀
            }
        }
    }
}

void SDFMap::checkDist()
{
    for(int x = 0;x < mp_.map_voxel_num_(0); ++x)
    {
        for(int y = 0;y < mp_.map_voxel_num_(1); ++y)
        {
            for(int z = 0;z < mp_.map_voxel_num_(2); ++z)
            {
                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, z), pos);

                Eigen::Vector3d grad;
                double dist = getDistWidthGradTrilinear(pos, grad);

                if(fabs(dist - getDistance(pos)) > 1e-3)
                {
                    //std::cout<<"dist: "<<dist<<" getDistance: "<<getDistance(pos)<<std::endl;
                }
            }
        }
    }
}

int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
    if (occ != 1 && occ != 0) return INVALID_IDX;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    int idx_ctns = toAddress(id);

    md_.count_hit_and_miss[idx_ctns] += 1;

    if (md_.count_hit_and_miss[idx_ctns] == 1) {
        md_.cache_voxel_.push(id);
    }

    if (occ == 1) md_.count_hit_[idx_ctns] += 1;

    return idx_ctns;
}

bool SDFMap::odomValid()
{
    return md_.has_odom_;
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size)
{
    ori = mp_.map_origin_;
    size = mp_.map_size_;
}

Eigen::Vector3d SDFMap::getOrigin()
{
    return mp_.map_origin_;
}

double SDFMap::getResolution()
{
    return mp_.resolution;
}

int SDFMap::getVoxelNum()
{
    return mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
}

bool SDFMap::hasDepthObservation()
{
    return md_.has_first_cloud_;
}

void SDFMap::depthhandle(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    if(cv_ptr->image.empty())
    {
        return;
    }
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1,mp_.k_depth_scaling_factor_);
    cv_ptr->image.copyTo(md_.depth_image_);
    md_.occ_need_update_ = true;
}

Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
    Eigen::Vector3d diff = pt - camera_pt;
    Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
    Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

    double min_t = 1000000;

    for (int i = 0; i < 3; ++i) {
        if (fabs(diff[i]) > 0) {

        double t1 = max_tc[i] / diff[i];
        if (t1 > 0 && t1 < min_t) min_t = t1;

        double t2 = min_tc[i] / diff[i];
        if (t2 > 0 && t2 < min_t) min_t = t2;
        }
    }

    return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::projectDepthImage()
{
    md_.proj_points_cnt = 0;
    uint16_t* row_ptr;
    int cols = md_.depth_image_.cols;
    int rows = md_.depth_image_.rows;

    double depth;

    Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

    if(!mp_.use_depth_filter_)
    {
        for(int v = 0; v < rows; v++)
        {
            row_ptr = md_.depth_image_.ptr<uint16_t>(v);
            for(int u = 0;u < cols;u++)
            {
                Eigen::Vector3d proj_pt;
                depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
                proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
                proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
                proj_pt(2) = depth;

                proj_pt = camera_r * proj_pt + md_.camera_pos_;
                md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
            }
        }
    }
    /* use depth filter*/
    else
    {
        if(!md_.has_first_depth_)
            md_.has_first_depth_ = true;
        else
        {
            Eigen::Vector3d pt_cur, pt_world, pt_reproj;
            Eigen::Matrix3d last_camera_r_inv;

            last_camera_r_inv = md_.last_camera_q.inverse();
            const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

            for(int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
            {
                row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;
                for(int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_)
                {   
                    depth = (*row_ptr) * inv_factor;
                    row_ptr = row_ptr + mp_.skip_pixel_;

                    if(*row_ptr == 0)
                    {
                        depth = mp_.max_ray_length_ + 0.1;
                    }
                    else if(depth < mp_.depth_filter_mindist_)
                    {
                        continue;
                    }
                    else if(depth > mp_.depth_filter_maxdist_)
                    {
                        depth = mp_.max_ray_length_ + 0.1;
                    }

                    pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
                    pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
                    pt_cur(2) = depth;

                    pt_world = camera_r * pt_cur + md_.camera_pos_;

                    md_.proj_points_[md_.proj_points_cnt++] = pt_world;
                }
            }
        }
    }

    md_.last_camera_pos_ = md_.camera_pos_;
    md_.last_camera_q = md_.camera_q_;
    md_.last_depth_image_ = md_.depth_image_;
}

void SDFMap::raycastProcess()
{
    if(md_.proj_points_cnt == 0) return;
    md_.raycast_num_ += 1;
    
    int vox_idx;
    double length;

    double min_x = mp_.map_max_boundary_(0);
    double min_y = mp_.map_max_boundary_(1);
    double min_z = mp_.map_max_boundary_(2);

    double max_x = mp_.map_min_boundary_(0);
    double max_y = mp_.map_min_boundary_(1);
    double max_z = mp_.map_min_boundary_(2);

    RayCaster raycaster;
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d ray_pt, pt_w;

    for(int i = 0;i < md_.proj_points_cnt; ++i)
    {
        pt_w = md_.proj_points_[i];
        if(!isInMap(pt_w))
        {
            pt_w = closetPointInMap(pt_w, md_.camera_pos_);
            length = (pt_w - md_.camera_pos_).norm();
            if(length > mp_.max_ray_length_)
            {
                pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
            }
            vox_idx = setCacheOccupancy(pt_w, 0);
        }
        else
        {
            length = (pt_w - md_.camera_pos_).norm();
            if(length > mp_.max_ray_length_)
            {
                pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
                vox_idx = setCacheOccupancy(pt_w, 0);
            }
            else
            {
                vox_idx = setCacheOccupancy(pt_w, 1);
            }
        }

        max_x = max(max_x, pt_w(0));
        max_y = max(max_y, pt_w(1));
        max_z = max(max_z, pt_w(2));

        min_x = min(min_x, pt_w(0));
        min_y = min(min_y, pt_w(1));
        min_z = min(min_z, pt_w(2));

        if(vox_idx != INVALID_IDX)
        {
            if(md_.flag_rayend_[vox_idx] == md_.raycast_num_)
            {
                continue;
            }
            else
            {
                md_.flag_rayend_[vox_idx] = md_.raycast_num_;
            }
        }

        raycaster.setInput(pt_w / mp_.resolution, md_.camera_pos_ / mp_.resolution);
        while(raycaster.step(ray_pt))
        {
            Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution;
            length = (tmp - md_.camera_pos_).norm();

            vox_idx = setCacheOccupancy(tmp, 0);
            if(vox_idx != INVALID_IDX)
            {
                if(md_.flag_traverse_[vox_idx] == md_.raycast_num_)
                {
                    break;
                }
                else
                {
                    md_.flag_traverse_[vox_idx] = md_.raycast_num_;
                }
            }
        }
    }

    min_x = min(min_x, md_.camera_pos_(0));
    min_y = min(min_y, md_.camera_pos_(1));
    min_z = min(min_z, md_.camera_pos_(2));

    max_x = max(max_x, md_.camera_pos_(0));
    max_y = max(max_y, md_.camera_pos_(1));
    max_z = max(max_z, md_.camera_pos_(2));

    max_z = max(max_z, mp_.ground_height_);
    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

    int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution);
    md_.local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
    md_.local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    md_.local_updated_ = true;

    Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
    Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

    Eigen::Vector3i min_id, max_id;

    posToIndex(local_range_min, min_id);
    posToIndex(local_range_max, max_id);

    boundIndex(min_id);
    boundIndex(max_id);

    while(!md_.cache_voxel_.empty())
    {
        Eigen::Vector3i idx = md_.cache_voxel_.front();
        int idx_ctns = toAddress(idx);
        md_.cache_voxel_.pop();

        double log_odds_update = md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;
        md_.count_hit_[idx_ctns] = md_.count_hit_and_miss[idx_ctns] = 0;

        if(log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
        {
            continue;
        }
        else if(log_odds_update < 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
        {
            md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
            continue;
        }

        bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1);
        if(!in_local)
        {
            md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
        }

        md_.occupancy_buffer_[idx_ctns] = std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_), mp_.clamp_max_log_);
    }

}

void SDFMap::clearAndInflateLocalMap()
{
    const int vec_margin = 5;

    Eigen::Vector3i min_cut = md_.local_bound_min_ -
    Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    Eigen::Vector3i max_cut = md_.local_bound_max_ +
    Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    boundIndex(min_cut);
    boundIndex(max_cut);

    Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    boundIndex(min_cut_m);
    boundIndex(max_cut_m);

    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
        for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) 
        {

            for (int z = min_cut_m(2); z < min_cut(2); ++z) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }

            for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }
        }

    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
        for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) 
        {
            for (int y = min_cut_m(1); y < min_cut(1); ++y) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }

            for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }
        }

    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
        for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) 
        {
            for (int x = min_cut_m(0); x < min_cut(0); ++x) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }

            for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }
        }
    // inflate occupied voxels to compensate robot size
    int inf_step = ceil(mp_.obstacle_inflation_ / mp_.resolution);
    // int inf_step_z = 1;
    std::vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
    // inf_pts.resize(4 * inf_step + 3);
    Eigen::Vector3i inf_pt;

    // clear outdated data
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
        for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
            md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        }

    // inflate obstacles
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
        for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {

            if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) {
            inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

            for (int k = 0; k < (int)inf_pts.size(); ++k) {
                inf_pt = inf_pts[k];
                int idx_inf = toAddress(inf_pt);
                if (idx_inf < 0 ||
                    idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
                continue;
                }
                md_.occupancy_buffer_inflate_[idx_inf] = 1;
            }
            }
        }

    // add virtual ceiling to limit flight height
    if (mp_.virtual_ceil_height_ > -0.5) {
        int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv);
        for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
            md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
        }
    }
}