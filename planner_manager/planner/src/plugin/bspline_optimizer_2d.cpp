#include<bspline_optimizer_2d.hpp>

#include<nlopt.hpp>
#include<yaml-cpp/yaml.h>

//#define PRINT_COSTR

const int BsplineOptimizer2d::SMOOTHNESS     = (1 << 0);
const int BsplineOptimizer2d::DISTANCE       = (1 << 1);
const int BsplineOptimizer2d::FEASIBILITY    = (1 << 2);
const int BsplineOptimizer2d::ENDPOINT       = (1 << 3);
const int BsplineOptimizer2d::GUIDE          = (1 << 4);
const int BsplineOptimizer2d::WAYPOINT       = (1 << 5);

const int BsplineOptimizer2d::GUIDE_PHASE    = BsplineOptimizer2d::GUIDE | BsplineOptimizer2d::SMOOTHNESS;
const int BsplineOptimizer2d::NORMAL_PHASE   = BsplineOptimizer2d::SMOOTHNESS | BsplineOptimizer2d::DISTANCE | BsplineOptimizer2d::FEASIBILITY;

void BsplineOptimizer2d::setParams(std::string config_path)
{
    YAML::Node config = YAML::LoadFile(config_path + "/config/optimize_param.yaml");
    lambda1_ = config["lambda1"].as<double>();
    lambda2_ = config["lambda2"].as<double>();
    lambda3_ = config["lambda3"].as<double>();
    lambda4_ = config["lambda4"].as<double>();
    lambda5_ = config["lambda5"].as<double>();
    lambda6_ = config["lambda6"].as<double>();
    lambda7_ = config["lambda7"].as<double>();
    //lambda8_ = config["lambda8"].as<double>();

    dist0_ = config["dist0"].as<double>();
    max_vel = config["max_vel"].as<double>();
    max_acc = config["max_acc"].as<double>();
    //visib_min_ = config["visib_min"].as<double>();
    //dlmin_ = config["dlmin"].as<double>();
    //wnl_ = config["wnl"].as<double>();

    max_iteration_num_[0] = config["max_iteration_num1"].as<int>();
    max_iteration_num_[1] = config["max_iteration_num2"].as<int>();
    //max_iteration_num_[2] = config["max_iteration_num_2"].as<int>();
    //max_iteration_num_[3] = config["max_iteration_num_3"].as<int>();
    max_iteration_time_[0] = config["max_iteration_time1"].as<double>();
    max_iteration_time_[1] = config["max_iteration_time2"].as<double>();
    //max_iteration_time_[2] = config["max_iteration_num_2"].as<int>();
    //max_iteration_time_[3] = config["max_iteration_num_3"].as<int>();

    algorithm1_ = config["algorithm1"].as<int>();
    algorithm2_ = config["algorithm2"].as<int>();
    order_ = config["order"].as<int>();
}

void BsplineOptimizer2d::setEnvironment(const EDTEnvironment::Ptr& env)
{
    edt_environment_ = env;
}

void BsplineOptimizer2d::setControlPoints(const Eigen::MatrixXd& points)
{
    control_points = points;
    dim_ = points.cols();
}

void BsplineOptimizer2d::setBsplineInterval(const double& ts)
{
    bspline_interval = ts;
}

void BsplineOptimizer2d::setZ(const double& z)
{
    z_ = z;
}

void BsplineOptimizer2d::setTerminateCond(const int& max_num_id, const int max_time_id)
{
    max_num_id_ = max_num_id;
    max_time_id_ = max_time_id;
}

void BsplineOptimizer2d::setCostFunction(const int& cost_function)
{
    cost_function_ = cost_function;
    std::string cost_str;
    if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
    if (cost_function_ & DISTANCE) cost_str += " dist  |";
    if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
    if (cost_function_ & ENDPOINT) cost_str += " endpt |";
    if (cost_function_ & GUIDE) cost_str += " guide |";
    if (cost_function_ & WAYPOINT) cost_str += " waypt |";

    std::cout<<"Cost function: "<<cost_str<<std::endl;
}

void BsplineOptimizer2d::setGuidePath(const std::vector<Eigen::Vector2d>& guide_pts)
{
    guide_pts_ = guide_pts;
}

void BsplineOptimizer2d::setWaypoints(const std::vector<Eigen::Vector2d>& waypts,
                                      const std::vector<int>& waypt_idx)
{
    waypoints_ = waypts;
    waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer2d::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts, const int& cost_function, int max_num_id, int max_time_id)
{
    setControlPoints(points);
    setBsplineInterval(ts);
    setCostFunction(cost_function);
    setTerminateCond(max_num_id, max_time_id);

    optimize();
    return control_points;
}

void BsplineOptimizer2d::optimize()
{
    iter_num_ = 0;
    min_cost_ = std::numeric_limits<double>::max();
    const int pt_num = control_points.rows();
    g_q_.resize(pt_num);
    g_smoothness_.resize(pt_num);
    g_distance_.resize(pt_num);
    g_feasibility_.resize(pt_num);
    g_endpoint_.resize(pt_num);
    g_waypoints_.resize(pt_num);
    g_guide_.resize(pt_num);

    if(cost_function_ & ENDPOINT)
    {
        variable_num_ = dim_ * (pt_num - order_);
        end_pt = (1 / 6.0) * (control_points.row(pt_num - 3) + 4 * control_points.row(pt_num - 2) + control_points.row(pt_num - 1));
    }
    else
    {
        variable_num_ = max(0, dim_ * (pt_num - order_ * 2));
    }
    nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
    opt.set_min_objective(BsplineOptimizer2d::costFunction, this);
    opt.set_maxeval(max_iteration_num_[max_num_id_]);
    opt.set_maxtime(max_iteration_time_[max_time_id_]);
    opt.set_xtol_rel(1e-5);
    std::vector<double> q(variable_num_);
    for(int i = order_ ; i < pt_num; i++)
    {
        if(!(cost_function_ & ENDPOINT) && i >= pt_num - order_)
            continue;
        for(int j = 0; j < dim_; j++)
        {
            q[dim_ * (i - order_) + j] = control_points(i, j);
        }
    }
    if(dim_ != 1)
    {
        std::vector<double> lb(variable_num_), ub(variable_num_);
        const double bound = 10.0;
        for(int i = 0; i < variable_num_; i++)
        {
            lb[i] = q[i] - bound;
            ub[i] = q[i] + bound;
        }
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
    }
    try
    {
        double final_cost;
        nlopt::result result = opt.optimize(q, final_cost);
    }
    catch(const std::exception& e)
    {
        std::cout<<e.what()<<std::endl;
    }

    for(int i = order_ ; i < control_points.rows(); i++)
    {
        if(!(cost_function_ & ENDPOINT) && i >= pt_num - order_)
            continue;
        for(int j = 0; j < dim_; j++)
        {
            control_points(i, j) = best_variable_[dim_ * (i - order_) + j];
        }
    }

}

void BsplineOptimizer2d::calcSmoothnessCost(const std::vector<Eigen::Vector2d>& q,
                                            double& cost,std::vector<Eigen::Vector2d>& gradient)
{
    cost = 0.0;
    Eigen::Vector2d zero(0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);
    Eigen::Vector2d jerk, temp_j;

    for (int i = 0; i < q.size() - order_; i++) {
        /* evaluate jerk */
        jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i]; 
        cost += jerk.squaredNorm(); 
        temp_j = 2.0 * jerk; 
        /* jerk gradient */
        gradient[i + 0] += -temp_j;
        gradient[i + 1] += 3.0 * temp_j;
        gradient[i + 2] += -3.0 * temp_j;
        gradient[i + 3] += temp_j;
    }
}

void BsplineOptimizer2d::calcDistanceCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient)
{
    cost = 0.0;
    Eigen::Vector2d zero(0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);
    double dist;
    Eigen::Vector3d dist_grad, g_zero(0, 0, 0);
    int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;
    for(int i = 0; i < end_idx; i++)
    {
        Eigen::Vector3d q_temp(q[i].x(), q[i].y(), z_);
        edt_environment_->evaluateEDTWithGrad(q_temp, -1.0, dist, dist_grad);
        if (dist_grad.norm() > 1e-4) dist_grad.normalize();
        if (dist < dist0_)
        {
            cost += pow(dist - dist0_, 2);
            gradient[i] += 2.0 * (dist - dist0_) * dist_grad.block<2, 1>(0, 0);
        }
    }
}

void BsplineOptimizer2d::calcFeasibilityCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient)
{
    cost = 0.0;
    Eigen::Vector2d zero(0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    double ts, vm2, am2, ts_inv2, ts_inv4;
    vm2 = max_vel * max_vel;
    am2 = max_acc * max_acc;

    ts = bspline_interval;
    ts_inv2 = 1.0 / (ts * ts);
    ts_inv4 = ts_inv2 * ts_inv2;

    for (int i = 0; i < q.size() - 1; i++)
    {
        Eigen::Vector2d vi = q[i + 1] - q[i];
        for (int j = 0; j < 2; j++) 
        {
            double vd = vi(j) * vi(j) * ts_inv2 - vm2;
            if (vd > 0.0) 
            {
                cost += pow(vd, 2);

                double temp_v = 4.0 * vd * ts_inv2;
                gradient[i + 0](j) += -temp_v * vi(j);
                gradient[i + 1](j) += temp_v * vi(j);
            }
        }
    }
    for (int i = 0; i < q.size() - 2; i++) 
    {
        Eigen::Vector2d ai = q[i + 2] - 2 * q[i + 1] + q[i];
        for (int j = 0; j < 2; j++) {
            double ad = ai(j) * ai(j) * ts_inv4 - am2;
            if (ad > 0.0) 
            {
                cost += pow(ad, 2);

                double temp_a = 4.0 * ad * ts_inv4;
                gradient[i + 0](j) += temp_a * ai(j);
                gradient[i + 1](j) += -2 * temp_a * ai(j);
                gradient[i + 2](j) += temp_a * ai(j);
            }
        }
    }
}

void BsplineOptimizer2d::calcEndpointCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient)
{
    cost = 0.0;
    Eigen::Vector2d zero(0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    // zero cost and gradient in hard constraints
    Eigen::Vector2d q_3, q_2, q_1, dq;
    q_3 = q[q.size() - 3];
    q_2 = q[q.size() - 2];
    q_1 = q[q.size() - 1];

    dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt;
    cost += dq.squaredNorm();

    gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
    gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
    gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizer2d::calcGuideCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient)
{
    cost = 0.0;
    Eigen::Vector2d zero(0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    int end_idx = q.size() - order_;

    for (int i = order_; i < end_idx; i++) {
        Eigen::Vector2d gpt = guide_pts_[i - order_];
        cost += (q[i] - gpt).squaredNorm();
        gradient[i] += 2 * (q[i] - gpt);
    }
}

void BsplineOptimizer2d::calcVisibilityCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient)
{
    
}

void BsplineOptimizer2d::calcWaypointsCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient)
{
    cost = 0.0;
    Eigen::Vector2d zero(0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    Eigen::Vector2d q1, q2, q3, dq;

    // for (auto wp : waypoints_) {
    for (int i = 0; i < waypoints_.size(); ++i) {
        Eigen::Vector2d waypt = waypoints_[i];
        int             idx   = waypt_idx_[i];

        q1 = q[idx];
        q2 = q[idx + 1];
        q3 = q[idx + 2];

        dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
        cost += dq.squaredNorm();

        gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
        gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
        gradient[idx + 2] += dq * (2.0 / 6.0);
    }
}

void BsplineOptimizer2d::calcViewCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient)
{
    
}

void BsplineOptimizer2d::combineCost(const std::vector<double>& x, std::vector<double>& grad, double& f_combine)
{
    for(int i = 0;i < order_; i++)
    {
        for(int j = 0; j < dim_; j++)
        {
            g_q_[i](j) = control_points(i, j);
        }
    }

    for(int i = 0; i < variable_num_ / dim_ ; i++)
    {
        for(int j = 0; j < dim_;j++)
        {
            g_q_[i + order_][j] = x[i * dim_ + j];
        }
        for (int j = dim_; j < 2; ++j)
        {
            g_q_[i + order_][j] = 0.0;
        }
    }

    if(!(cost_function_ & ENDPOINT))
    {
        for(int i = 0; i < order_; i++)
        {
            for(int j = 0; j < dim_; j++)
            {
                g_q_[order_ + variable_num_ / dim_ + i][j] = control_points(control_points.rows() - order_ + i, j);
            }
            for (int j = dim_; j < 2; ++j) {
                g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
            }
        }
    }

    f_combine = 0.0;
    grad.resize(variable_num_);
    fill(grad.begin(), grad.end(), 0.0);

    double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
    f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

    if (cost_function_ & SMOOTHNESS) 
    {
        calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
        f_combine += lambda1_ * f_smoothness;
        for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
    }
    if (cost_function_ & DISTANCE) 
    {
        calcDistanceCost(g_q_, f_distance, g_distance_);
        f_combine += lambda2_ * f_distance;
        for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
    }
    if (cost_function_ & FEASIBILITY) 
    {
        calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
        f_combine += lambda3_ * f_feasibility;
        for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
    }
    if (cost_function_ & ENDPOINT) 
    {
        calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
        f_combine += lambda4_ * f_endpoint;
        for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
    }


    if (cost_function_ & GUIDE) 
    {
        calcGuideCost(g_q_, f_guide, g_guide_);
        f_combine += lambda5_ * f_guide;
        for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
    }
    if (cost_function_ & WAYPOINT) 
    {
        calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
        f_combine += lambda7_ * f_waypoints;
        for (int i = 0; i < variable_num_ / dim_; i++)
        for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
    }

#ifdef PRINT_COSTR
    std::cout<<"Total Cost: "<<f_combine<<std::endl;
    std::cout<<"Smoothness Cost: "<<f_smoothness<<
               " , Distance Cost: "<<f_distance<<
               " , Feasibility Cost: "<<f_feasibility<<
               " , Endpoint Cost: "<<f_endpoint<<
               " , Guide Cost: "<<f_guide<<
               " , Waypoints Cost: "<<f_waypoints<<std::endl;
#endif
}

double BsplineOptimizer2d::costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data)
{
    BsplineOptimizer2d* opt = reinterpret_cast<BsplineOptimizer2d*>(func_data);
    double cost;
    opt->combineCost(x, grad, cost);

    if(cost < opt->min_cost_)
    {
        opt->min_cost_ = cost;
        opt->best_variable_ = x;
    }

    return cost;
}

bool BsplineOptimizer2d::isQuadratic()
{
    if (cost_function_ == GUIDE_PHASE) {
        return true;
    } else if (cost_function_ == (SMOOTHNESS | WAYPOINT)) {
        return true;
    }
    return false;
}