#include<iostream>

#include<polynormial_traj.hpp>


void PolynomialTraj::reset()
{
    times.clear();
    cxs.clear();
    cys.clear();
    czs.clear();
    time_sum = 0;
    num_seg = 0;
}

void PolynomialTraj::addSegment(std::vector<double> cx, std::vector<double> cy, std::vector<double> cz, double time)
{
    times.push_back(time);
    cxs.push_back(cx);
    cys.push_back(cy);
    czs.push_back(cz);
}

void PolynomialTraj::init()
{
    num_seg = times.size();

    time_sum = 0.0;
    for(int i = 0; i < num_seg; i++)
    {
        time_sum += times[i];
    }
}

Eigen::Vector3d PolynomialTraj::evaluate(double t)
{
    int idx = 0;
    while(t > times[idx] && t - times[idx] > 0.0005)
    {
        t -= times[idx];
        idx++;
    }
    int order = cxs[idx].size();
    Eigen::VectorXd cx(order), cy(order), cz(order), tv(order);
    for(int i = 0;i < order;i++)
    {
        cx(i) = cxs[idx][i], cy(i) = cys[idx][i], cz(i) = czs[idx][i];
        tv(order - i - 1) = std::pow(t, double(i));
    }

    Eigen::Vector3d pt;
    pt(0) = tv.dot(cx), pt(1) = tv.dot(cy), pt(2) = tv.dot(cz);
    return pt;
}

Eigen::Vector3d PolynomialTraj::evaluateVel(double t)
{
    int idx = 0;
    while(t > times[idx] && t - times[idx] > 0.0005)
    {
        t -= times[idx];
        idx++;
    }
    int order = cxs[idx].size();
    Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);
    
    for(int i = 0;i < order - 1;i++)
    {
        vx(i) = double(i + 1) * cxs[idx][order - 2 - i];
        vy(i) = double(i + 1) * cys[idx][order - 2 - i];
        vz(i) = double(i + 1) * czs[idx][order - 2 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 1);
    for(int i = 0;i < order - 1;i++)
    {
        tv(i) = std::pow(ts, double(i));
    }

    Eigen::Vector3d vel;
    vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);

    return vel;    
}

Eigen::Vector3d PolynomialTraj::evaluateAcc(double t)
{
    int idx = 0;
    while(t > times[idx] && t - times[idx] > 0.0005)
    {
        t -= times[idx];
        idx++;
    }
    int order = cxs[idx].size();
    Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);
    
    for(int i = 0;i < order - 2;i++)
    {
        ax(i) = double(i + 1) * double(i + 2) * cxs[idx][order - 3 - i];
        ay(i) = double(i + 1) * double(i + 2) * cys[idx][order - 3 - i];
        az(i) = double(i + 1) * double(i + 2) * czs[idx][order - 3 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 2);
    for(int i = 0;i < order - 2;i++)
    {
        tv(i) = std::pow(ts, double(i));
    }

    Eigen::Vector3d acc;
    acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);

    return acc;    
}

double PolynomialTraj::getTimeSum()
{
    return this->time_sum;
}

std::vector<Eigen::Vector3d> PolynomialTraj::getTraj()
{
    double eval_t = 0.0;
    traj_vec3d.clear();
    while(eval_t < time_sum)
    {
        traj_vec3d.push_back(evaluate(eval_t));
        eval_t += 0.01;
    }
    return traj_vec3d;
}

double PolynomialTraj::getLength()
{
    length = 0.0;
    Eigen::Vector3d p_l = traj_vec3d[0], p_n;
    for(int i = 1; i < traj_vec3d.size(); i++)
    {
        p_n = traj_vec3d[i];
        length += (p_n - p_l).norm();
        p_l = p_n;
    }
    return length;
}

double PolynomialTraj::getMeanVel()
{
    return length / time_sum; 
}

double PolynomialTraj::getAccCost()
{
    double cost = 0.0;
    int order = cxs[0].size();

    for (int s = 0; s < times.size(); ++s) {
        Eigen::Vector3d um;
        um(0) = 2 * cxs[s][order - 3], um(1) = 2 * cys[s][order - 3], um(2) = 2 * czs[s][order - 3];
        cost += um.squaredNorm() * times[s];
    }

    return cost;
}

double PolynomialTraj::getJerk() 
{
    double jerk = 0.0;

    /* evaluate jerk */
    for (int s = 0; s < times.size(); ++s) 
    {
        Eigen::VectorXd cxv(cxs[s].size()), cyv(cys[s].size()), czv(czs[s].size());
        /* convert coefficient */
        int order = cxs[s].size();
        for (int j = 0; j < order; ++j) {
        cxv(j) = cxs[s][order - 1 - j], cyv(j) = cys[s][order - 1 - j], czv(j) = czs[s][order - 1 - j];
        }
        double ts = times[s];

        /* jerk matrix */
        Eigen::MatrixXd mat_jerk(order, order);
        mat_jerk.setZero();
        for (double i = 3; i < order; i += 1)
        for (double j = 3; j < order; j += 1) {
            mat_jerk((int)i,(int)j) = i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) * std::pow(ts, i + j - 5) / (i + j - 5);
        }

        jerk += (cxv.transpose() * mat_jerk * cxv)(0, 0);
        jerk += (cyv.transpose() * mat_jerk * cyv)(0, 0);
        jerk += (czv.transpose() * mat_jerk * czv)(0, 0);
    }

    return jerk;
}

void PolynomialTraj::getMeanAndMaxVel(double& mean_v, double& max_v)
{
    int num = 0;
    mean_v = 0.0, max_v = -1.0;
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

      /* coef of vel */
      for (int i = 0; i < order - 1; ++i) {
        vx(i) = double(i + 1) * cxs[s][order - 2 - i];
        vy(i) = double(i + 1) * cys[s][order - 2 - i];
        vz(i) = double(i + 1) * czs[s][order - 2 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts) {
        Eigen::VectorXd tv(order - 1);
        for (int i = 0; i < order - 1; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d vel;
        vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
        double vn = vel.norm();
        mean_v += vn;
        if (vn > max_v) max_v = vn;
        ++num;

        eval_t += 0.01;
      }
    }

    mean_v = mean_v / double(num);
}

void PolynomialTraj::getMeanAndMaxAcc(double& mean_a, double& max_a)
{
    int num = 0;
    mean_a = 0.0, max_a = -1.0;
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

      /* coef of acc */
      for (int i = 0; i < order - 2; ++i) {
        ax(i) = double((i + 2) * (i + 1)) * cxs[s][order - 3 - i];
        ay(i) = double((i + 2) * (i + 1)) * cys[s][order - 3 - i];
        az(i) = double((i + 2) * (i + 1)) * czs[s][order - 3 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts) {
        Eigen::VectorXd tv(order - 2);
        for (int i = 0; i < order - 2; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d acc;
        acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
        double an = acc.norm();
        mean_a += an;
        if (an > max_a) max_a = an;
        ++num;

        eval_t += 0.01;
      }
    }

    mean_a = mean_a / double(num);
}

PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd& Pos, const Eigen::Vector3d& start_vel,
                                   const Eigen::Vector3d& end_vel, const Eigen::Vector3d& start_acc,
                                   const Eigen::Vector3d& end_acc, const Eigen::VectorXd& Time){
    int seg_num = Time.size();
    Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);
    Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);

    int num_f, num_p;  // number of fixed and free variables
    int num_d;         // number of all segments' derivatives

    const static auto Factorial = [](int x) {
        int fac = 1;
        for (int i = x; i > 0; i--)
        fac = fac * i;
        return fac;
    };

    /* ---------- end point derivative ---------- */
    Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
    Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
    Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

    for (int k = 0; k < seg_num; k++) {
        /* position to derivative */
        Dx(k * 6) = Pos(k, 0);
        Dx(k * 6 + 1) = Pos(k + 1, 0);
        Dy(k * 6) = Pos(k, 1);
        Dy(k * 6 + 1) = Pos(k + 1, 1);
        Dz(k * 6) = Pos(k, 2);
        Dz(k * 6 + 1) = Pos(k + 1, 2);

        if (k == 0) {
        Dx(k * 6 + 2) = start_vel(0);
        Dy(k * 6 + 2) = start_vel(1);
        Dz(k * 6 + 2) = start_vel(2);

        Dx(k * 6 + 4) = start_acc(0);
        Dy(k * 6 + 4) = start_acc(1);
        Dz(k * 6 + 4) = start_acc(2);
        } else if (k == seg_num - 1) {
        Dx(k * 6 + 3) = end_vel(0);
        Dy(k * 6 + 3) = end_vel(1);
        Dz(k * 6 + 3) = end_vel(2);

        Dx(k * 6 + 5) = end_acc(0);
        Dy(k * 6 + 5) = end_acc(1);
        Dz(k * 6 + 5) = end_acc(2);
        }
    }

    /* ---------- Mapping Matrix A ---------- */
    Eigen::MatrixXd Ab;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

    for (int k = 0; k < seg_num; k++) {
        Ab = Eigen::MatrixXd::Zero(6, 6);
        for (int i = 0; i < 3; i++) {
        Ab(2 * i, i) = Factorial(i);
        for (int j = i; j < 6; j++)
            Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
        }
        A.block(k * 6, k * 6, 6, 6) = Ab;
    }

    /* ---------- Produce Selection Matrix C' ---------- */
    Eigen::MatrixXd Ct, C;

    num_f = 2 * seg_num + 4;  // 3 + 3 + (seg_num - 1) * 2 = 2m + 4
    num_p = 2 * seg_num - 2;  //(seg_num - 1) * 2 = 2m - 2
    num_d = 6 * seg_num;
    Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
    Ct(0, 0) = 1;
    Ct(2, 1) = 1;
    Ct(4, 2) = 1;  // stack the start point
    Ct(1, 3) = 1;
    Ct(3, 2 * seg_num + 4) = 1;
    Ct(5, 2 * seg_num + 5) = 1;

    Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
    Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1;  // Stack the end point
    Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
    Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1;  // Stack the end point
    Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
    Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1;  // Stack the end point

    for (int j = 2; j < seg_num; j++) {
        Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
        Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
        Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
        Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
        Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
        Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
    }

    C = Ct.transpose();

    Eigen::VectorXd Dx1 = C * Dx;
    Eigen::VectorXd Dy1 = C * Dy;
    Eigen::VectorXd Dz1 = C * Dz;

    /* ---------- minimum snap matrix ---------- */
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

    for (int k = 0; k < seg_num; k++) {
        for (int i = 3; i < 6; i++) {
        for (int j = 3; j < 6; j++) {
            Q(k * 6 + i, k * 6 + j) =
                i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
        }
        }
    }

    /* ---------- R matrix ---------- */
    Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

    Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

    Dxf = Dx1.segment(0, 2 * seg_num + 4);
    Dyf = Dy1.segment(0, 2 * seg_num + 4);
    Dzf = Dz1.segment(0, 2 * seg_num + 4);

    Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
    Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
    Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
    Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

    Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
    Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
    Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
    Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

    /* ---------- close form solution ---------- */

    Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
    Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
    Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
    Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

    Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
    Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
    Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

    Px = (A.inverse() * Ct) * Dx1;
    Py = (A.inverse() * Ct) * Dy1;
    Pz = (A.inverse() * Ct) * Dz1;

    for (int i = 0; i < seg_num; i++) {
        poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
        poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
        poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
    }

    /* ---------- use polynomials ---------- */
    PolynomialTraj poly_traj;
    for (int i = 0; i < poly_coeff.rows(); ++i) {
        vector<double> cx(6), cy(6), cz(6);
        for (int j = 0; j < 6; ++j) {
        cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
        }
        reverse(cx.begin(), cx.end());
        reverse(cy.begin(), cy.end());
        reverse(cz.begin(), cz.end());
        double ts = Time(i);
        poly_traj.addSegment(cx, cy, cz, ts);
    }

    return poly_traj;        
}

PolynomialTraj PolynomialTraj::fastLine4deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc,
                            double max_jerk) {
    Eigen::Vector3d disp = end - start;
    double len = disp.norm();
    Eigen::Vector3d dir = disp.normalized();

    // get scale vector
    int max_id = -1;
    double max_dist = -1.0;
    for (int i = 0; i < 3; ++i) {
        if (fabs(disp(i)) > max_dist) {
        max_dist = disp(i);
        max_id = i;
        }
    }
    Eigen::Vector3d scale_vec = disp / max_dist;

    PolynomialTraj poly_traj;
    vector<double> cx(6), cy(6), cz(6), zero(6);
    for (int i = 0; i < 6; ++i)
        zero[i] = 0.0;

    Eigen::Vector3d p0 = start;
    Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d a0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d j0 = max_jerk * scale_vec;

    /* first segment */
    cx = cy = cz = zero;
    double t1 = max_acc / max_jerk;

    cx[5] = p0(0);
    cy[5] = p0(1);
    cz[5] = p0(2);

    cx[4] = v0(0);
    cy[4] = v0(1);
    cz[4] = v0(2);

    cx[3] = (1 / 2.0) * a0(0);
    cy[3] = (1 / 2.0) * a0(1);
    cz[3] = (1 / 2.0) * a0(2);

    cx[2] = (1 / 6.0) * j0(0);
    cy[2] = (1 / 6.0) * j0(1);
    cz[2] = (1 / 6.0) * j0(2);

    poly_traj.addSegment(cx, cy, cz, t1);

    Eigen::Vector3d p1 = p0 + v0 * t1 + (1 / 2.0) * a0 * pow(t1, 2) + (1 / 6.0) * j0 * pow(t1, 3);
    Eigen::Vector3d v1 = v0 + a0 * t1 + (1 / 2.0) * j0 * pow(t1, 2);
    Eigen::Vector3d a1 = a0 + j0 * t1;
    Eigen::Vector3d j1 = Eigen::Vector3d::Zero();

    /* second segment */
    cx = cy = cz = zero;
    double t2 = max_vel / max_acc - t1;

    cx[5] = p1(0);
    cy[5] = p1(1);
    cz[5] = p1(2);

    cx[4] = v1(0);
    cy[4] = v1(1);
    cz[4] = v1(2);

    cx[3] = (1 / 2.0) * a1(0);
    cy[3] = (1 / 2.0) * a1(1);
    cz[3] = (1 / 2.0) * a1(2);

    cx[2] = (1 / 6.0) * j1(0);
    cy[2] = (1 / 6.0) * j1(1);
    cz[2] = (1 / 6.0) * j1(2);

    poly_traj.addSegment(cx, cy, cz, t2);

    Eigen::Vector3d p2 = p1 + v1 * t2 + (1 / 2.0) * a1 * pow(t2, 2) + (1 / 6.0) * j1 * pow(t2, 3);
    Eigen::Vector3d v2 = v1 + a1 * t2 + (1 / 2.0) * j1 * pow(t2, 2);
    Eigen::Vector3d a2 = a1 + j1 * t2;
    Eigen::Vector3d j2 = -max_jerk * scale_vec;

    /* third segment */
    cx = cy = cz = zero;
    double t3 = t1;

    cx[5] = p2(0);
    cy[5] = p2(1);
    cz[5] = p2(2);

    cx[4] = v2(0);
    cy[4] = v2(1);
    cz[4] = v2(2);

    cx[3] = (1 / 2.0) * a2(0);
    cy[3] = (1 / 2.0) * a2(1);
    cz[3] = (1 / 2.0) * a2(2);

    cx[2] = (1 / 6.0) * j2(0);
    cy[2] = (1 / 6.0) * j2(1);
    cz[2] = (1 / 6.0) * j2(2);

    poly_traj.addSegment(cx, cy, cz, t3);

    Eigen::Vector3d p3 = p2 + v2 * t3 + (1 / 2.0) * a2 * pow(t3, 2) + (1 / 6.0) * j2 * pow(t3, 3);
    Eigen::Vector3d v3 = v2 + a2 * t3 + (1 / 2.0) * j2 * pow(t3, 2);
    Eigen::Vector3d a3 = a2 + j2 * t3;
    Eigen::Vector3d j3 = Eigen::Vector3d::Zero();

    /* fourth segment */
    cx = cy = cz = zero;
    double t4 = max_dist / max_vel - 2 * t1 - t2;

    cx[5] = p3(0);
    cy[5] = p3(1);
    cz[5] = p3(2);

    cx[4] = v3(0);
    cy[4] = v3(1);
    cz[4] = v3(2);

    cx[3] = (1 / 2.0) * a3(0);
    cy[3] = (1 / 2.0) * a3(1);
    cz[3] = (1 / 2.0) * a3(2);

    cx[2] = (1 / 6.0) * j3(0);
    cy[2] = (1 / 6.0) * j3(1);
    cz[2] = (1 / 6.0) * j3(2);

    poly_traj.addSegment(cx, cy, cz, t4);

    Eigen::Vector3d p4 = p3 + v3 * t4 + (1 / 2.0) * a3 * pow(t4, 2) + (1 / 6.0) * j3 * pow(t4, 3);
    Eigen::Vector3d v4 = v3 + a3 * t4 + (1 / 2.0) * j3 * pow(t4, 2);
    Eigen::Vector3d a4 = a3 + j3 * t4;
    Eigen::Vector3d j4 = -max_jerk * scale_vec;

    /* fifth segment */
    cx = cy = cz = zero;
    double t5 = t1;

    cx[5] = p4(0);
    cy[5] = p4(1);
    cz[5] = p4(2);

    cx[4] = v4(0);
    cy[4] = v4(1);
    cz[4] = v4(2);

    cx[3] = (1 / 2.0) * a4(0);
    cy[3] = (1 / 2.0) * a4(1);
    cz[3] = (1 / 2.0) * a4(2);

    cx[2] = (1 / 6.0) * j4(0);
    cy[2] = (1 / 6.0) * j4(1);
    cz[2] = (1 / 6.0) * j4(2);

    poly_traj.addSegment(cx, cy, cz, t5);

    Eigen::Vector3d p5 = p4 + v4 * t5 + (1 / 2.0) * a4 * pow(t5, 2) + (1 / 6.0) * j4 * pow(t5, 3);
    Eigen::Vector3d v5 = v4 + a4 * t5 + (1 / 2.0) * j4 * pow(t5, 2);
    Eigen::Vector3d a5 = a4 + j4 * t5;
    Eigen::Vector3d j5 = Eigen::Vector3d::Zero();

    /* sixth segment */
    cx = cy = cz = zero;
    double t6 = t2;

    cx[5] = p5(0);
    cy[5] = p5(1);
    cz[5] = p5(2);

    cx[4] = v5(0);
    cy[4] = v5(1);
    cz[4] = v5(2);

    cx[3] = (1 / 2.0) * a5(0);
    cy[3] = (1 / 2.0) * a5(1);
    cz[3] = (1 / 2.0) * a5(2);

    cx[2] = (1 / 6.0) * j5(0);
    cy[2] = (1 / 6.0) * j5(1);
    cz[2] = (1 / 6.0) * j5(2);

    poly_traj.addSegment(cx, cy, cz, t6);

    Eigen::Vector3d p6 = p5 + v5 * t6 + (1 / 2.0) * a5 * pow(t6, 2) + (1 / 6.0) * j5 * pow(t6, 3);
    Eigen::Vector3d v6 = v5 + a5 * t6 + (1 / 2.0) * j5 * pow(t6, 2);
    Eigen::Vector3d a6 = a5 + j5 * t6;
    Eigen::Vector3d j6 = max_jerk * scale_vec;

    /* seventh (last) segment */
    cx = cy = cz = zero;
    double t7 = t1;

    cx[5] = p6(0);
    cy[5] = p6(1);
    cz[5] = p6(2);

    cx[4] = v6(0);
    cy[4] = v6(1);
    cz[4] = v6(2);

    cx[3] = (1 / 2.0) * a6(0);
    cy[3] = (1 / 2.0) * a6(1);
    cz[3] = (1 / 2.0) * a6(2);

    cx[2] = (1 / 6.0) * j6(0);
    cy[2] = (1 / 6.0) * j6(1);
    cz[2] = (1 / 6.0) * j6(2);

    poly_traj.addSegment(cx, cy, cz, t7);

    return poly_traj;
}

PolynomialTraj PolynomialTraj::fastLine3deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc) {
    Eigen::Vector3d disp = end - start;
    double len = disp.norm();
    Eigen::Vector3d dir = disp.normalized();

    // get scale vector
    double max_dist = std::max(disp(0), disp(1));
    max_dist = std::max(max_dist, disp(2));
    Eigen::Vector3d scale_vec = disp / max_dist;

    // get time of 3 segments
    // head and tail segment time
    double t_ht = max_vel / max_acc;

    // middle segment time
    double len_ht = 0.5 * (max_acc * scale_vec.norm()) * t_ht * t_ht;
    double t_m = (len - 2 * len_ht) / (max_vel * scale_vec.norm());

    // two middle waypoints
    Eigen::Vector3d mpt1 = start + dir * len_ht;
    Eigen::Vector3d mpt2 = end - dir * len_ht;

    // 3 segments of traj
    PolynomialTraj poly_traj;
    vector<double> cx(6), cy(6), cz(6), zero(6);
    for (int i = 0; i < 6; ++i)
        zero[i] = 0.0;

    // head segment
    cx = cy = cz = zero;

    cx[5] = start(0);  // p0
    cy[5] = start(1);
    cz[5] = start(2);

    cx[4] = 0.0;  // v0
    cy[4] = 0.0;
    cz[4] = 0.0;

    cx[3] = 0.5 * max_acc * scale_vec(0);  // a0
    cy[3] = 0.5 * max_acc * scale_vec(1);
    cz[3] = 0.5 * max_acc * scale_vec(2);

    poly_traj.addSegment(cx, cy, cz, t_ht);

    // middle segment
    cx = cy = cz = zero;

    cx[5] = mpt1(0);  // p0
    cy[5] = mpt1(1);
    cz[5] = mpt1(2);

    cx[4] = max_vel * scale_vec(0);  // v0
    cy[4] = max_vel * scale_vec(1);
    cz[4] = max_vel * scale_vec(2);

    cx[3] = 0.0;  // a0
    cy[3] = 0.0;
    cz[3] = 0.0;

    poly_traj.addSegment(cx, cy, cz, t_m);

    // tail segment
    cx = cy = cz = zero;

    cx[5] = mpt2(0);  // p0
    cy[5] = mpt2(1);
    cz[5] = mpt2(2);

    cx[4] = max_vel * scale_vec(0);  // v0
    cy[4] = max_vel * scale_vec(1);
    cz[4] = max_vel * scale_vec(2);

    cx[3] = -0.5 * max_acc * scale_vec(0);  // a0
    cy[3] = -0.5 * max_acc * scale_vec(1);
    cz[3] = -0.5 * max_acc * scale_vec(2);

    poly_traj.addSegment(cx, cy, cz, t_ht);

    return poly_traj;
}