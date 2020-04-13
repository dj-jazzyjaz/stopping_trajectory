#include <stopping_trajectory/StoppingTrajectory.h>

namespace planner
{
namespace gu = geometry_utils;
namespace vu = vector_utils;

/**
 * @brief Returns polynomial coefficients that solve all higher-order derivatives going to 0. No 
 * goal position. Parameters are initial state and trajectory time. 
 * */
std::vector<double> StoppingTrajectory::getCoefficients(double pos, double vel, double acc, double jerk, double snap, double t)
{
  double a1 = 5 * pow(t, 4);
  double a2 = 6 * pow(t, 5);
  double a3 = 7 * pow(t, 6);
  double a4 = 8 * pow(t, 7);
  double a5 = 20 * pow(t, 3);
  double a6 = 30 * pow(t, 4);
  double a7 = 42 * pow(t, 5);
  double a8 = 56 * pow(t, 6);
  double a9 = 60 * pow(t, 2);
  double a10 = 120 * pow(t, 3);
  double a11 = 210 * pow(t, 4);
  double a12 = 336 * pow(t, 5);
  double a13 = 120 * t;
  double a14 = 360 * pow(t, 2);
  double a15 = 840 * pow(t, 3);
  double a16 = 1680 * pow(t, 4);
  double b1 = vel + acc * t + (jerk / 2) * pow(t, 2) + (snap / 6) * pow(t, 3);
  double b2 = acc + jerk * t + 0.5 * snap * pow(t, 2);
  double b3 = jerk + snap * t;
  double b4 = snap;

  //Solve for x such that Ax = B
  gu::Mat44 A = gu::Mat44(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16);

  gu::Mat44 inv = A.inv();
  gu::Vec4 B = gu::Vec4(-b1, -b2, -b3, -b4);
  gu::Vec4 x = inv * B;
  double c5 = x.data[0];
  double c6 = x.data[1];
  double c7 = x.data[2];
  double c8 = x.data[3];

  std::vector<double> all_coeffs{pos, vel, acc / 2, jerk / 6, snap / 24, c5, c6, c7, c8};
  return all_coeffs;
}

/** 
 * @brief Get coefficients that solve all higher-order derivatives going to 0 with desired goal 
 * position. Parameters are initial state and trajectory time. 
 * */
std::vector<double> StoppingTrajectory::getCoefficientsWithGoalPos(double pos, double vel, 
  double acc, double jerk, double goal_pos, double t)
{
  double a1 = pow(t, 4);
  double a2 = pow(t, 5);
  double a3 = pow(t, 6);
  double a4 = pow(t, 7);
  double a5 = pow(t, 8);
  double a6 = 4 * pow(t, 3);
  double a7 = 5 * pow(t, 4);
  double a8 = 6 * pow(t, 5);
  double a9 = 7 * pow(t, 6);
  double a10 = 8 * pow(t, 7);
  double a11 = 12 * pow(t, 2);
  double a12 = 20 * pow(t, 3);
  double a13 = 30 * pow(t, 4);
  double a14 = 42 * pow(t, 5);
  double a15 = 56 * pow(t, 6);
  double a16 = 24 * t;
  double a17 = 60 * pow(t, 2);
  double a18 = 120 * pow(t, 3);
  double a19 = 210 * pow(t, 4);
  double a20 = 336 * pow(t, 5);
  double a21 = 24;
  double a22 = 120 * t;
  double a23 = 360 * pow(t, 2);
  double a24 = 840 * pow(t, 3);
  double a25 = 1680 * pow(t, 4);
  double b1 = -goal_pos + pos + vel * t + (acc / 2) * pow(t, 2) + (jerk / 6) * pow(t, 3);
  double b2 = vel + acc * t + (jerk / 2) * pow(t, 2);
  double b3 = acc + jerk * t;
  double b4 = jerk;
  double b5 = 0;

  //Solve for x such that Ax = B
  boost::array<double, 25> a = {{a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, 
                                a16, a17, a18, a19, a20, a21, a22, a23, a24, a25}};
  gu::MatrixNxNBase<double, 5> A = gu::MatrixNxNBase<double, 5>(a);
  gu::MatrixNxNBase<double, 5> inv = A.inv();

  boost::array<double, 5> b = {{-b1, -b2, -b3, -b4, -b5}};
  gu::VectorNBase<double, 5> B = gu::VectorNBase<double, 5>(b);
  gu::VectorNBase<double, 5> x = inv * B;
  double c4 = x.data[0];
  double c5 = x.data[1];
  double c6 = x.data[2];
  double c7 = x.data[3];
  double c8 = x.data[4];

  std::vector<double> all_coeffs{pos, vel, acc / 2, jerk / 6, c4, c5, c6, c7, c8};
  return all_coeffs;
}

/**
 * @brief Caclulates the polynomial coefficients for x, y, z and yaw that make HODs go to 0, 
 * no goal position.
 * 
 * @param coefficients [out] Vector in which to write the computed coefficients
 * */
void StoppingTrajectory::getPolynomials(const state_t& state, 
  ros::Duration duration, std::vector<std::vector<double>>& coefficients)
{
  double t_stop = (double)duration.sec;
  for (int i = 0; i < 3; i++)
  {
    coefficients.push_back(getCoefficients(state.pos.data[i], state.vel.data[i], state.acc.data[i], 
                                          state.jerk.data[i], state.snap.data[i], t_stop));
  }  
  coefficients.push_back(getCoefficients(state.yaw(), state.dyaw, state.d2yaw, state.d3yaw, 0, t_stop));
}

/**
 * @brief Caclulates the polynomial coefficients for x, y, z and yaw that make HODs go to 0, with 
 * goal position.
 * 
 * @param coefficients [out] Vector in which to write the computed coefficients
 * */
void StoppingTrajectory::getPolynomialsWithPos(const state_t& state, 
  gu::Vec3 goal_pos, double t_stop, std::vector<std::vector<double>>& coefficients)
{
  for (int i = 0; i < 3; i++)
  {
    coefficients.push_back(getCoefficientsWithGoalPos(state.pos.data[i], state.vel.data[i], 
      state.acc.data[i], state.jerk.data[i], goal_pos.data[i], t_stop));
  }
  coefficients.push_back(getCoefficients(state.yaw(), state.dyaw, state.d2yaw, state.d3yaw, 0, t_stop));
}

/**
 * @brief Given polynomial coefficients, populates a vector of state_t by evaluating the polynomial 
 * at each timestep for each dimension
 * 
 * @param traj_length [in] The number of waypoints in the interpolated trajectory. Should be equal 
 * to t_stop/step
 * @param traj [out] Vector in which to store computed trajectory waypoints
 *
 * */
void StoppingTrajectory::populateTrajectory(const ros::Time &reference_time, 
  int traj_length, const std::vector<std::vector<double>>& coefficients, ros::Duration duration, 
  double step, std::vector<state_t> &traj)
{
  ros::Time start_time = reference_time;
  Polynomial poly = Polynomial(start_time, duration, coefficients);
  traj.resize(traj_length);
  gu::Vec4 vec;

  for (int i = 0; i < traj_length; i++)
  {
    // Get position at time step i 
    vec = poly.getReference(i * step, 0);
    traj[i].pos = gu::Vec3(vec.data[0], vec.data[1], vec.data[2]);
    traj[i].yaw(vec.data[3]);
    traj[i].rot = gu::Rot3(0, 0, vec.data[3]);

    // Get velocity
    vec = poly.getReference(i * step, 1);
    traj[i].vel = gu::Vec3(vec.data[0], vec.data[1], vec.data[2]);
    traj[i].dyaw = vec.data[3];
    traj[i].ang = gu::Vec3(0, 0, vec.data[3]);

    // Get acceleration
    vec = poly.getReference(i * step, 2);
    traj[i].acc = gu::Vec3(vec.data[0], vec.data[1], vec.data[2]);
    traj[i].d2yaw = vec.data[3];
    traj[i].angacc = gu::Vec3(0, 0, vec.data[3]);

    // Get jerk
    vec = poly.getReference(i * step, 3);
    traj[i].jerk = gu::Vec3(vec.data[0], vec.data[1], vec.data[2]);
    traj[i].d3yaw = vec.data[3];

    // Get snap
    vec = poly.getReference(i * step, 4);
    traj[i].snap = gu::Vec3(vec.data[0], vec.data[1], vec.data[2]);
  }
}

/**
 * @brief Checks each point along a trajectory for collisions, using KDtree.
 * Returns true if no collisions, false if collision detected.
 * */
bool StoppingTrajectory::checkTrajectoryCollision(const std::vector<state_t> &traj)
{
  float &r = collision_radius_;
  bool isSafe = collision_avoidance_->checkWaypointsSafeKDTree(traj, r);
  return isSafe;
}

/**
 * @brief Checks each point along a trajectory for collisions, using the global map.
 * Returns true if no collisions, false if collision detected.
 * */
bool StoppingTrajectory::checkTrajectoryCollisionGlobal(const std::vector<state_t> &traj)
{
  float nearest_dist;
  for (uint i = 0; i < traj.size(); i++)
  {
    float dist = global_map_.find_nearest_neighbor(traj[i].pos(0), traj[i].pos(1), traj[i].pos(2));
    if (dist < collision_radius_)
    {
      // std::cout << "Collision detected along trajectory, within " << dist << std::endl;
      return false;
    }

    // Keep track of nearest obstacle the trajectory approaches
    if (i == 0 || dist < nearest_dist)
    {
      nearest_dist = dist;
    }
  }

  // std::cout << "Trajectory is collision free, nearest obstacle:" << nearest_dist << std::endl;
  return true;
}

/**
 * @brief Checks if the trajectory's acceleration is within the threshold for every point along the 
 * trajectory in x, y and z.
 * */
bool StoppingTrajectory::checkAccelThreshold(const std::vector<std::vector<double>>& coefficients, 
  ros::Duration &duration, double acc_threshold)
{
  double step = 0.1;
  double duration_time = duration.toSec();
  ros::Time start_time(0.0);

  Polynomial poly = Polynomial(start_time, duration, coefficients);

  std::vector<gu::Vec4> evaluated_poly;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;

  for (double t = 0; t < duration_time; t += step)
  {
    // Evaluate the acceleration (2nd derivative) at each time step
    gu::Vec4 vec = poly.getReference(t * step, 2);
    evaluated_poly.push_back(vec);

    // For logging purposes
    x.push_back(vec.data[0]);
    y.push_back(vec.data[1]);
    z.push_back(vec.data[2]);
  }

  // Remove all points that are within the threshold
  vu::RemoveAllIf(evaluated_poly,
                  [acc_threshold](gu::Vec4 point) { 
                    return fabs(point.data[0]) < acc_threshold && 
                    fabs(point.data[1]) < acc_threshold && 
                    fabs(point.data[2]) < acc_threshold; });

  // Print max and min values
  // std::cout << "Acceleration: x: (" << vu::Min(x) << "," << vu::Max(x) << ") y: (" << vu::Min(y) 
  // << "," << vu::Max(y) << ") z: (" << vu::Min(z) << "," << vu::Max(z) << ")" << std::endl;

  // Return true if no points above threshold
  return (evaluated_poly.size() == 0);
}

/**
 * @brief Checks if the trajectory's higher-order derivatives are within the thresholds for every 
 * point along the trajectory in x, y and z.
 * 
 * Coefficientss in decreasing order (c0, c1, c2, c3 ... c9). 
 * Thresholds in decreasing order (vel, acc, jerk, snap). t is time
 * */
bool StoppingTrajectory::checkTrajectoryHOD(const std::vector<std::vector<double>>& coefficients, 
  ros::Duration &duration, std::vector<double> thresholds)
{
  double step = 0.1;
  double duration_time = duration.toSec();
  ros::Time start_time(0.0);

  Polynomial poly = Polynomial(start_time, duration, coefficients);
  for (int i = 0; i < 4; i++)
  {
    std::vector<gu::Vec4> evaluated_poly;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

    for (double t = 0; t < duration_time; t += step)
    {
      // Evaluate the i+1th derivative of poly trajectory at each time step
      gu::Vec4 vec = poly.getReference(t * step, i + 1);
      evaluated_poly.push_back(vec);

      // For logging purposes
      x.push_back(vec.data[0]);
      y.push_back(vec.data[1]);
      z.push_back(vec.data[2]);
    }

    // Remove all points that are within the threshold
    vu::RemoveAllIf(evaluated_poly,
                    [thresholds, i](gu::Vec4 point) { 
                      return fabs(point.data[0]) < thresholds[i] && 
                      fabs(point.data[1]) < thresholds[i] && 
                      fabs(point.data[2]) < thresholds[i]; });

    //Print max and min values
    /*std::cout << "Order: " << i << "x: (" << vu::Min(x) << "," << vu::Max(x) << ") y: (" 
    << vu::Min(y) << "," << vu::Max(y) << ") z: (" << vu::Min(z) << "," << vu::Max(z) << ")" 
    << std::endl; */

    if (evaluated_poly.size() > 0)
      return false;
  }
  return true;
}

/**
 * @brief Caclulates the duration of a trajectory from state to goal_pos using an heuristic
 * WIP: Find a better heuristic to use. Current heuristic is too unreliable. 
 * */
double StoppingTrajectory::getTrajectoryDuration(state_t state, gu::Vec3 goal_pos)
{
  std::vector<double> T; // Candidate durations, will take max
  double MAX_ACC = 1;    // Maximum allowed acceleration in any dimension
  // For each dimension x, y, z
  for (int i = 0; i < 3; i++)
  {
    // Time for velocity to reach 0
    double del_pos = goal_pos.data[i] - state.pos.data[i];
    T.push_back(std::fabs(del_pos / state.vel.data[i]));

    // time for acceleration to reach 0
    double del_vel = state.vel.data[i];
    // T.push_back(std::fabs(del_vel/state.acc.data[i]));
    T.push_back(std::fabs(del_vel / MAX_ACC));

    // time for jerk to reach 0
    //double del_acc = state.acc.data[i];
    //T.push_back(std::fabs(del_acc/state.jerk.data[i]));
  }

  // Print T values
  for (uint i = 0; i < T.size(); i++)
  {
    std::cout << "T: " << T[i] << std::endl;
  }
  std::cout << "Max T: " << vu::Max(T) << std::endl;
  return std::max(0.5, vu::Max(T));
}
} // namespace planner
