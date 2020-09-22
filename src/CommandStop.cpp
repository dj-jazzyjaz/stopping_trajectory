#include <stopping_trajectory/StoppingTrajectory.h>

namespace planner
{

namespace gu = geometry_utils;
namespace pu = parameter_utils;
namespace gr = gu::ros;

/**
 * @brief Generates a stopping trajectory is vehicle is within stop distance
 * */
void StoppingTrajectory::stopWithinDistance(const ros::TimerEvent &)
{
  ros::Time ref_time;
  state_t ref_state;
  float lookahead = 0;
  if (!getNextReference(ref_state, ref_time, lookahead))
  {
    ROS_ERROR("[Stop within distance] reference not found; escape points not generated");
  }

  if (ref_state.vel.norm() < compute_thresh_) return;

  traj_.markers.clear();

  float dist_from_obstacle = global_map_.find_nearest_neighbor(ref_state.pos.x(), ref_state.pos.y(), ref_state.pos.z());
  if (dist_from_obstacle < stopping_radius_)
  {
    std::cout << "Generate escape path, dist=" << dist_from_obstacle << std::endl;
    generateCollisionFreeWaypoints(ref_state, ref_time);
  }
  else
  {
    std::cout << "Not within obstacle radius, dist=" << dist_from_obstacle << std::endl;
  }
}

/**
 * @brief Returns fraction of free points to total points
 * */
float StoppingTrajectory::freePointsRatio(state_t ref_state)
{
  int num_free = 0;
  std::vector<gu::Vec3> escape_points;
  generateGridPoints(ref_state.pos, ref_state.vel, ref_state.yaw(), escape_points);
  if (escape_points.size() == 0)
  return 1;
  for (uint i = 0; i < escape_points.size(); i++)
  {
    float dist_from_obstacle = global_map_.find_nearest_neighbor(escape_points[i](0), escape_points[i](1), escape_points[i](2));
    if (dist_from_obstacle > collision_radius_)
    {
      num_free++;
    }
  }
  return (float)num_free / (float)escape_points.size();
}

/**
 * @brief Generates a stopping trajectory if vehicle meets criteria for stopping, 
 * including the distance of the nearest obstacle relative to heading, the ratio of free escape 
 * points, and the derivative of free points ratio.
 * */
void StoppingTrajectory::commandStop(const ros::TimerEvent &)
{
  if (!flagEnabledQ("teleop")) return;
  clock_t start = std::clock();

  ros::Time ref_time;
  state_t ref_state;
  float lookahead = 0;
  if (!getNextReference(ref_state, ref_time, lookahead))
  {
    ROS_ERROR("[Command Stop] Reference not found; escape points not generated");
  }

  if (ref_state.vel.norm() < compute_thresh_) return;
  traj_.markers.clear();

  // Get distance and location of nearest obstacle
  float *neigh_point = new float[3];
  float dist_from_obstacle = global_map_.find_nearest_neighbor(ref_state.pos.x(), ref_state.pos.y(), ref_state.pos.z(), neigh_point);

  // get the angle between vector from position->obstacle relative to current yaw
  float offset_x = neigh_point[0] - ref_state.pos.x();
  float offset_y = neigh_point[1] - ref_state.pos.y();
  
  // Since x is the direction of vehicle heading when yaw is 0, take atan(y, x)
  // Take absolute value and min between 2*pi-angle and angle to get true offset
  float offset_angle = gu::math::atan2(offset_y, offset_x) - (float)ref_state.yaw();
  offset_angle = gu::math::fmin(fabs(offset_angle), (float)(2*3.142 - fabs(offset_angle)));
 
  // Calcuate stop cost as a weighted sum of distance, angle and velocity
  float stop_cost = offset_angle + stop_dist_weight_ * dist_from_obstacle 
    - stop_vel_weight_ * ref_state.vel.norm() - stop_bias_; 
  
  double compute_duration = (std::clock() - start ) / (double) CLOCKS_PER_SEC;

  ROS_INFO("[Command Stop] Cost: %f. Angle: %f. Dist: %f; %f. Vel: %f; %f. Time: %f sec \n", 
    stop_cost, 
    offset_angle, 
    dist_from_obstacle, 
    stop_dist_weight_ * dist_from_obstacle,
    ref_state.vel.norm(),
    stop_vel_weight_ * ref_state.vel.norm(),
    compute_duration);
  
  if (stop_cost < 0) {
    std::cout << "======== Generate stopping command ========" << std::endl;
    generateCollisionFreeWaypoints(ref_state, ref_time);
    std_msgs::String event_msg;
    event_msg.data = "HoverEvent";
    event_pub_.publish(event_msg);
  }
  

  delete[] neigh_point;
}

} // namespace planner