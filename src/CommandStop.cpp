#include <stopping_trajectory/StoppingTrajectory.h>

namespace planner
{

namespace gu = geometry_utils;
namespace pu = parameter_utils;
namespace gr = gu::ros;
namespace vu = vector_utils;

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
 * @brief Generates a stopping trajectory if vehicle meets criteria for stopping
 * */
void StoppingTrajectory::commandStop(const ros::TimerEvent &, float stopping_trajectory_duration)
{
  if (!flagEnabledQ("teleop")) return;
  clock_t start = std::clock();

  // Clear visualization
  visualization_msgs::MarkerArray marker_array;
  stop_traj_vis_pub_.publish(marker_array);

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
  
  // Stop is distance is too small
  if (dist_from_obstacle < stopping_radius_)
  {
    std::cout << "======== Generate stopping command ========" << std::endl;
    ROS_INFO("[Command Stop] Distance from obstacle: %f, Stoping radius: %f", dist_from_obstacle, stopping_radius_);
    generateCollisionFreeWaypoints(ref_state, ref_time, stopping_trajectory_duration);
    std_msgs::String event_msg;
    event_msg.data = "HoverEvent";
    event_pub_.publish(event_msg);
  }

  // Get points in neighbors
  float neighbor_radius = 3.0f;
  std::vector<pcl::PointXYZ> neighbors; 
  global_map_.check_neighbor_in_radius(ref_state.pos.x(), ref_state.pos.y(), ref_state.pos.z(), neighbor_radius, &neighbors);
  std::cout << "Num neighbors" << neighbors.size() << std::endl;

  
  if (neighbors.size() == 0) return;

  std::vector<gu::Vec3> relevant_neighbors;
  std::vector<float> neighbor_costs;
  double offset_angle;
  double vel_norm = ref_state.vel.norm();
  double stop_cost;

  // Project points vector onto velocity vector. Throw away those with a negative dot product
  for(auto neighbor : neighbors) {
    gu::Vec3 curr_neighbor(neighbor.x, neighbor.y, neighbor.z);
    // Put the neighbor point in the robot velocity frame
    gu::Vec3 neighbor_offset = curr_neighbor - ref_state.pos;
    if(neighbor_offset.normalize().dot(ref_state.vel.normalize()) > 0) {
      relevant_neighbors.push_back(curr_neighbor);
      
      offset_angle = gu::math::acos(
        neighbor_offset.dot(ref_state.vel)/(neighbor_offset.norm() * ref_state.vel.norm())
      ); // Use the dot product cosine rule
      
      // Calculate the stop cost
      stop_cost = stop_angle_weight_ * offset_angle + stop_dist_weight_ * neighbor_offset.norm() 
        - stop_vel_weight_ * vel_norm - stop_bias_; 
      neighbor_costs.push_back((float)stop_cost);
    }
  }

  // Visualize the results
  std::vector<gu::Vec3> neighbors_sorted;
  std::vector<float> neighbor_costs_sorted;
  vu::ArgSort(relevant_neighbors, &neighbors_sorted, neighbor_costs, &neighbor_costs_sorted);
  if(relevant_neighbors.size() > 10) {
    relevant_neighbors = vu::Slice(neighbors_sorted, 0, 10);
    neighbor_costs = vu::Slice(neighbor_costs_sorted, 0, 10);
  } else {
    relevant_neighbors = neighbors_sorted;
    neighbor_costs = neighbor_costs_sorted;
  }
  
  int i = 0;
  for(auto n: relevant_neighbors) {
    std::cout << "Neighbor " << i << std::endl;
    n.print();
    gu::Vec3 neighbor_offset = n - ref_state.pos;
    neighbor_offset.print();
    neighbor_offset.normalize().print();
    std::cout << neighbor_offset.normalize().dot(ref_state.vel.normalize()) << ", " << neighbor_costs[i] << std::endl;
    i++;

  }

  std::cout << "Velocity" << std::endl;
  ref_state.vel.normalize().print();
 
  if(relevant_neighbors.size() > 0) visualizeNeighborhood(relevant_neighbors, neighbor_costs, ref_state.pos, ref_state.vel);

  if(relevant_neighbors.size() > 0 && *std::min_element(neighbor_costs.begin(), neighbor_costs.end()) < 0) {
    std::cout << "======== Generate stopping command ========" << std::endl;
    generateCollisionFreeWaypoints(ref_state, ref_time, stopping_trajectory_duration);
    std_msgs::String event_msg;
    event_msg.data = "HoverEvent";
    event_pub_.publish(event_msg);
  }

  return;
  /*
  // get the angle between vector from position->obstacle relative to current yaw
  float offset_x = neigh_point[0] - ref_state.pos.x();
  float offset_y = neigh_point[1] - ref_state.pos.y();
  float offset_z = neigh_point[2] - ref_state.pos.z();
  gu::Vec3 obstacle_offset(offset_x, offset_y, offset_z);
  
  float cosine_similarity = obstacle_offset.dot(ref_state.vel)/(obstacle_offset.norm() * ref_state.vel.norm());

  // Since x is the direction of vehicle heading when yaw is 0, take atan(y, x)
  // Take absolute value and min between 2*pi-angle and angle to get true offset
  float offset_angle = gu::math::atan2(offset_y, offset_x) - (float)ref_state.yaw();
  offset_angle = gu::math::fmin(fabs(offset_angle), (float)(2*3.142 - fabs(offset_angle)));
  
  offset_angle = gu::math::acos(cosine_similarity);
  
  // Calcuate stop cost as a weighted sum of distance, angle and velocity
  float stop_cost = stop_angle_weight_ * offset_angle + stop_dist_weight_ * dist_from_obstacle 
   - stop_vel_weight_ * ref_state.vel.norm() - stop_bias_; 
  
  gu::Vec3 diff = obstacle_offset - ref_state.vel;
  // float stop_cost = diff.norm() - 0.5 * ref_state.vel.norm();

  double compute_duration = (std::clock() - start ) / (double) CLOCKS_PER_SEC;

  ROS_INFO("[Command Stop] %f", stop_cost);
  ROS_INFO("[Command Stop Log] \n%f\n%f\n%f\n%f", stop_cost, dist_from_obstacle, gu::math::acos(cosine_similarity), ref_state.vel.norm());
  ROS_INFO("[Command Stop] Stop angle:%f * %f, Distance: %f * %f, Vel: %f * %f, Bias: %f",
    stop_angle_weight_,
     offset_angle,
    stop_dist_weight_,
    dist_from_obstacle, 
    stop_vel_weight_ , 
    ref_state.vel.norm(),
     stop_bias_
  );  
  /* ROS_INFO("[Command Stop] \nVel: [%f, %f, %f] \nObs: [%f, %f, %f]\nDot: %f", 
    ref_state.vel.x(),
    ref_state.vel.y(),
    ref_state.vel.z(),
    obstacle_offset.x(),
    obstacle_offset.y(),
    obstacle_offset.z(),
    stop_cost
  );
 */
  /* ROS_INFO("[Command Stop] Cost: %f = %f [a] + %f [d] - %f [v] - %f [b] \n Angle: %f \n Dist: %f \n Vel: %f \n Compute Time: %f sec \n", 
    stop_cost, 
    offset_angle * stop_angle_weight_,
    stop_dist_weight_ * dist_from_obstacle,
    stop_vel_weight_ * ref_state.vel.norm(),
    stop_bias_,
    offset_angle  * (180/3.14159), 
    dist_from_obstacle, 
    ref_state.vel.norm(),
    compute_duration);

  ROS_INFO("[Command Stop] Position: (%f, %f) \n Neighbor: (%f, %f) \n Offset: (%f, %f) \n Yaw: %f    Offset: %f      Diff:  %f",
    ref_state.pos.x(), ref_state.pos.y(),
    neigh_point[0], neigh_point[1],
    offset_x, offset_y, 
    ref_state.yaw() * (180/3.14159), 
    gu::math::atan2(offset_y, offset_x) * (180/3.14159), 
    offset_angle  * (180/3.14159)
  ); */
/*
  gu::Vec3 neighbor_pos(neigh_point[0], neigh_point[1], neigh_point[2]);
  // visualizeNearestObstacle(ref_state.pos, neighbor_pos, ref_state.yaw(), 0.5 * ref_state.vel);

  
  if (stop_cost < 0 ) {
    std::cout << "======== Generate stopping command ========" << std::endl;
    generateCollisionFreeWaypoints(ref_state, ref_time, stopping_trajectory_duration);
    std_msgs::String event_msg;
    event_msg.data = "HoverEvent";
    event_pub_.publish(event_msg);
  }
  

  delete[] neigh_point;
  */
}

} // namespace planner