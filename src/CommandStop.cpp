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

  float dist_from_obstacle = findNearestNeighbor(ref_state.pos, NULL);
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
  neighbors_vis_pub_.publish(marker_array);

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
  float dist_from_obstacle = findNearestNeighbor(ref_state.pos, neigh_point);
  
  // Stop is distance is too small
  if (dist_from_obstacle < stopping_radius_)
  {
    ROS_WARN("[CommandStop] Generate stopping command because stop distance is too small");;
    ROS_INFO("[CommandStop] Distance from obstacle: %f, Stopping radius: %f", dist_from_obstacle, stopping_radius_);
    generateCollisionFreeWaypoints(ref_state, ref_time, stopping_trajectory_duration);
    if (publish_hover_after_stop_)
    {
      std_msgs::String event_msg;
      event_msg.data = "HoverEvent";
      event_pub_.publish(event_msg);
    }
  }

  // Get points in neighbors
  double neighbor_radius = 3.0f;
  std::vector<pcl::PointXYZ> neighbors;
  queryRadiusNeighbors(ref_state.pos, neighbor_radius, &neighbors);
  if (verbose_) std::cout << "Num neighbors" << neighbors.size() << std::endl;


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
  relevant_neighbors = vu::Slice(neighbors_sorted, 0, 10);
  neighbor_costs = vu::Slice(neighbor_costs_sorted, 0, 10);
  
  // Print stuff 
  int i = 0;
  for(auto n: relevant_neighbors) {
    gu::Vec3 neighbor_offset = n - ref_state.pos;
    if (verbose_)
    {
      std::cout << "Neighbor " << i << std::endl;
      neighbor_offset.print();
      std::cout << neighbor_costs[i] << std::endl;
    }
    i++;
  }

  if (verbose_)
  {
    std::cout << "Velocity" << std::endl;
    ref_state.vel.normalize().print();
  }

//time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, vel, hasNeighbors, distance, offset_angle, cost, stopped, neigh_x, neigh_y, neigh_z" << std::endl;
  if (record_ && relevant_neighbors.size() > 0) {
    gu::Vec3 curr_neighbor = relevant_neighbors[0];
    gu::Vec3 neighbor_offset = curr_neighbor - ref_state.pos;
    offset_angle = gu::math::acos(
        neighbor_offset.dot(ref_state.vel)/(neighbor_offset.norm() * ref_state.vel.norm())
    ); // Use the dot product cosine rule
      
    // Calculate the stop cost
    stop_cost = stop_angle_weight_ * offset_angle + stop_dist_weight_ * neighbor_offset.norm() 
      - stop_vel_weight_ * vel_norm - stop_bias_; 
      
    log_file_ << ref_time.toSec() <<","
      <<  ref_state.pos(0) <<"," << ref_state.pos(1) <<"," << ref_state.pos(2)<< ","
      <<  ref_state.vel(0) <<"," << ref_state.vel(1) <<"," << ref_state.vel(2)<< ","
      <<  ref_state.vel.norm() << "," << true << "," 
      << neighbor_offset.norm() << "," << offset_angle << "," << stop_cost << "," << (stop_cost < 0) << ","
      << curr_neighbor.x() << "," << curr_neighbor.y() << "," << curr_neighbor.z() << std::endl;
  } else if(record_) {
    log_file_ << ref_time.toSec() <<","
      <<  ref_state.pos(0) <<"," << ref_state.pos(1) <<"," << ref_state.pos(2)<< ","
      <<  ref_state.vel(0) <<"," << ref_state.vel(1) <<"," << ref_state.vel(2)<< ","
      <<  ref_state.vel.norm() << "," << false << "," 
      << "0" << "," << "0" << "," << "0" << "," << false << ","
      << "0" << "," << "0" << "," << "0" << std::endl;
  }

  if(relevant_neighbors.size() > 0) visualizeNeighborhood(relevant_neighbors, neighbor_costs, ref_state.pos, ref_state.vel);

  if(relevant_neighbors.size() > 0 && vu::Min(neighbor_costs) < 0) {
    ROS_ERROR("[CommandStop] Generate stopping command due to neighbor cost < 0");
    generateCollisionFreeWaypoints(ref_state, ref_time, stopping_trajectory_duration);
    if (publish_hover_after_stop_)
    {
      std_msgs::String event_msg;
      event_msg.data = "HoverEvent";
      event_pub_.publish(event_msg);
    }
  }

  return;
}

} // namespace planner