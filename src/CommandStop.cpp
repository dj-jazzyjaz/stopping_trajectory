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
 * @brief Generates a stopping trajectory if vehicle meets criteria for stopping
 * */
void StoppingTrajectory::commandStop(const ros::TimerEvent &, float stopping_trajectory_duration)
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

  /* Get points in neighbors */
  float neighbor_radius = 3.0f;
  std::vector<pcl::PointXYZ> neighbors; 
  global_map_.check_neighbor_in_radius(ref_state.pos.x(), ref_state.pos.y(), ref_state.pos.z(), neighbor_radius, &neighbors);
  std::cout << "Num neighbors" << neighbors.size() << std::endl;

  // Get orthonormal basis wrt vehicle velocity vector
  gu::Vec3 b1 = ref_state.vel.normalize();
  gu::Vec3 b2(-b1.y(), b1.x(), 0);
  b2 = b2.normalize();
  gu::Vec3 b3 = b1.cross(b2).normalize();
  gu::Mat33 R(b1.x(), b2.x(), b3.x(),
    b1.y(), b2.y(), b3.y(),
    b1.z(), b2.z(), b3.z()  
  );
  gu::Mat33 Rinv = R.inv();

  // std::cout << 'Rotation matrix' << R << std::endl;
  // std::cout << 'Rotation matrix inv' << Rinv << std::endl;
  gu::Vec3 nearest_neighbor; 
  float nearest_dist = MAXFLOAT;
  for(auto neighbor : neighbors) {
    gu::Vec3 curr_neighbor(neighbor.x, neighbor.y, neighbor.z);
    // Put the neighbor point in the robot velocity frame
    gu::Vec3 neighbor_wrt_body = Rinv * (curr_neighbor - ref_state.pos);
    // Scale the axes. TODO: scale with velocity 
    gu::Vec3 scaled_neighbor = neighbor_wrt_body/gu::Vec3(1, 2, 2);
    // Get the new 'distance'
    float scaled_dist = scaled_neighbor.norm();
    if(scaled_dist < nearest_dist && scaled_neighbor.x() > 0) {
      nearest_dist = scaled_dist;
      nearest_neighbor = curr_neighbor;
    }
  }

  // DEBUGGNG
  if(nearest_dist != MAXFLOAT) {
    gu::Vec3 curr_neighbor = nearest_neighbor;
    // Put the neighbor point in the robot velocity frame
    gu::Vec3 neighbor_wrt_body = Rinv * (curr_neighbor - ref_state.pos);
    // Scale the axes. TODO: scale with velocity 
    gu::Vec3 scaled_neighbor = neighbor_wrt_body/gu::Vec3(1, 2, 2);
    // Get the new 'distance'
    float scaled_dist = scaled_neighbor.norm();
    if(scaled_dist < nearest_dist) {
      nearest_dist = scaled_dist;
      nearest_neighbor = curr_neighbor;
    }
    /*std::cout << "Curr neighbor" << std::endl;
    curr_neighbor.print();*/
    std::cout << "Neighbor wrt body" << std::endl;
    neighbor_wrt_body.print();
    std::cout << "Scaled neighbor" << std::endl;
    scaled_neighbor.print();
    std::cout << "Scaled: " << scaled_dist << " Actual dist: " << (curr_neighbor - ref_state.pos).norm() << std::endl;
    ROS_INFO("[Command Stop] %f, %fm/s", nearest_dist - ref_state.vel.norm(), ref_state.vel.norm());
    visualizeNearestObstacle(ref_state.pos, curr_neighbor, ref_state.yaw(), b1, b2, b3);
  }


  if(nearest_dist < ref_state.vel.norm()) {

    std::cout << "======== Generate stopping command ========" << std::endl;
    generateCollisionFreeWaypoints(ref_state, ref_time, stopping_trajectory_duration);
    std_msgs::String event_msg;
    event_msg.data = "HoverEvent";
    event_pub_.publish(event_msg);
  }

  
  return;

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
}

} // namespace planner