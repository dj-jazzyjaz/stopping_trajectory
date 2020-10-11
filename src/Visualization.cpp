#include <stopping_trajectory/StoppingTrajectory.h>

namespace planner
{

namespace gu = geometry_utils;
namespace pu = parameter_utils;
namespace vu = vector_utils;
namespace gr = gu::ros;

void StoppingTrajectory::displayEscapePoints() {
    ros::Rate r(2); //Display at a rate of 2 Hz
    while (ros::ok()) {
        ros::Time ref_time;
        state_t ref_state;
        float lookahead = 0;
        if (!getNextReference(ref_state, ref_time, lookahead))
        {
            ROS_ERROR("reference not found; escape points not generated");
        }
        if(ref_state.vel.norm() < 0.0001) continue;
        traj_.markers.clear();
        std::vector<gu::Vec3> escapePoints;
        getEscapePoints(ref_state.pos, ref_state.vel, ref_state.yaw(), escapePoints);
        r.sleep();
    }   
}

visualization_msgs::Marker StoppingTrajectory::getTrajectoryVis(std::vector<state_t> traj_path)
{
    int id = traj_.markers.size();
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = fixed_frame_id_;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.pose.position = gr::toPoint(gu::Vec3(0, 0, 0));

    // Give trajectory color ranging between blue and red based off index
    std_msgs::ColorRGBA color;
    color.a = 0.5;
    color.r = std::min(1.0, 0.01 * id);
    color.b = std::max(0.01 * (100-id), 0.0);
    color.g = 0.0;
    marker.color = color;
    
    for (unsigned int i = 0; i<traj_path.size(); i++) {
        marker.points.push_back(gr::toPoint(traj_path[i].pos));
    }
    return marker;
}

void StoppingTrajectory::publishTrajectoryVis()
{
  stop_traj_vis_pub_.publish(traj_);
  std::cout << "Visualize sampled stopping trajectories, # traj = " << traj_.markers.size() << std::endl;
}

void StoppingTrajectory::visualizeSampleSpace(std::vector<gu::Vec3> rect_pts) {
    // Visualize the rectangle by getting the corners
    if(rect_pts.size() == 0) return;

    int y = sample_length_n;
    int z = sample_height_n;
    gu::Vec3 corner1 = rect_pts[0];
    gu::Vec3 corner2 = rect_pts[z-1];
    gu::Vec3 corner3 = rect_pts[y*z-z];
    gu::Vec3 corner4 = rect_pts[y*z-1];
    gu::Vec3 corner5 = rect_pts[rect_pts.size() - y*z];
    gu::Vec3 corner6 = rect_pts[rect_pts.size() - y*z + z - 1];
    gu::Vec3 corner7 = rect_pts[rect_pts.size() - z];
    gu::Vec3 corner8 = rect_pts[rect_pts.size() - 1];
    
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = fixed_frame_id_;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.pose.position = gr::toPoint(gu::Vec3(0, 0, 0));
    marker.colors = {green_, green_, green_, green_, green_,
      red_, red_, red_, red_, red_,
      red_, green_, green_, red_, red_, green_};

    marker.points.push_back(gr::toPoint(corner1));
    marker.points.push_back(gr::toPoint(corner2));
    marker.points.push_back(gr::toPoint(corner4));
    marker.points.push_back(gr::toPoint(corner3));
    marker.points.push_back(gr::toPoint(corner1));

    marker.points.push_back(gr::toPoint(corner5));
    marker.points.push_back(gr::toPoint(corner6));
    marker.points.push_back(gr::toPoint(corner8));
    marker.points.push_back(gr::toPoint(corner7));
    marker.points.push_back(gr::toPoint(corner5));

    marker.points.push_back(gr::toPoint(corner7));
    marker.points.push_back(gr::toPoint(corner3));
    marker.points.push_back(gr::toPoint(corner4));
    marker.points.push_back(gr::toPoint(corner8));
    marker.points.push_back(gr::toPoint(corner6));
    marker.points.push_back(gr::toPoint(corner2));
    
    sample_space_vis_pub_.publish(marker);
}

void StoppingTrajectory::visualizeEscapePoints(std::vector<gu::Vec3> escape_points, std::vector<float> costs) {
  std::vector<float> scaled_costs;
  vu::RescaleMinMax(costs, &scaled_costs);

  marker_array_.markers.clear();
  double max_size = 0.2;
  double min_size = 0.04;

  double max_cost, min_cost, scale;
  scale = 1.0;
  if(escape_points.size() > 0) {
    max_cost = 1;
    min_cost = 0;
    scale = (max_cost - min_cost == 0) ? 0 : (max_size-min_size)/(max_cost-min_cost);
  }
  
  for (unsigned int i = 0; i<escape_points.size(); i++) {
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = fixed_frame_id_;
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    // Scale the size of the marker relative to cost
    //double marker_size = (scaled_costs[i]-min_cost) * scale + min_size; 
    double marker_size = 0.05;
    marker.scale.x = marker_size;
    marker.scale.y = marker_size;
    marker.scale.z = marker_size;
    marker.pose.position = gr::toPoint(escape_points[i]);
    // marker.color = red_;

    // Markers that are more red indicate lower costs, markers that are more blue
    // indicate higher costs
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.r = std::min(1.0, 0.01 * i);
    color.b = std::max(0.01 * (100-i), 0.0);
    color.g = 0.0;
    marker.color = color;

    marker_array_.markers.push_back(marker);
  }
  escape_points_vis_pub_.publish(marker_array_);
}

void StoppingTrajectory::visualizeNeighborhood(std::vector<gu::Vec3> neighbor_points, std::vector<float> neighbor_costs, gu::Vec3 vehicle_pos, gu::Vec3 vehicle_vel) {
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for(int i = 0; i < neighbor_points.size(); i++) {
    gu::Vec3 neighbor = neighbor_points[i];
    auto cost = neighbor_costs[i];

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = fixed_frame_id_;
    marker.id = id;
    id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.pose.position = gr::toPoint(gu::Vec3(0, 0, 0));

    // Give trajectory color ranging between blue and red based off index
    std_msgs::ColorRGBA color;
    if(cost > 0) {
      printf("green\n");
      color = green_;
    } else {
      printf("red\n");
      color = red_;
    }
    marker.color = color;
    
    marker.points.push_back(gr::toPoint(neighbor));
    marker.points.push_back(gr::toPoint(vehicle_pos));

    marker_array.markers.push_back(marker);
  }

  // Make marker for velocity vector
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = fixed_frame_id_;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.pose.position = gr::toPoint(gu::Vec3(0, 0, 0));
  marker.color = pink_;
  
  marker.points.push_back(gr::toPoint(vehicle_pos + vehicle_vel));
  marker.points.push_back(gr::toPoint(vehicle_pos));

  marker_array.markers.push_back(marker);
  stop_traj_vis_pub_.publish(marker_array);
}

void StoppingTrajectory::writeLog(){
  if (record_) {
    double stddev;
    double query_time = stats_utils::Average(avg_query_time, &stddev);
    std::cout << "Average query time per escape point (ms)" << 1000*query_time << std::endl;
    std::cout << "Average sample time (ms)" <<  1000*stats_utils::Average(sample_time, &stddev) << std::endl;
    
    for(uint i = 0; i < sample_log.size(); i++) {
      SampleLog s = sample_log[i];
      std::cout << "# Points, # Free-space points, # Sample points, Sample Width, Sample Length, Variance, Stdev, Sample time" << std::endl;
      std::cout << s.num_points_initial << "\n"
        << s.num_free_points << "\n"
        << s.num_sample_points << "\n"
        << s.sample_space_width << "\n"
        << s.sample_space_length << "\n"
        << s.variance << "\n" 
        << s.stddev << "\n" 
        << s.t_sample << std::endl;
    }
  }
}
}