#include <stopping_trajectory/StoppingTrajectory.h>

namespace planner {
namespace gu = geometry_utils;
namespace lu = linalg_utils;
namespace vu = vector_utils;
namespace gr = gu::ros;
namespace su = sample_utils;

/** @brief Samples escape points given the current vehicle state. 
 * 
 * Generates a rectangular prism grid of points, with dimensions based on parameters grid_length_ 
 * and grid_width_ and vel magnitude. Uses method specified by sample_method to choose points. 
 * */
void StoppingTrajectory::getEscapePoints(gu::Vec3& pos, gu::Vec3& vel, double yaw, std::vector<gu::Vec3>& escapePoints){
  std::clock_t sample_start = std::clock();
  generateGridPoints(pos, vel, yaw, escapePoints);
  // Log timing info
  double sample_duration = (std::clock() - sample_start) / (double) CLOCKS_PER_SEC;
  sample_time.push_back(sample_duration);
  
  SampleLog log_entry = {};
  log_entry.num_points_initial = escapePoints.size();
  log_entry.sample_space_width = grid_width_ * vel.norm();
  log_entry.sample_space_length = grid_length_ * vel.norm(); 
  
  escapePoints = sampleEscapePoints(pos, vel, escapePoints, log_entry);
}

/** @brief Returns cost of an escape point at the given vehicle state
 * 
 * Cost is a weighted sum of the deviation from vehicle heading (penalize for escape point outside 
 * of direct path of vehicle), and distance from obstacle (penalize for proximity to obstacle).
 * Cost weights are specified by paraemters obstacle_dist_weight and vel_deviation_weight.
 * 
 * */
float StoppingTrajectory::getCost(gu::Vec3& escape_point, gu::Vec3& current_pos, 
    gu::Vec3& current_vel, float dist_from_obstacle) {
  float deviation_from_velocity = point_to_line_distance(escape_point, current_pos, current_vel);

  return -obstacle_dist_weight_ * dist_from_obstacle + vel_deviation_weight_ * deviation_from_velocity;
}

/** @brief Generates a rectangular prism meshgrid with corners located at (x1, y1) and (x2, y2), 
 * and fixed z height of 2.
 * */
void StoppingTrajectory::getRectangleGrid(double x1, double x2, double y1, double y2, std::vector<gu::Vec3>& gridPts) {
  if(x2-x1 < sample_width_ || y2-y1 < sample_length_) {
    return;
  }
  
  std::vector<double> xs = lu::Arange<double>(x1, x2, sample_width_);
  std::vector<double> ys = lu::Arange<double>(y1, y2, sample_length_);
  std::vector<double> zs = lu::Arange<double>(-1, 1, sample_height_);
  sample_length_n = ys.size(); // Used for visualizing the sample space
  sample_height_n = zs.size(); // Also used for visualization

  for (uint ix = 0; ix < xs.size(); ix++){
    for (uint iy = 0; iy < ys.size(); iy++){
      for(uint iz = 0; iz < zs.size(); iz++) {
        gridPts.push_back(gu::Vec3(xs[ix], ys[iy], zs[iz]));
      }   
    }
  }
}

/** @brief Generates points in rectangular prism grid, centered about the velocity vector of the 
 * vehicle, with dimensions based off the velocity and parameters grid_length_ and grid_width_.
 * */
void StoppingTrajectory::generateGridPoints(gu::Vec3& pos, gu::Vec3& vel, double yaw, std::vector<gu::Vec3>& rect_pts) {
  double gridLength = grid_length_ * vel.norm();
  double gridWidth = grid_width_ * vel.norm();

  // Get rectangular meshgrid of points, relative to origin and no rotation
  getRectangleGrid(0, gridLength, -gridWidth, gridWidth, rect_pts);

  // Transform points to vehicle's location and rotation
  gu::Rotation3Base<double> rotation = gu::Rotation3Base<double>(0, 0, yaw);
  gu::Transform3d trans;
  trans.translation = pos;
  trans.rotation = rotation;
  for (uint i = 0; i < rect_pts.size(); i++){
    rect_pts[i] = trans * rect_pts[i];
  }

  visualizeSampleSpace(rect_pts);
  
  // Additionally, sample points directly along velocity vector. 
  // Start from i=1 so we do not include the current position
  gu::Vec3 y_axis = vel.normalize();
  for (float i = sample_length_; i < (float) gridLength; i+=sample_length_) {
    rect_pts.push_back(pos + i * y_axis);
  }

  // Remove points with z < 0
  vu::RemoveAllIf(rect_pts, [](gu::Vec3 point){return point.data[2] < 0;});
}

/** @brief Calculates the closest distance from a point in 3-space to a vector line. line_point is a 
 * point along the line, line_slope is the slope of the line
 * */
float StoppingTrajectory::point_to_line_distance(gu::Vec3& point, gu::Vec3& line_point, 
                                                gu::Vec3& line_slope) {
  gu::Vec3 x1 = point - line_point;
  gu::Vec3 x2 = line_point + line_slope;
  gu::Vec3 x3 = point - x2;
  gu::Vec3 v1 = x1.cross(x3);
  gu::Vec3 v2 = x2-line_point;

  return v1.norm()/v2.norm();
}
}
