#include <stopping_trajectory/StoppingTrajectory.h>

namespace planner {
namespace gu = geometry_utils;
namespace lu = linalg_utils;
namespace vu = vector_utils;
namespace gr = gu::ros;
namespace su = sample_utils;
namespace stu = stats_utils;

/** @brief Sort escape_points by corresponding costs, where costs[i] is the cost for escape_points[i].
 * Also sorts `costs` in place.
 * 
 * */
void StoppingTrajectory::sortByCost(std::vector<float>& costs, 
                          std::vector<gu::Vec3> escape_points) {  
  // Sort costs via distance
  std::vector<size_t> costs_dist_sorted_idx;
  vu::Sort<float>(costs, &costs, &costs_dist_sorted_idx);

  // Push escape points by sorted distance
  std::vector<gu::Vec3> escape_points_sorted;
  for(uint i = 0; i < escape_points.size(); i++) {
    escape_points_sorted.push_back(escape_points[costs_dist_sorted_idx[i]]);
  }
  escape_points = escape_points_sorted;
}

/** @brief Calculates cost for each point in `escape_points` and stores cost in `costs`.
 *  Removes points from `escape_points` if not in free space.
 * */
void StoppingTrajectory::getFreePoints(gu::Vec3& pos, gu::Vec3& vel, 
                    std::vector<gu::Vec3>& escape_points, 
                    std::vector<float>& costs) {  
  std::vector<gu::Vec3> free_escape_points;
  for(uint i = 0; i < escape_points.size(); i++){
    float dist_from_obstacle = global_map_.find_nearest_neighbor(escape_points[i](0), 
                        escape_points[i](1), escape_points[i](2));
    if(dist_from_obstacle > collision_radius_) {
      costs.push_back(getCost(escape_points[i], pos, vel, dist_from_obstacle));
      free_escape_points.push_back(escape_points[i]);
    }        
  }
  escape_points = free_escape_points;
}

/** @brief Samples from given `escape_points` using method specified by sample_method. Returns 
 * sampled points.
 * */
std::vector<gu::Vec3> StoppingTrajectory::sampleEscapePoints(gu::Vec3& pos, gu::Vec3& vel, 
                    std::vector<gu::Vec3> escape_points, SampleLog log_entry) {  
  int num_escape_points_init = escape_points.size();
  
  std::clock_t clock_start = std::clock();
  std::vector<float> costs;
  getFreePoints(pos, vel, escape_points, costs);

  if(escape_points.size() == 0) {
    return escape_points;
  }

  // Log timing info
  double cost_duration = (std::clock() - clock_start) / (double) CLOCKS_PER_SEC;
  log_entry.t_cost = cost_duration;
  log_entry.t_cost_per_point = cost_duration/num_escape_points_init;
  log_entry.num_free_points = escape_points.size();

  // Weighted random sample
  if(sample_method_ == weighted_random) {
    std::clock_t sample_clock_start = std::clock();
    std::vector<float> sampled_costs;
    std::vector<gu::Vec3> sampled_points;

    // Get indexes from weighted sample using costs
    std::vector<int> sample_idxs = weightedSample(costs);

    // Add points at sample_idxs 
    for (uint i = 0; i < sample_idxs.size(); i++){
      int idx = sample_idxs[i];
      sampled_points.push_back(escape_points[idx]);
      sampled_costs.push_back(costs[idx]);
    }
    costs = sampled_costs;
    escape_points = sampled_points;
    log_entry.t_sample = (std::clock() - sample_clock_start) / (double) CLOCKS_PER_SEC;
  }
  clock_start = std::clock();
  sortByCost(costs, escape_points);

  // Stratified sample
  if(sample_method_ == stratified) {
    std::clock_t sample_clock_start = std::clock();

    // Get sampled indexs
    std::vector<int> sample_idxs = stratifiedSample(costs, escape_points);
    
    std::vector<gu::Vec3> sampled_points;
    std::vector<float> sampled_costs;
    for(uint i = 0; i < sample_idxs.size(); i++) {
      sampled_points.push_back(escape_points[sample_idxs[i]]);
      sampled_costs.push_back(costs[sample_idxs[i]]);
    }

    sortByCost(sampled_costs, sampled_points);
    escape_points = sampled_points;
    costs = sampled_costs;

    log_entry.t_sample = (std::clock() - sample_clock_start) / (double) CLOCKS_PER_SEC;
  }

  // Sample best n points
  if(sample_method_ == best_n) {
    std::clock_t sample_clock_start = std::clock();
    escape_points = vu::Slice(escape_points, 0, sample_num_);
    costs = vu::Slice(costs, 0, sample_num_);
    log_entry.t_sample = (std::clock() - sample_clock_start) / (double) CLOCKS_PER_SEC;
  }
  
  // Log timing info
  double sort_duration = (std::clock() - clock_start) / (double) CLOCKS_PER_SEC;
  log_entry = logSampleStatistics(escape_points, log_entry);
  log_entry.t_sort = sort_duration;
  log_entry.t_sort_per_point = sort_duration/escape_points.size();
  log_entry.num_sample_points = escape_points.size();
  
  sort_log.push_back(log_entry);
  visualizeEscapePoints(escape_points, costs);
  return escape_points;
}

SampleLog StoppingTrajectory::logSampleStatistics(std::vector<gu::Vec3> escapePoints, SampleLog log_entry) { 
  // Get individual vectors for x, y and z
  std::vector<double> x, y, z;
  for(uint i = 0; i < escapePoints.size(); i++){
    x.push_back(escapePoints[i].x());
    y.push_back(escapePoints[i].y());
    z.push_back(escapePoints[i].z());
  }

  // Find mean for each of x, y and z
  double temp;
  double mean_x = stu::Average(x, &temp);
  double mean_y = stu::Average(y, &temp);
  double mean_z = stu::Average(z, &temp);

  // Get squared distance of each point from mean to determine variance
  double variance;
  for(uint i = 0; i < escapePoints.size(); i++){
    double dist_sqr = (mean_x - x[i]) * (mean_x - x[i]) +
      (mean_y - y[i]) * (mean_y - y[i]) +
      (mean_z - z[i]) * (mean_z - z[i]);
    variance += dist_sqr;
  }
  variance /= escapePoints.size();
  double stddev = pow(variance, 0.5);
  log_entry.variance = variance;
  log_entry.stddev = stddev;
  return log_entry;
}

/**
 * @brief Returns indexes of chosen sample points using a straified, where the layers are specified 
 * by the parameters strat_sample_fraction_ and strat_sample_num_
 * */
std::vector<int> StoppingTrajectory::stratifiedSample(std::vector<float>& sorted_costs, std::vector<gu::Vec3>& escape_points) {
  int num_points = sorted_costs.size();

  // TODO: Probably can get rid of this
  for(uint i = 0; i < sorted_costs.size(); i++) {
    sorted_costs[i] = -1 * sorted_costs[i];
  }

  std::vector<int> sample_idxs;
  int start_idx = 0;
  // Sample from each stratified layer
  for(uint i = 0; i < strat_sample_fraction_.size(); i++) {
    int end_idx = start_idx + floor(strat_sample_fraction_[i] * num_points);
    std::vector<int> slice_idxs = sampleSlice(sorted_costs, start_idx, end_idx, strat_sample_num_[i]);
    sample_idxs.insert(sample_idxs.end(), slice_idxs.begin(), slice_idxs.end());
    start_idx = end_idx + 1; 
  }

  for(uint i = 0; i < sorted_costs.size(); i++) {
    sorted_costs[i] = -1 * sorted_costs[i];
  }

  return sample_idxs;
}

/**
 * @brief Returns indexes of chosen sample points based on a weighted sample relative to costs,
 * where points with lower costs are more likely to be chosen.
 * */
std::vector<int> StoppingTrajectory::weightedSample(std::vector<float> costs) {
  // Multiply costs by -1 so that better points->lower cost->higher probability
  for(uint i = 0; i < costs.size(); i++) {
    costs[i] = -1 * costs[i];
  }
  std::vector<float> scaled_costs;
  vu::RescaleMinMax(costs, &scaled_costs);
  vu::Normalize(scaled_costs, &costs);
  std::vector<int> sampled_points = su::DiscreteSampleWithoutReplacement(costs, sample_num_);

  return sampled_points;
}

/**
 * @brief Samples a slice of the given array between indices `start` and `end`. 
 * */
std::vector<int> StoppingTrajectory::sampleSlice(std::vector<float> sorted_costs, int start, 
  int end, int num_sample) {
  std::vector<float> slice = vu::Slice(sorted_costs, start, end);
  std::vector<float> scaled_costs;
  vu::RescaleMinMax(sorted_costs, &scaled_costs);
  vu::Normalize(scaled_costs, &sorted_costs);
  std::vector<int> sampled_points = su::DiscreteSampleWithoutReplacement(sorted_costs, num_sample);
  return sampled_points;
}

}