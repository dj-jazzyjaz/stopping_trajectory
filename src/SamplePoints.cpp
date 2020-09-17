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
 * Returns the time to sort in seconds
 */
double StoppingTrajectory::sortByCost(std::vector<float>& costs, 
                          std::vector<gu::Vec3>& escape_points) {  
  std::clock_t clock_start = std::clock()
  // Sort costs
  std::vector<size_t> costs_dist_sorted_idx;
  vu::Sort<float>(costs, &costs, &costs_dist_sorted_idx);

  // Push escape points by sorted cost
  std::vector<gu::Vec3> escape_points_sorted;
  for(uint i = 0; i < escape_points.size(); i++) {
    escape_points_sorted.push_back(escape_points[costs_dist_sorted_idx[i]]);
  }
  escape_points = escape_points_sorted;

  double sort_time = (std::clock() - clock_start) / (double) CLOCKS_PER_SEC;
  return sort_time;
}

/** @brief Calculates cost for each point in `escape_points` and stores cost in `costs`.
 *  Removes points from `escape_points` if not in free space.
 */
void StoppingTrajectory::getFreePoints(gu::Vec3& pos, gu::Vec3& vel, 
                    std::vector<gu::Vec3>& escape_points, 
                    std::vector<float>& costs) {  
  std::vector<gu::Vec3> free_escape_points;
  for(uint i = 0; i < escape_points.size(); i++){
    float dist_from_obstacle = global_map_.find_nearest_neighbor(escape_points[i](0), 
                        escape_points[i](1), escape_points[i](2));
    // Keep escape point if not within collision_radius_
    if(dist_from_obstacle > collision_radius_) {
      costs.push_back(getCost(escape_points[i], pos, vel, dist_from_obstacle));
      free_escape_points.push_back(escape_points[i]);
    }        
  }
  escape_points = free_escape_points;
}

/** @brief Samples from given `escape_points` using method specified by sample_method. Returns 
 * sampled points.
 */
std::vector<gu::Vec3> StoppingTrajectory::sampleEscapePoints(gu::Vec3& pos, gu::Vec3& vel, 
                    std::vector<gu::Vec3>& escape_points, SampleLog& log_entry) {  
  int num_escape_points_init = escape_points.size();
  std::clock_t clock_start = std::clock();
  std::vector<float> costs;
  getFreePoints(pos, vel, escape_points, costs); /* Calculate cost of each point */
  double cost_duration = (std::clock() - clock_start) / (double) CLOCKS_PER_SEC;

  if(escape_points.size() == 0) {
    return escape_points;
  }

  if(log_) {
    log_entry.t_cost = cost_duration;
    log_entry.t_cost_per_point = cost_duration/num_escape_points_init;
    log_entry.num_free_points = escape_points.size();
  }
  
  std::clock_t sample_clock_start = std::clock();
  double sort_duration = 0;

  // Weighted random sample
  if(sample_method_ == weighted_random) {
    weightedRandomSample(costs, escape_points);
    sort_duration = sortByCost(costs, escape_points);
  }

  // Stratified sample
  if(sample_method_ == stratified) {
    sort_duration = sortByCost(costs, escape_points); // Pre sort the points
    stratifiedSample(costs, escape_points);
  }

  // Sample best n points
  if(sample_method_ == best_n) {
    sort_duration = sortByCost(costs, escape_points);
    escape_points = vu::Slice(escape_points, 0, sample_num_);
    costs = vu::Slice(costs, 0, sample_num_);
  }
  
  // Log timing info
  if (log_) {
    log_entry.t_sample = (std::clock() - sample_clock_start) / (double) CLOCKS_PER_SEC;
    log_entry = logSampleStatistics(escape_points, log_entry);
    log_entry.t_sort = sort_duration;
    log_entry.t_sort_per_point = sort_duration/escape_points.size();
    log_entry.num_sample_points = escape_points.size();
    sample_log.push_back(log_entry);
  }
  
  visualizeEscapePoints(escape_points, costs);
  return escape_points;
}

/**
 * @brief Calculates statistics on a sample of escape points, including variance and standard deviation
 * of the distribution of points.
 * */
SampleLog StoppingTrajectory::logSampleStatistics(std::vector<gu::Vec3>& escapePoints, SampleLog log_entry) { 
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
 * @brief Sample points based on a weighted sample relative to costs,
 * where points with lower costs are more likely to be chosen.
 * */
void StoppingTrajectory::weightedRandomSample(std::vector<float>& costs, std::vector<gu::Vec3>& escape_points) {
  // Negate costs so that larger costs are less likely to be chosen
  for(uint i = 0; i < costs.size(); i++) {
    costs[i] = -1 * costs[i];
  }

  // Noramlize the costs and perform sample
  std::vector<float> scaled_costs;
  vu::RescaleMinMax(costs, &scaled_costs);
  vu::Normalize(scaled_costs, &costs);
  std::vector<int> sample_idxs = su::DiscreteSampleWithoutReplacement(costs, sample_num_);

  // Add points at sampled indexes
  std::vector<float> sampled_costs;
  std::vector<gu::Vec3> sampled_points; 
  for (uint i = 0; i < sample_idxs.size(); i++){
    int idx = sample_idxs[i];
    sampled_points.push_back(escape_points[idx]);
    sampled_costs.push_back(costs[idx]);
  }

  // Update costs and escape_points to the sample
  costs = sampled_costs;
  escape_points = sampled_points;
}

/**
 * @brief Returns indexes of chosen sample points using a straified, where the layers are specified 
 * by the parameters strat_sample_fraction_ and strat_sample_num_
 * */
void StoppingTrajectory::stratifiedSample(std::vector<float>& sorted_costs, std::vector<gu::Vec3>& escape_points) {
  int num_points = sorted_costs.size();

  // Negate costs to that higher costs are less likely to be sampled
  // TODO: Could potentially get rid of sampling weights since costs are similar
  for(uint i = 0; i < sorted_costs.size(); i++) {
    sorted_costs[i] = -1 * sorted_costs[i];
  }

  std::vector<int> sample_idxs;
  int start_idx = 0;
  // Get indexes by sampling from each stratified layer
  for(uint i = 0; i < strat_sample_fraction_.size(); i++) {
    int end_idx = start_idx + floor(strat_sample_fraction_[i] * num_points);
    std::vector<int> slice_idxs = sampleSlice(sorted_costs, start_idx, end_idx, strat_sample_num_[i]);
    sample_idxs.insert(sample_idxs.end(), slice_idxs.begin(), slice_idxs.end());
    start_idx = end_idx + 1; 
  }

  // Unnegate costs
  for(uint i = 0; i < sorted_costs.size(); i++) {
    sorted_costs[i] = -1 * sorted_costs[i];
  }

  // Push points at sampled indexes onto sampled points
  std::vector<gu::Vec3> sampled_points;
  std::vector<float> sampled_costs;
  for(uint i = 0; i < sample_idxs.size(); i++) {
      sampled_points.push_back(escape_points[sample_idxs[i]]);
      sampled_costs.push_back(sorted_costs[sample_idxs[i]]);
  }
  sortByCost(sampled_costs, sampled_points);
  escape_points = sampled_points;
  sorted_costs = sampled_costs;

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
