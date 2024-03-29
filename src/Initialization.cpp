#include <stopping_trajectory/StoppingTrajectory.h>

namespace planner
{
namespace gu = geometry_utils;
namespace pu = parameter_utils;
namespace gr = gu::ros;

bool StoppingTrajectory::initialize(const ros::NodeHandle &n, const std::shared_ptr<CollisionChecker>& collision_checker)
{
  ros::NodeHandle nh(n); //make copy of n
  flags_sub_ = nh.subscribe("flags", 0, &StoppingTrajectory::flagsCallback, this);

  std::string prefix = ros::this_node::getNamespace();
  wpts_pub = nh.advertise<control_arch::Waypoints>(ros::names::append(ros::names::append(prefix, "trajectory"), "waypoints"), 1);
  get_reference_state_ = nh.serviceClient<control_arch::GetReferenceState>("get_reference_state");
  ROS_INFO("[Stopping Trajectory] Waiting for Reference State service.");
  get_reference_state_.waitForExistence();
  ROS_INFO("[Stopping Trajectory] Reference State service found.");

  // Publishers
  escape_points_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("escape_points_vis", 1);
  stop_traj_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("stop_traj_vis", 1);
  sample_space_vis_pub_ = nh.advertise<visualization_msgs::Marker>("sample_space_vis", 1);
  neighbors_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("stop_neighbors_vis", 1);
  event_pub_ = nh.advertise<std_msgs::String>("event", 0, true);
  
  getParams();

  if (map_rep_ == 0) {
    if (!initGlobalMap()) return false;
  } else if (map_rep_ == 1) {
    if (!initGMMMap(collision_checker)) return false;
  }
  

  if (!pu::get("frame_id/fixed", fixed_frame_id_))
  {
    ROS_INFO("[Stopping Trajectory] Could not get frame_id");
    return false;
  }

  ROS_INFO("[Stopping Trajectory] stopping_trajectory initialzed");
  record_ = true;
  return true;
}

bool StoppingTrajectory::initGlobalMap()
{
  std::string map_name;
  if (!pu::get("map_filename", map_name))
    return false;
  float discretization_ = 0.1;
  global_map_ = GlobalMapGenerator(map_name, discretization_, false);
  bool voxel_grid_created_ = global_map_.create_voxel_map();

  if (!voxel_grid_created_)
  {
    ROS_INFO("[Stopping Trajectory] Could not get voxel grid map");
    return false;
  }
  return true;
}

bool StoppingTrajectory::initGMMMap(const std::shared_ptr<CollisionChecker>& collision_checker)
{
  if(collision_checker_ == nullptr) {
    ROS_ERROR("[Stopping Trajectory] Collision checker shared pointer was null, failed to initialize map");
    return false;
  }
  collision_checker_ = collision_checker;

  // TODO: GMM Map
  return true;
}

void StoppingTrajectory::getParams()
{
  // Get parameters
  pu::get("hod_threshold/vel", vel_threshold_, double(100.0));
  pu::get("hod_threshold/acc", acc_threshold_, double(100.0));
  pu::get("hod_threshold/jerk", jerk_threshold_, double(100.0));
  pu::get("hod_threshold/snap", snap_threshold_, double(500.0));
  thresholds_.assign({vel_threshold_, acc_threshold_, jerk_threshold_, snap_threshold_});
  pu::get("cost_function/vel_deviation_weight", vel_deviation_weight_, float(0.5));
  pu::get("cost_function/obstacle_dist_weight", obstacle_dist_weight_, float(0.5));
  pu::get("escape_point_generation/grid_length", grid_length_);
  pu::get("escape_point_generation/grid_width", grid_width_);
  pu::get("escape_point_generation/sample_length", sample_length_);
  pu::get("escape_point_generation/sample_width", sample_width_);
  pu::get("escape_point_generation/sample_height", sample_height_);
  pu::get("collision_radius", collision_radius_);
  pu::get("command_stop/stopping_radius", stopping_radius_, float(1.0));
  pu::get("sample_method", sample_method_);
  pu::get("sample_num", sample_num_);
  if (sample_method_ == stratified)
  {
    pu::get("stratified_sample/sample_fraction", strat_sample_fraction_);
    pu::get("stratified_sample/sample_num", strat_sample_num_);
  }
  pu::get("command_stop/dist_weight", stop_dist_weight_);
  pu::get("command_stop/vel_weight", stop_vel_weight_);
  pu::get("command_stop/angle_weight", stop_angle_weight_);
  pu::get("command_stop/bias", stop_bias_);
  pu::get("compute_thresh", compute_thresh_);
  pu::get("map", map_rep_);

  record_ = false;
  
  red_.a = 1;
  red_.r = 1;
  red_.b = 0;
  red_.g = 0;

  green_.a = 0.5;
  green_.r = 0.0;
  green_.b = 0.0;
  green_.g = 1.0;

  pink_.a = 0.8;
  pink_.r = 1;
  pink_.g = 0;
  pink_.b = 0.5;
}

void StoppingTrajectory::flagsCallback(const control_arch::FsmFlags::ConstPtr &msg)
{
  flags_.clear();
  flags_.insert(msg->flags.begin(), msg->flags.end());
}

bool StoppingTrajectory::flagEnabledQ(const std::string &flag)
{
  return flags_.count(flag) != 0;
}

} // namespace planner