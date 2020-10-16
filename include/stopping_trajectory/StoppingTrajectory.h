#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <collision_checker/CollisionChecker.h>
#include <control_arch/utils/state_t.h>
#include <control_arch/GetReferenceState.h>
#include <control_arch/Waypoints.h>
#include <control_arch/trajectory/Waypoints.h>
#include <control_arch/trajectory/Polynomial.h>
#include <cmath>
#include <control_arch/FsmFlags.h>
#include <geometry_utils/GeometryUtils.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/point_types.h>
#include <cpp_utils/vector_utils.h>
#include <cpp_utils/linalg_utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <local_map_generator/global_map_generator.h>
#include <cpp_utils/stats_utils.h>
#include <cpp_utils/sample_utils.h>
#include <std_msgs/String.h>

namespace planner {
namespace gu = geometry_utils;

typedef struct {
    int num_points_initial;   // Number of initial points generatedt
    int num_free_points;      // Number of points in free space

    int num_sample_points;     // Number of points selected from sampling
    float sample_space_width;  // Physical width of sample area
    float sample_space_length; // Physical length of sample area
    float variance;            // Variance of sampled points from sample mean
    float stddev;              // Sqrt of variance
    float t_sample;            // Time to perform sample  
} SampleLog;

class StoppingTrajectory
{
public:
    StoppingTrajectory();
    ~StoppingTrajectory();
    bool initialize(const ros::NodeHandle &n, const std::shared_ptr<CollisionChecker>& collision_checker=nullptr);
    void generateWaypoints(state_t state, const ros::Time& reference_time, float stopping_trajectory_duration=1.0);
    void generateCollisionFreeWaypoints(state_t state, const ros::Time& reference_time, float stopping_trajectory_duration=2.0);
    void commandStop(const ros::TimerEvent &, float stopping_trajectory_duration=2.0);
    
private:
    enum SamplingMethod {none, weighted_random, stratified, best_n};
    
    // Collision Checker
    std::shared_ptr<CollisionChecker> collision_checker_;
    
    // Init
    bool initGlobalMap();
    bool initGMMMap(const std::shared_ptr<CollisionChecker>& collision_checker);
    void getParams();
    void flagsCallback(const control_arch::FsmFlags::ConstPtr& msg);
    bool flagEnabledQ(const std::string& flag);
    
    //Stopping Trajectory
    bool getNextReference(state_t& state, ros::Time& reference_time, float lookahead);
    void publishTrajectory(std::vector<state_t> traj_wpts, float interval, ros::Duration duration);
    void displayEscapePoints();

    //Collision avoidance
    bool obstacle_avoidance_on_;

    // Compute Trajectory
    std::vector<double> getCoefficients(double pos, double vel, double acc, double jerk, double snap, double stopping_time);
    std::vector<double> getCoefficientsWithGoalPos(double pos, double vel, double acc, double jerk, double goal_pos, double stopping_time);
    void getPolynomials(const state_t& state, ros::Duration duration, std::vector<std::vector<double>>& coefficients); 
    void getPolynomialsWithPos(const state_t& state, gu::Vec3 goal_pos, double t_stop, std::vector<std::vector<double>>& coefficients);
    void populateTrajectory(const ros::Time &reference_time, int traj_length, const std::vector<std::vector<double>>& coefficients, ros::Duration duration, double step, std::vector<state_t> &traj);
    bool checkTrajectoryCollisionGlobal(const std::vector<state_t>& traj);
    bool checkTrajectoryCollisionGMM(const std::vector<state_t>& traj);
    bool checkTrajectoryHOD(const std::vector<std::vector<double>>& coefficients, ros::Duration& duration, std::vector<double> thresholds);
    bool checkAccelThreshold(const std::vector<std::vector<double>>& coefficients, ros::Duration &duration, double acc_threshold);

    // Escape Points
    void getEscapePoints(gu::Vec3& pos, gu::Vec3& vel, double yaw, std::vector<gu::Vec3>& escapePoints);
    void getRectangleGrid(double x1, double x2, double y1, double y2, std::vector<gu::Vec3>& rectangleGrid);
    void generateGridPoints(gu::Vec3& pos, gu::Vec3& vel, double yaw, std::vector<gu::Vec3>& gridPoints);
    float point_to_line_distance(gu::Vec3& point, gu::Vec3& line_pos, gu::Vec3& line_vel);
    float getCost(gu::Vec3& escape_point, gu::Vec3& current_pos, gu::Vec3& current_vel, float dist_from_obstacle);
   
    // Sample Points
    void getFreePoints(gu::Vec3& pos, gu::Vec3& vel, std::vector<gu::Vec3>& escape_points, std::vector<float>& costs);
    void sortByCost(std::vector<float>& costs, std::vector<gu::Vec3>& escape_points);
    std::vector<gu::Vec3> sampleEscapePoints(gu::Vec3& pos, gu::Vec3& vel, std::vector<gu::Vec3>& escape_points, SampleLog& log_entry);
    void stratifiedSample(std::vector<float>& sorted_costs, std::vector<gu::Vec3>& escape_points);
    std::vector<int> sampleSlice(std::vector<float> sorted_costs, int start, int end, int num_sample);
    SampleLog logSampleStatistics(std::vector<gu::Vec3>& escapePoints, SampleLog log_entry);
    void weightedRandomSample(std::vector<float>& costs, std::vector<gu::Vec3>& escape_points);
    // Visualization
    visualization_msgs::Marker getTrajectoryVis(std::vector<state_t> traj_path);
    void publishTrajectoryVis();
    void visualizeEscapePoints(std::vector<gu::Vec3> escape_points, std::vector<float> costs);
    void visualizeSampleSpace(std::vector<gu::Vec3> rect_pts);
    void visualizeNeighborhood(std::vector<gu::Vec3> neighbor_points, std::vector<float> neighbor_costs, gu::Vec3 vehicle_pos, gu::Vec3 vehicle_vel);
    
    // Command Stop
    void stopWithinDistance(const ros::TimerEvent &);
    float freePointsRatio(state_t ref_state);
    double findNearestNeighbor(gu::Vec3 pos, float neighbor[3]=NULL);
    bool checkNeighborInRadius(gu::Vec3 pos, float neighbor_radius, std::vector<pcl::PointXYZ>* neighbors=nullptr);

    // Parameters
    double vel_threshold_;
    double acc_threshold_;
    double jerk_threshold_;
    double snap_threshold_;
    std::vector<double> thresholds_;
    float collision_radius_;
    float stopping_radius_;
    double grid_length_;
    double grid_width_;
    float vel_deviation_weight_;
    float obstacle_dist_weight_;
    float compute_thresh_;
    float stop_dist_weight_;
    float stop_vel_weight_;
    float stop_angle_weight_;
    float stop_bias_;
   
    // Sample variables
    int sample_length_n;
    int sample_height_n;
    float sample_length_;
    float sample_width_;
    float sample_height_;
    int sample_method_;
    int sample_num_; 
    std::vector<float> strat_sample_fraction_;
    std::vector<int> strat_sample_num_;

    // Mapping
    int map_rep_;

    // Publishers & Subscribers
    ros::Publisher escape_points_vis_pub_;
    ros::Publisher stop_traj_vis_pub_;
    ros::Publisher sample_space_vis_pub_;
    ros::Publisher neighbors_vis_pub_;
    ros::Publisher wpts_pub;
    ros::Publisher event_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber flags_sub_;
    ros::ServiceClient get_reference_state_;

    std::set<std::string> flags_; 
    GlobalMapGenerator global_map_;
    std_msgs::ColorRGBA red_, green_, pink_;
    std::string fixed_frame_id_;                // Vehicle base frame ID
    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::MarkerArray traj_;

    // Logging
    void writeLog();
    std::vector<SampleLog> sample_log;
    std::vector<double> sample_time;
    std::vector<double> avg_query_time;
    std::ofstream log_file_;
    bool record_;

};


} //namespace planner
