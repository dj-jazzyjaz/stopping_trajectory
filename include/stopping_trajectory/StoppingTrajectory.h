#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
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
#include <planning_arch/collision_checker/CollisionChecker.h>
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
    int num_points_initial;   // Number of initial points generated
    float t_cost;             // Duration to calculate costs for all points
    float t_cost_per_point;   // Duration to calculate cost per 1 point
  
    int num_free_points;      // Number of points in free space
    float t_sort;             // Duration to sort escape points
    float t_sort_per_point;   // Duration to sort point per 1 point

    int num_sample_points;     // Number of points selected from sampling
    float sample_space_width;  // Physical width of sample area
    float sample_space_length; // Physical lenght of sample area
    float variance;            // Variance of sampled points from sample mean
    float stddev;              // Sqrt of variance
    float t_sample;            // Time to perform sample  
} SampleLog;

class StoppingTrajectory
{
public:
    StoppingTrajectory();
    ~StoppingTrajectory();
    bool initialize(const ros::NodeHandle &n);
    void generateWaypoints(state_t state, const ros::Time& reference_time);
    void generateCollisionFreeWaypoints(state_t state, const ros::Time& reference_time);
    std::unique_ptr<CollisionChecker> collision_avoidance_;

private:
    enum SamplingMethod {none, weighted_random, stratified, best_n};
    // Init
    bool initMap();
    void getParams();
    void flagsCallback(const control_arch::FsmFlags::ConstPtr& msg);
    bool flagEnabledQ(const std::string& flag);
    
    //Stopping Trajectory
    void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);
    bool getNextReference(state_t& state, ros::Time& reference_time, float lookahead);
    void publishTrajectory(std::vector<state_t> traj_wpts, float interval, ros::Duration duration);
    void displayEscapePoints();

    //Collision avoidance
    bool obstacle_avoidance_on_;

    // Compute Trajectory
    std::vector<double> getCoefficients(double pos, double vel, double acc, double jerk, double snap, double stopping_time);
    std::vector<double> getCoefficientsWithGoalPos(double pos, double vel, double acc, double jerk, double goal_pos, double stopping_time);
    void getPolynomials(const state_t& state, ros::Duration duration, std::vector<std::vector<double>> coefficients); 
    void getPolynomialsWithPos(const state_t& state, gu::Vec3 goal_pos, double t_stop, std::vector<std::vector<double>> coefficients);
    void populateTrajectory(const ros::Time &reference_time, int traj_length, const std::vector<std::vector<double>>& coefficients, ros::Duration duration, double step, std::vector<state_t> &traj);
    bool checkTrajectoryCollision(const std::vector<state_t>& traj);
    bool checkTrajectoryCollisionGlobal(const std::vector<state_t>& traj);
    bool checkTrajectoryHOD(const std::vector<std::vector<double>>& coefficients, ros::Duration& duration, std::vector<double> thresholds);
    bool checkAccelThreshold(const std::vector<std::vector<double>>& coefficients, ros::Duration &duration, double acc_threshold);
    double getTrajectoryDuration(state_t state, gu::Vec3 goal_pos);

    // Escape Points
    void getEscapePoints(gu::Vec3& pos, gu::Vec3& vel, double yaw, std::vector<gu::Vec3> escapePoints);
    void getRectangleGrid(double x1, double x2, double y1, double y2, std::vector<gu::Vec3> rectangleGrid);
    void generateGridPoints(gu::Vec3& pos, gu::Vec3& vel, double yaw, std::vector<gu::Vec3> gridPoints);
    float point_to_line_distance(gu::Vec3& point, gu::Vec3& line_pos, gu::Vec3& line_vel);
    float getCost(gu::Vec3& escape_point, gu::Vec3& current_pos, gu::Vec3& current_vel, float dist_from_obstacle);
   
    // Sample Points
    void getFreePoints(gu::Vec3& pos, gu::Vec3& vel, std::vector<gu::Vec3>& escape_points, std::vector<float>& costs);
    void sortByCost(std::vector<float>& costs, std::vector<gu::Vec3> escape_points);
    std::vector<gu::Vec3> sampleEscapePoints(gu::Vec3& pos, gu::Vec3& vel, std::vector<gu::Vec3> escape_points, SampleLog log_entry);
    std::vector<int> weightedSample(std::vector<float> costs);
    std::vector<int> stratifiedSample(std::vector<float>& sorted_costs, std::vector<gu::Vec3>& escape_points);
    std::vector<int> sampleSlice(std::vector<float> sorted_costs, int start, int end, int num_sample);
    SampleLog logSampleStatistics(std::vector<gu::Vec3> escapePoints, SampleLog log_entry);

    // Visualization
    visualization_msgs::Marker getTrajectoryVis(std::vector<state_t> traj_path);
    void publishTrajectoryVis();
    void visualizeEscapePoints(std::vector<gu::Vec3> escape_points, std::vector<float> costs);
    void visualizeSampleSpace(std::vector<gu::Vec3> rect_pts);

    // Command Stop
    void stopWithinDistance(const ros::TimerEvent &);
    float freePointsRatio(state_t ref_state);
    void commandStop(const ros::TimerEvent &);

    // Parameters
    double vel_threshold_;
    double acc_threshold_;
    double jerk_threshold_;
    double snap_threshold_;
    std::vector<double> thresholds_;
    float collision_radius_;
    float stopping_radius_;
    float stopping_angle_;
    double vel_deviation_weight_;
    double obstacle_dist_weight_;
    double grid_length_;
    double grid_width_;
    float free_ratio_thresh_;
    float delta_free_thresh_;
    int delta_count_thresh_; 
    float compute_thresh_;
   
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

    // Publishers & Subscribers
    ros::Publisher escape_points_vis_pub_;
    ros::Publisher stop_traj_vis_pub_;
    ros::Publisher sample_space_vis_pub_;
    ros::Publisher wpts_pub;
    ros::Publisher event_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber flags_sub_;
    ros::ServiceClient get_reference_state_;

    std::set<std::string> flags_; 
    GlobalMapGenerator global_map_;
    std_msgs::ColorRGBA red_, green_;
    std::string fixed_frame_id_;                // Vehicle base frame ID
    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::MarkerArray traj_;

    // Command Stop variables
    ros::Timer stop_timer_;
    float free_points_ratio;
    int below_threshold_count;

    // Logging
    void setUpFileWriting(const std::string& saveto_directory);
    void writeToLog();
    std::vector<SampleLog> sort_log;
    std::vector<double> sample_time;
    std::vector<double> avg_query_time;
    std::ofstream log_file_;
    bool record_;

};


} //namespace planner