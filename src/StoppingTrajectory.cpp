#include <stopping_trajectory/StoppingTrajectory.h>

namespace planner
{

namespace gu = geometry_utils;
namespace pu = parameter_utils;
namespace gr = gu::ros;


StoppingTrajectory::StoppingTrajectory() {}
StoppingTrajectory::~StoppingTrajectory() 
{
    writeToLog();
}

bool StoppingTrajectory::getNextReference(state_t &state, ros::Time &reference_time, float lookahead)
{
    control_arch::GetReferenceState service_call;

    reference_time = ros::Time::now() + ros::Duration(lookahead);
    service_call.request.time = reference_time;

    if (!get_reference_state_.call(service_call))
        return false;

    state.fromROS(service_call.response.state);
    return true;
}

// TODO: Don't think I'm using this, delete
void StoppingTrajectory::joystickCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    //std::cout << msg->buttons[8] << std::endl;
    ros::Time ref_time;
    state_t ref_state;
    float lookahead = 0;

    // if (!flagEnabledQ("teleop")) {
    //    return;
    //}

    if (msg->buttons[8])
    {
        if (!getNextReference(ref_state, ref_time, lookahead))
        {
            ROS_ERROR("reference not found; trajectory not generated");
            return;
        }
        std::cout << "Generate waypoints" << std::endl;
        generateCollisionFreeWaypoints(ref_state, ref_time);
    }
}

/** @brief Generates a stopping trajectory that is collision free. Publishes the trajectory.
 * */
void StoppingTrajectory::generateCollisionFreeWaypoints(state_t state, const ros::Time &reference_time)
{
    // Return if current velocity is very small
    if(state.vel.norm() < compute_thresh_) return;

    traj_.markers.clear();
    std::clock_t start;
    double compute_duration;
    double query_duration;;
    start = std::clock();
    
    // Set duration and interval
    float interval_step, num_intervals;

    // Get potential escape points
    std::vector<gu::Vec3> escapePoints;
    getEscapePoints(state.pos, state.vel, state.yaw(), escapePoints);
    // No escape points, generate stopping trajectory without goal point
    if (escapePoints.size() == 0) {
        std::cout << "No escape points, generate stopping trajectory without goal point" << std::endl;
        generateWaypoints(state, reference_time);
        return;
    }
    
    // Keep track of bad escape points and query time
    std::vector<gu::Vec3> invalidHODEscapePoints;
    std::vector<gu::Vec3> invalidCollisionEscapePoints;
    std::vector<double> queryDurations;

    // Check feasibility of each escape point
    for (uint i = 0; i < escapePoints.size(); i++)
    {
        std::clock_t query_start = std::clock();

        // Determine trajectory duration and time step interval
        double t_stop = 2.0; //getTrajectoryDuration(state, escapePoints[i]);
        ros::Duration duration(t_stop);
        num_intervals = 200;
        interval_step = t_stop/(num_intervals-1);

        // Get the polynomial coefficients of the trajectory. Check if within accel thresholds.
        std::vector<std::vector<double>> polyCoeffs;
        getPolynomialsWithPos(state, escapePoints[i], t_stop, polyCoeffs);
        //if (!checkTrajectoryHOD(polyCoeffs, duration, thresholds_)) {
        if (!checkAccelThreshold(polyCoeffs, duration, acc_threshold_)) { 
            // Trajectory not within the higher order derivative bounds
            invalidHODEscapePoints.push_back(escapePoints[i]);
            continue;
        }

        // Populate trajectory with waypoints along path. 
        std::vector<state_t> traj_wpts;
        populateTrajectory(reference_time, num_intervals, polyCoeffs, 
                            duration, interval_step, traj_wpts);
        bool isCollisionFree = checkTrajectoryCollisionGlobal(traj_wpts);
        
        query_duration = (std::clock() - query_start) / (double) CLOCKS_PER_SEC;
        queryDurations.push_back(query_duration);
        traj_.markers.push_back(getTrajectoryVis(traj_wpts));

        if (isCollisionFree) {
            // Visualize the bad escape points
            //visualizeEscapePoints(invalidHODEscapePoints, 1); //yellow
            //visualizeEscapePoints(invalidCollisionEscapePoints, 2); //red

            // Publish the collision free trajectory
            compute_duration = (std::clock() - start ) / (double) CLOCKS_PER_SEC;
            std::cout << "======== Found escape point. Total duration: " << compute_duration << "=================" << std::endl;

            double stddev_query_time;
            double avg_query = stats_utils::Average(queryDurations, &stddev_query_time);
            std::cout << "Average Query time: " << avg_query << ", std: " << stddev_query_time << std::endl;
            avg_query_time.push_back(avg_query);
            publishTrajectory(traj_wpts, interval_step, duration);
            publishTrajectoryVis();
            return;
        }
        
        invalidCollisionEscapePoints.push_back(escapePoints[i]);
        
    }
    publishTrajectoryVis();
    std::cout << "=========== Unable to find valid escape point ===============" << std::endl;
    // Visualize the bad escape points
    //visualizeEscapePoints(invalidHODEscapePoints, 1); //yellow
    //visualizeEscapePoints(invalidCollisionEscapePoints, 2); //red

    // Generate stopping trajectory with no goal position
    generateWaypoints(state, reference_time);
    compute_duration = (std::clock() - start ) / (double) CLOCKS_PER_SEC;

    double stddev_query_time;
    double avg_query = stats_utils::Average(queryDurations, &stddev_query_time);
    avg_query_time.push_back(avg_query);
}

/** @brief Generate and publishes a stopping trajectory. Doesn't check for collision.
*/
void StoppingTrajectory::generateWaypoints(state_t state, const ros::Time &reference_time)
{
    float interval = 0.01;
    int num_intervals = 101; //((float)duration.nsec)/interval + 1;
    ros::Duration duration(1);
    std::vector<std::vector<double>> polyCoeffs;
    getPolynomials(state, duration, polyCoeffs);
    std::vector<state_t> traj_wpts;
    populateTrajectory(reference_time, num_intervals, polyCoeffs, duration, interval, traj_wpts);
    publishTrajectory(traj_wpts, interval, duration);
}

void StoppingTrajectory::publishTrajectory(std::vector<state_t> traj_wpts, float interval, ros::Duration duration)
{
    control_arch::Waypoints msg;
    Waypoints traj(traj_wpts, interval);
    traj.toMessage(msg);

    msg.header.stamp = ros::Time::now();
    msg.duration = duration;
    msg.start_time = ros::Time::now();
    msg.interval = interval;
    msg.trajectory_options.required_flag = "teleop";

    wpts_pub.publish(msg);
}
} // namespace planner