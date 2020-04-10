#include <stopping_trajectory/StoppingTrajectory.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "stopping_trajectory");
    ros::NodeHandle node("~");

    planner::StoppingTrajectory st;

    if(!st.initialize(node))
    {
        ROS_ERROR("%s: failed to initialize stopping trajectory", ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }

    ros::spin();

    return EXIT_SUCCESS;
}
