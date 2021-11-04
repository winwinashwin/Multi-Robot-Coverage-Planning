// Copyright [2021] Ashwin A Nayar

#include "frontier_exploration/explore.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

    ros::NodeHandle nh, pnh("~");
    frontier_exploration::Explore explore(&nh, &pnh);
    ros::spin();
}
