/**
 * A simple ROS node to publish Realtime factor in gazebo to a topic.
 *
 * This is later displayed in rviz using the custom plugin from the `rviz_plugins` package.
 */

#include <gazebo_msgs/PerformanceMetrics.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

using Float64 = std_msgs::Float64;
using PerformanceMetrics = gazebo_msgs::PerformanceMetrics;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gz_realtime_factor_node");
    ros::NodeHandle nh;

    auto publisher = nh.advertise<Float64>("realtime_factor", 10);
    auto subscriber = nh.subscribe<PerformanceMetrics>("performance_metrics", 10,
                                                       [&publisher](const PerformanceMetrics::ConstPtr& msg)
                                                       {
                                                           auto pubMsg = Float64();
                                                           pubMsg.data = msg->real_time_factor;
                                                           publisher.publish(pubMsg);
                                                       });  // NOLINT
    ros::spin();
    return EXIT_SUCCESS;
}
