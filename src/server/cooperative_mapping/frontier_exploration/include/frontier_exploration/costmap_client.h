// Copyright [2021] Ashwin A Nayar

#ifndef FRONTIER_EXPLORATION_COSTMAP_CLIENT_H
#define FRONTIER_EXPLORATION_COSTMAP_CLIENT_H

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace frontier_exploration
{
class Costmap2DClient
{
public:
    Costmap2DClient(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf::TransformListener* tf);

    geometry_msgs::Pose getRobotPose() const;

    costmap_2d::Costmap2D* getCostmap() { return &_costmap; }

    const std::string& getGlobalFrameID() const { return _globalFrame; }

protected:
    void updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

protected:
    costmap_2d::Costmap2D _costmap;
    const tf::TransformListener* const _tf;
    std::string _globalFrame;
    std::string _robotBaseFrame;
    double _tfTolerance;

private:
    ros::Subscriber _costmapSub;
    const std::array<uint8_t, 256> COST_TRANSLATION_TABLE;
};

}  // namespace frontier_exploration

#endif  // FRONTIER_EXPLORATION_COSTMAP_CLIENT_H
