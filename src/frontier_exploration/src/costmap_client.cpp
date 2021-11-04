// Copyright [2021] Ashwin A Nayar

#include "frontier_exploration/costmap_client.h"

#include <functional>
#include <mutex>
#include <string>

#define ATTRIBUTE_UNUSED(expr) (void)(expr)

namespace frontier_exploration
{

static std::array<uint8_t, 256> initTranslationTable()
{
    std::array<uint8_t, 256> table = {};

    for (size_t i = 0; i < table.size(); i++)
    {
        table[i] = static_cast<uint8_t>(1 + (251 * (i - 1)) / 97);
    }

    table[0] = 0;                           // NO obstacle
    table[99] = 253;                        // INSCRIBED obstacle
    table[100] = 254;                       // LETHAL obstacle
    table[static_cast<uint8_t>(-1)] = 255;  // UNKNOWN

    return table;
}

Costmap2DClient::Costmap2DClient(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf::TransformListener* tf)
    : _tf(tf)
    , _tfTolerance(0.3)
    , COST_TRANSLATION_TABLE(initTranslationTable())
{
    std::string costmapTopic("costmap");
    std::string footprint;
    pnh.param("robot_base_frame", _robotBaseFrame, std::string("base_link"));
    // transform tolerance is used for all tf transforms here
    pnh.param("transform_tolerance", _tfTolerance, 0.3);

    _costmapSub = nh.subscribe<nav_msgs::OccupancyGrid>(costmapTopic, 1000, &Costmap2DClient::updateFullMap, this);
    ATTRIBUTE_UNUSED(_costmapSub);
    ROS_INFO_STREAM("Waiting for costmap to become available on topic: " << costmapTopic);
    auto costmapMsg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(costmapTopic, nh);
    updateFullMap(costmapMsg);

    std::string tfPrefix = tf::getPrefixParam(pnh);
    _robotBaseFrame = tf::resolve(tfPrefix, _robotBaseFrame);

    ros::Time lastError = ros::Time::now();
    std::string tfError;
    while (ros::ok() && !_tf->waitForTransform(_globalFrame, _robotBaseFrame, ros::Time(), ros::Duration(0.1),
                                               ros::Duration(0.01), &tfError))
    {
        ros::spinOnce();
        if (lastError + ros::Duration(5.0) < ros::Time::now())
        {
            ROS_WARN(
                "Timed out waiting for transform from %s to %s to become available "
                "before subscribing to costmap, tf error: %s",
                _robotBaseFrame.c_str(), _globalFrame.c_str(), tfError.c_str());
            lastError = ros::Time::now();
        }

        tfError.clear();
    }
}

void Costmap2DClient::updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    _globalFrame = msg->header.frame_id;

    const size_t sizeCellsX = msg->info.width;
    const size_t sizeCellsY = msg->info.height;
    double resolution = msg->info.resolution;
    double originX = msg->info.origin.position.x;
    double originY = msg->info.origin.position.y;

    ROS_DEBUG("Received full new map, resizing to: (%lu, %lu)", sizeCellsX, sizeCellsY);
    _costmap.resizeMap(sizeCellsX, sizeCellsY, resolution, originX, originY);

    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*_costmap.getMutex());

    uint8_t* costmapData = _costmap.getCharMap();
    size_t costmapSize = _costmap.getSizeInCellsX() * _costmap.getSizeInCellsY();
    ROS_DEBUG("Full map update, %lu values", costmapSize);
    for (size_t i = 0; i < costmapSize && i < msg->data.size(); ++i)
    {
        auto cellCost = static_cast<uint8_t>(msg->data[i]);
        costmapData[i] = COST_TRANSLATION_TABLE[cellCost];
    }
    ROS_DEBUG("Map updated, written %lu values", costmapSize);
}

geometry_msgs::Pose Costmap2DClient::getRobotPose() const
{
    tf::Stamped<tf::Pose> globalPose;
    globalPose.setIdentity();
    tf::Stamped<tf::Pose> robotPose;
    robotPose.setIdentity();
    robotPose.frame_id_ = _robotBaseFrame;
    robotPose.stamp_ = ros::Time();
    ros::Time currentTime = ros::Time::now();

    // get the global pose of the robot
    try
    {
        _tf->transformPose(_globalFrame, robotPose, globalPose);
    }
    catch (tf::LookupException& ex)
    {
        ROS_ERROR_THROTTLE(1.0,
                           "No Transform available Error looking up robot "
                           "pose: %s\n",
                           ex.what());
        return {};
    }
    catch (tf::ConnectivityException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
        return {};
    }
    catch (tf::ExtrapolationException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
        return {};
    }
    // check global_pose timeout
    if (currentTime.toSec() - globalPose.stamp_.toSec() > _tfTolerance)
    {
        ROS_WARN_THROTTLE(1.0,
                          "Costmap2DClient transform timeout. Current time: "
                          "%.4f, global_pose stamp: %.4f, tolerance: %.4f",
                          currentTime.toSec(), globalPose.stamp_.toSec(), _tfTolerance);
        return {};
    }

    geometry_msgs::PoseStamped msg;
    tf::poseStampedTFToMsg(globalPose, msg);
    return msg.pose;
}

}  // namespace frontier_exploration
