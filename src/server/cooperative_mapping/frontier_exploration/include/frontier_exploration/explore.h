// Copyright [2021] Ashwin A Nayar

#ifndef FRONTIER_EXPLORATION_EXPLORE_H
#define FRONTIER_EXPLORATION_EXPLORE_H

#include "frontier_exploration/costmap_client.h"
#include "frontier_exploration/frontier_search.h"

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <move_base_msgs/MoveBaseAction.h>
#include <mutex>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <string>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

namespace frontier_exploration
{
class Explore
{
public:
    Explore(ros::NodeHandle* nh, ros::NodeHandle* pnh);
    ~Explore();

    void start();
    void stop();

private:
    void makePlan();

    void visualizeFrontiers(const std::vector<Frontier>& frontiers);

    void reachedGoal(const actionlib::SimpleClientGoalState& status,
                     const move_base_msgs::MoveBaseResultConstPtr& result, const geometry_msgs::Point& frontier_goal);

    bool goalOnBlacklist(const geometry_msgs::Point& goal);

    bool onExplorationStart(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool onExplorationAbort(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

private:
    ros::NodeHandle* _pnh;
    ros::NodeHandle* _nh;
    ros::Publisher _markerPub;
    tf::TransformListener _tf;
    std::atomic<bool> _active;

    Costmap2DClient _costmapClient;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _mbClient;
    frontier_exploration::FrontierSearch _search;
    ros::Timer _explorationTimer;
    ros::Timer _oneShotTimer;

    ros::ServiceServer _explorationStartSrv, _explorationAbortSrv;
    std::vector<geometry_msgs::Point> _frontierBlacklist;
    geometry_msgs::Point _prevGoal;
    double _prevDistance;
    ros::Time _lastProgress;

    // parameters
    double _plannerFrequency;
    double _potentialScale, _gainScale;
    ros::Duration _progressTimeout;
    bool _visualize;
};
}  // namespace frontier_exploration

#endif  // FRONTIER_EXPLORATION_EXPLORE_H
