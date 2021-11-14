#ifndef RVIZ_PLUGINS_GAZEBO_REMOTE_HPP
#define RVIZ_PLUGINS_GAZEBO_REMOTE_HPP

#include <gazebo_msgs/GetPhysicsProperties.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

namespace rviz_plugins
{
class GazeboRemote
{
public:
    GazeboRemote()
    {
        _pauseClient = _nh.serviceClient<std_srvs::Empty>("gazebo/pause_physics");
        _unpauseClient = _nh.serviceClient<std_srvs::Empty>("gazebo/unpause_physics");
        _physicsPropertiesClient =
            _nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("gazebo/get_physics_properties");

        updatePhysicsStatus();
    }

    void updatePhysicsStatus()
    {
        gazebo_msgs::GetPhysicsProperties srv;
        if (!_physicsPropertiesClient.call(srv))
        {
            ROS_ERROR("Cannot communicate with gazebo!");
            return;
        }
        _physicsPaused = srv.response.pause;
    }

    bool isPhysicsPaused() const { return _physicsPaused; }

    void pausePhysics()
    {
        std_srvs::Empty srv;
        if (!_pauseClient.call(srv))
        {
            ROS_ERROR("Cannot communicate with gazebo!");
            return;
        }
        updatePhysicsStatus();
    }

    void unpausePhysics()
    {
        std_srvs::Empty srv;
        if (!_unpauseClient.call(srv))
        {
            ROS_ERROR("Cannot communicate with gazebo!");
            return;
        }
        updatePhysicsStatus();
    }

protected:
    // service clients
    ros::ServiceClient _pauseClient, _unpauseClient, _physicsPropertiesClient;

    // The ROS node handle.
    ros::NodeHandle _nh;

    bool _physicsPaused = false;
};

}  // namespace rviz_plugins

#endif  // RVIZ_PLUGINS_GAZEBO_REMOTE_HPP
