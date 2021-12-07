#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <tf/transform_listener.h>
#include <vector>

#define ROS_UNUSED(expr)                                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        (void)(expr);                                                                                                  \
    } while (0)  // NOLINT

class CoverageProgressNode
{
public:
    CoverageProgressNode()
        : _pnh("~")
    {
        initializeMap();

        _gridPub = _nh.advertise<nav_msgs::OccupancyGrid>("coverage_grid", 1);
        _resetSrv = _nh.advertiseService("reset", &CoverageProgressNode::reset, this);
        ROS_UNUSED(_resetSrv);  // should be alive for server to be active
        _pnh.param("rate", _rate, 0.5);
        _updateTimer =
            _nh.createTimer(ros::Duration(1.0 / _rate), [this](const ros::TimerEvent&) { updateCallback(); });
        ROS_UNUSED(_updateTimer);  // should be alive for timer to be active
    }

private:
    void initializeMap()
    {
        _pnh.param("grid/width", _covArea.x, 10.0);
        _pnh.param("grid/height", _covArea.y, 10.0);
        _pnh.getParam("coverage/radius", _covRadiusMeters);
        _pnh.param("grid/resolution", _covResolution, 0.05);
        _pnh.param("coverage/effectivity", _covEffectivity, 0.5);
        _pnh.getParam("map_frame", _mapFrame);
        _pnh.getParam("coverage_frame", _covFrame);

        _covRadiusMeters += 2 * _covResolution;  // compensate for discretization
        _covRadiusCells = static_cast<int>(_covRadiusMeters / _covResolution);

        _grid = nav_msgs::OccupancyGrid();
        _grid.info.resolution = static_cast<float>(_covResolution);
        _grid.info.width = abs(static_cast<int>(_covArea.x / _covResolution));
        _grid.info.height = abs(static_cast<int>(_covArea.y / _covResolution));

        _pnh.param("grid/origin/x", _grid.info.origin.position.x, -5.0);
        _pnh.param("grid/origin/y", _grid.info.origin.position.y, -5.0);
        _grid.info.origin.orientation.w = 1;

        _grid.data = std::vector<int8_t>(_grid.info.width * _grid.info.height, 0);
    }

    void updateCallback()
    {
        tf::StampedTransform trans;
        try
        {
            _listener.lookupTransform(_mapFrame, _covFrame, ros::Time(), trans);
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN_STREAM_THROTTLE(2, "Transform lookup error from " << _covFrame << "to " << _mapFrame);
            return;
        }
        auto xPoint = static_cast<int>((trans.getOrigin().getX() - _grid.info.origin.position.x) / _covResolution);
        auto yPoint = static_cast<int>((trans.getOrigin().getY() - _grid.info.origin.position.y) / _covResolution);

        _grid.header.frame_id = _mapFrame;

        for (int i = 0; i < 2 * _covRadiusCells; ++i)
        {
            for (int j = 0; j < 2 * _covRadiusCells; ++j)
            {
                auto xIndex = j - _covRadiusCells;
                auto yIndex = i - _covRadiusCells;

                auto arrayIndex = xPoint + xIndex + _grid.info.width * (yPoint + yIndex);
                bool cellInCovCircle = xIndex * xIndex + yIndex * yIndex < _covRadiusCells * _covRadiusCells;
                bool cellInGrid = true;
                cellInGrid &= xPoint + xIndex >= 0;
                cellInGrid &= xPoint + xIndex <= static_cast<int>(_covArea.x / _covResolution);
                cellInGrid &= yPoint + yIndex >= 0;
                cellInGrid &= yPoint + yIndex <= static_cast<int>(_covArea.y / _covResolution);

                if (cellInCovCircle && cellInGrid)
                {
                    _grid.data[arrayIndex] = 100 * _covEffectivity;
                }
                else
                {
                    ROS_DEBUG("xPoint %i, yPoint %i, xMeas %f, yMeas %f", xPoint, yPoint, trans.getOrigin().getX(),
                              trans.getOrigin().getY());
                }
            }
        }
        _gridPub.publish(_grid);
    }

    bool reset(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
    {
        static constexpr auto msg = "Reset coverage progress and grid";
        ROS_INFO(msg);
        initializeMap();
        response.success = true;
        response.message = msg;
        return true;
    }

private:
    double _covResolution{};
    double _covEffectivity{};
    double _covRadiusMeters{};
    int _covRadiusCells{};
    struct
    {
        double x, y;
    } _covArea{};
    std::string _mapFrame, _covFrame;

    ros::NodeHandle _nh, _pnh;
    tf::TransformListener _listener;
    ros::Publisher _gridPub;
    nav_msgs::OccupancyGrid _grid;
    ros::ServiceServer _resetSrv;
    double _rate{};
    ros::Timer _updateTimer;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coverage_progress");
    CoverageProgressNode node;
    ros::spin();
    return EXIT_SUCCESS;
}
