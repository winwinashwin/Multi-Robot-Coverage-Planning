#ifndef TOPICS_RVIZ_PLUGIN_TOPIC_INFO_HPP
#define TOPICS_RVIZ_PLUGIN_TOPIC_INFO_HPP

#include <QLCDNumber>
#include <QLabel>
#include <QTime>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>

namespace topics_rviz_plugin
{

class TopicInfo : public QObject
{
    Q_OBJECT

public:
    TopicInfo(const std::string topic_name, const std::string topic_type,
              const ros::Duration refresh_duration = ros::Duration(0));

    virtual ~TopicInfo();

    void setMaximumRefreshRate(const ros::Duration d);
    ros::Duration maximumRefreshRate();

    const std::string topic_name_;
    const std::string topic_type_;
    std::shared_ptr<QLabel> label_;
    std::shared_ptr<QWidget> display_;

Q_SIGNALS:
    void adjustLCDNumberOfDigits(const long unsigned number);

protected:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

private Q_SLOTS:
    void adjustLCDNumberOfDigitsHandler(const long unsigned number);

private:
    ros::Time last_ = ros::Time::now();
    ros::Duration refresh_duration_;

    bool refresh();

    void boolCallback(const std_msgs::BoolConstPtr& msg);
    void durationCallback(const std_msgs::DurationConstPtr& msg);
    void float32Callback(const std_msgs::Float32ConstPtr& msg);
    void float64Callback(const std_msgs::Float64ConstPtr& msg);
    void int8Callback(const std_msgs::Int8ConstPtr& msg);
    void int16Callback(const std_msgs::Int16ConstPtr& msg);
    void int32Callback(const std_msgs::Int32ConstPtr& msg);
    void int64Callback(const std_msgs::Int64ConstPtr& msg);
    void stringCallback(const std_msgs::StringConstPtr& msg);
    void timeCallback(const std_msgs::TimeConstPtr& msg);
    void uint8Callback(const std_msgs::UInt8ConstPtr& msg);
    void uint16Callback(const std_msgs::UInt16ConstPtr& msg);
    void uint32Callback(const std_msgs::UInt32ConstPtr& msg);
    void uint64Callback(const std_msgs::UInt64ConstPtr& msg);
};

}  // namespace topics_rviz_plugin

#endif
