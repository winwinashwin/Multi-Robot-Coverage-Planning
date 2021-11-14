#ifndef RVIZ_PLUGINS_TOPIC_INFO_HPP
#define RVIZ_PLUGINS_TOPIC_INFO_HPP

#include <QLCDNumber>
#include <QLabel>
#include <QTime>
#include <memory>
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
#include <string>

namespace rviz_plugins
{

class TopicInfo : public QObject
{
    Q_OBJECT

public:
    TopicInfo(const std::string& topic_name, const std::string& topic_type,
              ros::Duration refresh_duration = ros::Duration(0));

    ~TopicInfo() override;

    __attribute__((unused)) void setMaximumRefreshRate(ros::Duration d);
    ros::Duration maximumRefreshRate();

    const std::string topic_name_;
    const std::string topic_type_;
    std::shared_ptr<QLabel> label_;
    std::shared_ptr<QWidget> display_;

Q_SIGNALS:
    void adjustLCDNumberOfDigits(uint32_t number);

protected:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

private Q_SLOTS:
    void adjustLCDNumberOfDigitsHandler(uint32_t number) const;

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

}  // namespace rviz_plugins

#endif  // RVIZ_PLUGINS_TOPIC_INFO_HPP
