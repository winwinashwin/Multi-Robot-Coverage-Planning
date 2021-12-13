#include <memory>
#include <rviz_plugins/topic_info.hpp>
#include <string>
#include <utility>

namespace rviz_plugins
{

TopicInfo::TopicInfo(const std::string& topic_name, const std::string& topic_type, ros::Duration refresh_duration)
    : topic_name_(topic_name)
    , topic_type_(topic_type)
    , refresh_duration_(refresh_duration)
{
    // Force refresh
    last_ = ros::Time::now() - refresh_duration_;

    label_ = std::make_shared<QLabel>(QString::fromStdString(topic_name_));
    label_->setToolTip(QString::fromStdString(topic_type_));

    if (topic_type_ == "std_msgs/Bool" || topic_type_ == "std_msgs/Duration" || topic_type_ == "std_msgs/String" ||
        topic_type_ == "std_msgs/Time")
    {
        display_ = std::make_shared<QLabel>();
    }
    else if (topic_type_ == "std_msgs/Int8" || topic_type_ == "std_msgs/Int16" || topic_type_ == "std_msgs/Int32" ||
             topic_type_ == "std_msgs/Int64" || topic_type_ == "std_msgs/UInt8" || topic_type_ == "std_msgs/UInt16" ||
             topic_type_ == "std_msgs/UInt32" || topic_type_ == "std_msgs/UInt64" ||
             topic_type_ == "std_msgs/Float32" || topic_type_ == "std_msgs/Float64")
    {
        display_ = std::make_shared<QLCDNumber>();
        std::static_pointer_cast<QLCDNumber>(display_)->setFrameShape(QFrame::Shape::NoFrame);
    }
    else
    {
        ROS_ERROR_STREAM("Unsupported built-in type, cannot display topic " << topic_name_ << " of type "
                                                                            << topic_type_);
        display_ = std::make_shared<QLabel>("Unsupported topic type");
        return;
    };

    if (topic_type == "std_msgs/Bool")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::boolCallback, this);
    else if (topic_type == "std_msgs/Duration")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::durationCallback, this);
    else if (topic_type == "std_msgs/Float32")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::float32Callback, this);
    else if (topic_type == "std_msgs/Float64")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::float64Callback, this);
    else if (topic_type == "std_msgs/Int8")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::int8Callback, this);
    else if (topic_type == "std_msgs/Int16")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::int16Callback, this);
    else if (topic_type == "std_msgs/Int32")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::int32Callback, this);
    else if (topic_type == "std_msgs/Int64")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::int64Callback, this);
    else if (topic_type == "std_msgs/String")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::stringCallback, this);
    else if (topic_type == "std_msgs/Time")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::timeCallback, this);
    else if (topic_type == "std_msgs/UInt8")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::uint8Callback, this);
    else if (topic_type == "std_msgs/UInt16")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::uint16Callback, this);
    else if (topic_type == "std_msgs/UInt32")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::uint32Callback, this);
    else if (topic_type == "std_msgs/UInt64")
        sub_ = nh_.subscribe(topic_name_, 1, &TopicInfo::uint64Callback, this);
    else
    {
        ROS_ERROR_STREAM("Could not find callback for topic type " << topic_type_);
        return;
    }

    connect(this, &TopicInfo::adjustLCDNumberOfDigits, this, &TopicInfo::adjustLCDNumberOfDigitsHandler);
}

TopicInfo::~TopicInfo() { nh_.shutdown(); }

void TopicInfo::setMaximumRefreshRate(const ros::Duration d)
{
    refresh_duration_ = d;
    last_ = ros::Time::now() - refresh_duration_;  // Force update of the value
}

ros::Duration TopicInfo::maximumRefreshRate() { return refresh_duration_; }

bool TopicInfo::refresh()
{
    ros::Time now(ros::Time::now());

    if (refresh_duration_.isZero())
        return true;

    if ((now - refresh_duration_) >= last_)
    {
        last_ = now;
        return true;
    }
    else
        return false;
}

void TopicInfo::adjustLCDNumberOfDigitsHandler(uint32_t number) const
{
    std::shared_ptr<QLCDNumber> lcd(std::dynamic_pointer_cast<QLCDNumber>(display_));
    if (!lcd)
        return;

    if (number < 100)
        lcd->setDigitCount(5);
    else if (number < 10000)
        lcd->setDigitCount(7);
    else if (number < 1000000)
        lcd->setDigitCount(9);
    else if (number < 100000000)
        lcd->setDigitCount(11);
    else
        lcd->setDigitCount(20);
}

void TopicInfo::boolCallback(const std_msgs::BoolConstPtr& msg)
{
    if (!refresh())
        return;

    QString label("False");
    if (msg->data)
        label = "True";

    std::static_pointer_cast<QLabel>(display_)->setText(label);
}

void TopicInfo::durationCallback(const std_msgs::DurationConstPtr& msg)
{
    if (!refresh())
        return;

    QTime t(0, 0, 0, 0);
    t = t.addSecs(msg->data.sec);
    t = t.addMSecs(msg->data.nsec / 1e6);
    std::static_pointer_cast<QLabel>(display_)->setText(t.toString("hh:mm:ss:zzz"));
}

void TopicInfo::float32Callback(const std_msgs::Float32ConstPtr& msg)
{
    if (!refresh())
        return;

    uint32_t digits_counting(msg->data > 0 ? msg->data : -msg->data);
    Q_EMIT adjustLCDNumberOfDigits(digits_counting);
    std::static_pointer_cast<QLCDNumber>(display_)->display(msg->data);
}

void TopicInfo::float64Callback(const std_msgs::Float64ConstPtr& msg)
{
    if (!refresh())
        return;

    uint32_t digits_counting(msg->data > 0 ? msg->data : -msg->data);
    Q_EMIT adjustLCDNumberOfDigits(digits_counting);
    std::static_pointer_cast<QLCDNumber>(display_)->display(msg->data);
}

void TopicInfo::int8Callback(const std_msgs::Int8ConstPtr& msg)
{
    if (!refresh())
        return;

    uint32_t digits_counting(msg->data > 0 ? msg->data : -msg->data);
    Q_EMIT adjustLCDNumberOfDigits(digits_counting);
    std::static_pointer_cast<QLCDNumber>(display_)->display(msg->data);
}

void TopicInfo::int16Callback(const std_msgs::Int16ConstPtr& msg)
{
    if (!refresh())
        return;

    uint32_t digits_counting(msg->data > 0 ? msg->data : -msg->data);
    Q_EMIT adjustLCDNumberOfDigits(digits_counting);
    std::static_pointer_cast<QLCDNumber>(display_)->display(msg->data);
}

void TopicInfo::int32Callback(const std_msgs::Int32ConstPtr& msg)
{
    if (!refresh())
        return;

    uint32_t digits_counting(msg->data > 0 ? msg->data : -msg->data);
    Q_EMIT adjustLCDNumberOfDigits(digits_counting);
    std::static_pointer_cast<QLCDNumber>(display_)->display(msg->data);
}

void TopicInfo::int64Callback(const std_msgs::Int64ConstPtr& msg)
{
    if (!refresh())
        return;

    uint32_t digits_counting(msg->data > 0 ? msg->data : -msg->data);
    Q_EMIT adjustLCDNumberOfDigits(digits_counting);
    int32_t casted(msg->data);
    std::static_pointer_cast<QLCDNumber>(display_)->display(casted);
}

void TopicInfo::stringCallback(const std_msgs::StringConstPtr& msg)
{
    if (!refresh())
        return;

    std::static_pointer_cast<QLabel>(display_)->setText(QString::fromStdString(msg->data));
}

void TopicInfo::timeCallback(const std_msgs::TimeConstPtr& msg)
{
    if (!refresh())
        return;

    QDateTime time;
    time.setMSecsSinceEpoch(msg->data.sec * 1e3 + msg->data.nsec / 1e6);
    std::static_pointer_cast<QLabel>(display_)->setText(time.toString(Qt::DateFormat::ISODate));
}

void TopicInfo::uint8Callback(const std_msgs::UInt8ConstPtr& msg)
{
    if (!refresh())
        return;

    Q_EMIT adjustLCDNumberOfDigits(msg->data);
    std::static_pointer_cast<QLCDNumber>(display_)->display(msg->data);
}

void TopicInfo::uint16Callback(const std_msgs::UInt16ConstPtr& msg)
{
    if (!refresh())
        return;

    Q_EMIT adjustLCDNumberOfDigits(msg->data);
    std::static_pointer_cast<QLCDNumber>(display_)->display(msg->data);
}

void TopicInfo::uint32Callback(const std_msgs::UInt32ConstPtr& msg)
{
    if (!refresh())
        return;

    uint16_t casted(msg->data);
    Q_EMIT adjustLCDNumberOfDigits(casted);
    std::static_pointer_cast<QLCDNumber>(display_)->display(casted);
}

void TopicInfo::uint64Callback(const std_msgs::UInt64ConstPtr& msg)
{
    if (!refresh())
        return;

    uint16_t casted(msg->data);
    Q_EMIT adjustLCDNumberOfDigits(casted);
    std::static_pointer_cast<QLCDNumber>(display_)->display(casted);
}

}  // namespace rviz_plugins
