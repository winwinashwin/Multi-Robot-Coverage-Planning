#ifndef TOPICS_RVIZ_PLUGIN_DISPLAY_TOPICS_HPP
#define TOPICS_RVIZ_PLUGIN_DISPLAY_TOPICS_HPP

#ifndef Q_MOC_RUN
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <topics_rviz_plugin/topic_info.hpp>
#endif

#include <QApplication>
#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QHeaderView>
#include <QLCDNumber>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QtGlobal>

namespace topics_rviz_plugin
{

class DisplayTopics : public rviz::Panel
{
    Q_OBJECT

public:
    DisplayTopics(QWidget* parent = NULL);
    virtual ~DisplayTopics();

    struct TopicDetails
    {
        std::string type;
        ros::Duration refresh_rate;
    };

Q_SIGNALS:
    void enable(const bool);
    void displayMessageBox(const QString, const QString, const QString, const QMessageBox::Icon);

protected Q_SLOTS:
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

    void topics();
    void settings();
    void updateTopicsDisplayed();

    void displayMessageBoxHandler(const QString title, const QString text, const QString info = "",
                                  const QMessageBox::Icon icon = QMessageBox::Icon::Information);

protected:
    ros::NodeHandle nh_;

    bool short_topic_names_ = false;
    QVBoxLayout* layout_;
    QTableWidget* table_;
    std::map<std::string, TopicDetails> displayed_topics_;
    std::vector<std::shared_ptr<TopicInfo>> topic_infos_;
};

}  // namespace topics_rviz_plugin

#endif
