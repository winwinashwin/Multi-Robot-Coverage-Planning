#ifndef RVIZ_PLUGINS_GAZEBO_PHYSICS_GUI_HPP
#define RVIZ_PLUGINS_GAZEBO_PHYSICS_GUI_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QComboBox>
#include <QPushButton>
#include <rviz_plugins/gazebo_remote.hpp>

class QLineEdit;
class QSpinBox;

namespace rviz_plugins
{
class GazeboPhysicsGui : public rviz::Panel
{
    Q_OBJECT
public:
    explicit GazeboPhysicsGui(QWidget* parent = nullptr);

    void load(const rviz::Config& config) override;
    void save(rviz::Config config) const override;

public Q_SLOTS:

protected Q_SLOTS:
    void onButtonPause();
    void onButtonUnpause();

protected:
    void updateBtnStatus();

protected:
    QPushButton* _btnPause;
    QPushButton* _btnUnpause;

    GazeboRemote _gzRemote;
};

}  // namespace rviz_plugins

#endif  // RVIZ_PLUGINS_GAZEBO_PHYSICS_GUI_HPP
