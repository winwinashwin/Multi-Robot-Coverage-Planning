#include "rviz_plugins/gazebo_physics_gui.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

namespace rviz_plugins
{
GazeboPhysicsGui::GazeboPhysicsGui(QWidget* parent)
    : rviz::Panel(parent)
{
    // Create a push button
    _btnPause = new QPushButton(this);
    _btnPause->setText("Pause");
    connect(_btnPause, SIGNAL(clicked()), this, SLOT(onButtonPause()));

    // Create a push button
    _btnUnpause = new QPushButton(this);
    _btnUnpause->setText("Unpause");
    connect(_btnUnpause, SIGNAL(clicked()), this, SLOT(onButtonUnpause()));

    // Horizontal Layout
    auto* hlayout = new QHBoxLayout;
    hlayout->addWidget(_btnPause);
    hlayout->addWidget(_btnUnpause);

    // Vertical layout
    auto* layout = new QVBoxLayout;
    layout->addLayout(hlayout);
    setLayout(layout);

    updateBtnStatus();
}

void GazeboPhysicsGui::onButtonPause()
{
    _gzRemote.pausePhysics();
    updateBtnStatus();
}

void GazeboPhysicsGui::onButtonUnpause()
{
    _gzRemote.unpausePhysics();
    updateBtnStatus();
}

void GazeboPhysicsGui::save(rviz::Config config) const { rviz::Panel::save(config); }

void GazeboPhysicsGui::load(const rviz::Config& config) { rviz::Panel::load(config); }

void GazeboPhysicsGui::updateBtnStatus()
{
    _btnPause->setEnabled(!_gzRemote.isPhysicsPaused());
    _btnUnpause->setEnabled(_gzRemote.isPhysicsPaused());
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::GazeboPhysicsGui, rviz::Panel)
