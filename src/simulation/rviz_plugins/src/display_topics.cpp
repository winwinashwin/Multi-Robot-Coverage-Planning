#include <algorithm>
#include <map>
#include <memory>
#include <rviz_plugins/display_topics.hpp>
#include <string>
#include <utility>
#include <vector>

namespace rviz_plugins
{
DisplayTopics::DisplayTopics(QWidget* parent)
    : rviz::Panel(parent)
{
    connect(this, &DisplayTopics::enable, this, &DisplayTopics::setEnabled);
    Q_EMIT enable(false);
    setObjectName("Topics RViz plugin");
    setName(objectName());

    qRegisterMetaType<QMessageBox::Icon>();
    connect(this, &DisplayTopics::displayMessageBox, this, &DisplayTopics::displayMessageBoxHandler);

    layout_ = new QVBoxLayout();
    auto* scroll_widget = new QWidget;
    scroll_widget->setLayout(layout_);
    QScrollArea* scroll_area = new QScrollArea;
    scroll_area->setWidget(scroll_widget);
    scroll_area->setWidgetResizable(true);
    scroll_area->setFrameShape(QFrame::NoFrame);
    QVBoxLayout* main_layout = new QVBoxLayout(this);
    main_layout->addWidget(scroll_area);

    QPushButton* topics = new QPushButton("Topics");
    connect(topics, &QPushButton::clicked, this, &DisplayTopics::topics);

    QPushButton* settings = new QPushButton("Settings");
    settings->setIcon(QIcon::fromTheme("preferences-system"));
    connect(settings, &QPushButton::clicked, this, &DisplayTopics::settings);

    QHBoxLayout* buttons = new QHBoxLayout;
    buttons->addWidget(topics);
    buttons->addWidget(settings);
    layout_->addLayout(buttons);

    table_ = new QTableWidget;
    table_->insertColumn(0);
    table_->insertColumn(0);
    QStringList labels;
    labels.push_back("Topic");
    labels.push_back("Value");
    table_->setHorizontalHeaderLabels(labels);
    table_->setShowGrid(false);
    layout_->addWidget(table_);
    Q_EMIT enable(true);

    connect(table_->horizontalHeader(), &QHeaderView::sectionResized, this, &DisplayTopics::configChanged);
    connect(table_->verticalHeader(), &QHeaderView::sectionResized, this, &DisplayTopics::configChanged);
}

DisplayTopics::~DisplayTopics() { nh_.shutdown(); }

void DisplayTopics::load(const rviz::Config& config)
{
    rviz::Panel::load(config);

    unsigned i(0);
    while (1)
    {
        QString topic_name;
        QString topic_type;
        int refresh_rate_ms(0);

        if (!config.mapGetString("topic_" + QString::number(i) + "_name", &topic_name))
            break;

        if (!config.mapGetString("topic_" + QString::number(i) + "_type", &topic_type))
            break;

        // Not mandatory
        config.mapGetInt("topic_" + QString::number(i) + "_refresh_rate", &refresh_rate_ms);

        TopicDetails details;
        details.type = topic_type.toStdString();
        details.refresh_rate = ros::Duration(static_cast<double>(refresh_rate_ms) / 1e3);

        displayed_topics_.insert(std::pair<std::string, TopicDetails>(topic_name.toStdString(), details));
        ++i;
    }

    bool tmp_bool(false);
    if (config.mapGetBool("short_topic_names", &tmp_bool))
        short_topic_names_ = tmp_bool;
    else
        short_topic_names_ = false;

    if (!displayed_topics_.empty())
        updateTopicsDisplayed();

    QString table_header_state_str;
    if (config.mapGetString("horizontal_table_header", &table_header_state_str))
    {
        table_->horizontalHeader()->restoreState(QByteArray::fromHex(qPrintable(table_header_state_str)));
    }

    if (config.mapGetString("vertical_table_header", &table_header_state_str))
    {
        table_->verticalHeader()->restoreState(QByteArray::fromHex(qPrintable(table_header_state_str)));
    }
}

void DisplayTopics::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    unsigned i(0);
    for (auto topic : displayed_topics_)
    {
        config.mapSetValue("topic_" + QString::number(i) + "_name", QString::fromStdString(topic.first));
        config.mapSetValue("topic_" + QString::number(i) + "_type", QString::fromStdString(topic.second.type));
        config.mapSetValue("topic_" + QString::number(i) + "_refresh_rate", topic.second.refresh_rate.toNSec() / 1e6);
        ++i;
    }

    QByteArray table_header_state(table_->horizontalHeader()->saveState());
    // Must be saved as hex
    config.mapSetValue("horizontal_table_header", table_header_state.toHex());

    table_header_state = table_->verticalHeader()->saveState();
    // Must be saved as hex
    config.mapSetValue("vertical_table_header", table_header_state.toHex());

    config.mapSetValue("short_topic_names", short_topic_names_);
}

void DisplayTopics::topics()
{
    Q_EMIT enable(false);

    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics))
    {
        Q_EMIT displayMessageBox("Error getting topics", "Could not retrieve the topics names.", "",
                                 QMessageBox::Icon::Critical);
        Q_EMIT enable(true);
        return;
    }

    ros::master::V_TopicInfo supported_topics;
    for (auto topic : topics)
    {
        if (topic.datatype == "std_msgs/Bool" || topic.datatype == "std_msgs/Int8" ||
            topic.datatype == "std_msgs/UInt8" || topic.datatype == "std_msgs/Int16" ||
            topic.datatype == "std_msgs/UInt16" || topic.datatype == "std_msgs/Int32" ||
            topic.datatype == "std_msgs/UInt32" || topic.datatype == "std_msgs/Int64" ||
            topic.datatype == "std_msgs/UInt64" || topic.datatype == "std_msgs/Float32" ||
            topic.datatype == "std_msgs/Float64" || topic.datatype == "std_msgs/String" ||
            topic.datatype == "std_msgs/Time" || topic.datatype == "std_msgs/Duration")
            supported_topics.emplace_back(topic);
    }

    if (supported_topics.empty())
    {
        Q_EMIT enable(false);

        QDialog* no_topics_dialog = new QDialog(this);
        no_topics_dialog->setWindowTitle("No supported topic");
        QVBoxLayout* layout = new QVBoxLayout;
        no_topics_dialog->setLayout(layout);
        layout->addWidget(
            new QLabel("Error with topics, no supported topics found.\n"
                       "- Ok will clear the topics displayed\n- Cancel will not change the displayed topics"));

        QDialogButtonBox* button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        no_topics_dialog->layout()->addWidget(button_box);

        connect(button_box, &QDialogButtonBox::accepted, no_topics_dialog, &QDialog::accept);
        connect(button_box, &QDialogButtonBox::rejected, no_topics_dialog, &QDialog::reject);

        if (!no_topics_dialog->exec())
        {
            Q_EMIT enable(true);
            return;
        }

        displayed_topics_.clear();
        Q_EMIT configChanged();
        updateTopicsDisplayed();
        Q_EMIT enable(true);
        return;
    }

    QDialog* pick_topics_dialog = new QDialog(this);
    pick_topics_dialog->setWindowTitle("Pick displayed topics");

    QGridLayout* layout = new QGridLayout();
    QWidget* scroll_widget = new QWidget;
    scroll_widget->setLayout(layout);
    QScrollArea* scroll_area = new QScrollArea;
    scroll_area->setWidget(scroll_widget);
    scroll_area->setWidgetResizable(true);
    scroll_area->setFrameShape(QFrame::NoFrame);
    QVBoxLayout* dialog_layout = new QVBoxLayout(pick_topics_dialog);
    dialog_layout->addWidget(scroll_area);

    unsigned row(0), col(0);

    layout->addWidget(new QLabel("Only supported built-in types are displayed"), row, col);
    ++row;
    std::sort(supported_topics.begin(), supported_topics.end(),
              [](ros::master::TopicInfo a, ros::master::TopicInfo b) { return a.name < b.name; });

    std::vector<QCheckBox*> topic_buttons;
    for (auto topic : supported_topics)
    {
        QCheckBox* radio_button = new QCheckBox;

        if (displayed_topics_.find(topic.name) != displayed_topics_.end())
            radio_button->setChecked(true);

        topic_buttons.emplace_back(radio_button);
        radio_button->setText(QString::fromStdString(topic.name));
        radio_button->setObjectName(QString::fromStdString(topic.name));
        radio_button->setToolTip(QString::fromStdString(topic.datatype));
        layout->addWidget(radio_button, row, col);
        ++row;
        if (row > 20)
        {
            ++col;
            row = 1;  // Don't start at 0, it's the label
        }
    }

    QDialogButtonBox* button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(button_box, layout->rowCount() + 1, col);

    connect(button_box, &QDialogButtonBox::accepted, pick_topics_dialog, &QDialog::accept);
    connect(button_box, &QDialogButtonBox::rejected, pick_topics_dialog, &QDialog::reject);

    if (!pick_topics_dialog->exec())
    {
        Q_EMIT enable(true);
        return;
    }

    Q_EMIT configChanged();

    std::map<std::string, TopicDetails> displayed_topics_old(displayed_topics_);
    displayed_topics_.clear();
    for (auto button : topic_buttons)
    {
        if (!button->isChecked())
            continue;

        TopicDetails details;

        // If topic was already displayed before, copy its details
        for (auto& topic : displayed_topics_old)
        {
            if (topic.first == button->objectName().toStdString())
                details = topic.second;
        }

        details.type = button->toolTip().toStdString();
        displayed_topics_.insert(std::pair<std::string, TopicDetails>(button->objectName().toStdString(), details));
    }

    updateTopicsDisplayed();
    Q_EMIT enable(true);
}

void DisplayTopics::settings()
{
    Q_EMIT enable(false);

    QDialog* dialog(new QDialog(this));
    dialog->setWindowTitle("Display topics - Settings");
    QVBoxLayout* layout = new QVBoxLayout(dialog);

    QCheckBox* short_topic_names(new QCheckBox("Short topic names"));
    short_topic_names->setChecked(short_topic_names_);
    layout->addWidget(short_topic_names);

    // Topic refresh rate
    layout->addStretch(1);
    QLabel* maximum_refresh_rate(new QLabel("<b>Maximum refresh rate</b>"));
    layout->addWidget(maximum_refresh_rate);

    QGridLayout* grid(new QGridLayout);
    std::size_t i(0);
    std::vector<QSpinBox*> topic_spinboxes;
    for (auto& topic : displayed_topics_)
    {
        QLabel* topic_name(new QLabel(QString::fromStdString(topic.first)));
        topic_name->setToolTip(QString::fromStdString(topic.second.type));
        grid->addWidget(topic_name, i, 0);
        QSpinBox* refresh_rate(new QSpinBox);
        topic_spinboxes.emplace_back(refresh_rate);
        refresh_rate->setObjectName(QString::fromStdString(topic.first));
        refresh_rate->setToolTip("If set to zero the displayed value is updated every time a value is received.");
        refresh_rate->setSingleStep(100);
        refresh_rate->setRange(0, 10000);
        refresh_rate->setSuffix(" ms");
        grid->addWidget(refresh_rate, i, 1);
        ++i;

        // Retrieve value from the displayed topics
        for (auto& topic_info : topic_infos_)
        {
            if (topic_info->topic_name_ != topic.first)
                continue;
            refresh_rate->setValue(topic_info->maximumRefreshRate().toSec() * 1e3);
            break;
        }
    }

    layout->addLayout(grid);
    layout->addStretch(1);

    QDialogButtonBox* button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(button_box);

    connect(button_box, &QDialogButtonBox::accepted, dialog, &QDialog::accept);
    connect(button_box, &QDialogButtonBox::rejected, dialog, &QDialog::reject);

    if (!dialog->exec())
    {
        Q_EMIT enable(true);
        return;
    }

    short_topic_names_ = short_topic_names->isChecked();

    // Change refresh rates
    for (auto& spinbox : topic_spinboxes)
    {
        for (auto& topic : displayed_topics_)
        {
            if (topic.first != spinbox->objectName().toStdString())
                continue;

            const ros::Duration d(static_cast<double>(spinbox->value()) / 1e3);  // milliseconds to seconds
            topic.second.refresh_rate = d;
            break;
        }
    }

    updateTopicsDisplayed();
    Q_EMIT configChanged();
    Q_EMIT enable(true);
}

void DisplayTopics::updateTopicsDisplayed()
{
    table_->setRowCount(0);
    table_->setRowCount(displayed_topics_.size());

    topic_infos_.clear();
    for (auto topic : displayed_topics_)
    {
        std::shared_ptr<TopicInfo> topic_info(new TopicInfo(topic.first, topic.second.type, topic.second.refresh_rate));
        topic_infos_.emplace_back(topic_info);
    }

    unsigned i(0);
    for (auto topic_info : topic_infos_)
    {
        if (short_topic_names_)
        {
            // Names always start with /
            // Names never ends with /
            QString topic_name(topic_info->label_.get()->text());
            int n = topic_name.lastIndexOf("/");
            if (n != 0)
                topic_name.remove(0, n + 1);

            QLabel* topic_label(new QLabel(topic_name));
            table_->setCellWidget(i, 0, topic_label);
        }
        else
            table_->setCellWidget(i, 0, topic_info->label_.get());
        table_->setCellWidget(i, 1, topic_info->display_.get());
        ++i;
    }
}

void DisplayTopics::displayMessageBoxHandler(const QString title, const QString text, const QString info,
                                             const QMessageBox::Icon icon)
{
    const bool old_state(isEnabled());
    setEnabled(false);
    QMessageBox msg_box;
    msg_box.setWindowTitle(title);
    msg_box.setText(text);
    msg_box.setInformativeText(info);
    msg_box.setIcon(icon);
    msg_box.setStandardButtons(QMessageBox::Ok);
    msg_box.exec();
    setEnabled(old_state);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DisplayTopics, rviz::Panel)
