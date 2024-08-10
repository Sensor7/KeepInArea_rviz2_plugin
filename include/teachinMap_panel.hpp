#include <rviz_common/panel.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <QPushButton>
#include <QVBoxLayout>
#include <QDialog>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDialogButtonBox>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <example_interfaces/srv/set_bool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <stihl_nav_msgs/srv/post_polygon_to_map.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <Ogre.h>


namespace teachinmap_rviz_plugin
{
class TeachinMapPanel : public rviz_common::Panel
{
Q_OBJECT
public:
    TeachinMapPanel(QWidget *parent = 0);

protected:
    void onInitialize() override;

private Q_SLOTS:
    void onShowPolygonButtonClicked();
    void onDefinePolygonButtonClicked();
    void onAddCurrentPositionButtonClicked();

private:
    void postPolygon();
    void handleMouseClick(const Ogre::Vector3& point);
    rviz_common::Tool* getToolByName(const std::string& name);

    QPushButton *show_polygon_button_;
    QPushButton *define_polygon_button_;
    QPushButton *add_current_position_button_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub;
    rclcpp::Client<stihl_nav_msgs::srv::PostPolygonToMap>::SharedPtr polygon_client_;
    std::vector<geometry_msgs::msg::Point> polygon_points_;
    rviz_common::DisplayContext* context_;
};

TeachinMapPanel::TeachinMapPanel(QWidget *parent)
    : rviz_common::Panel(parent), node_(rclcpp::Node::make_shared("teachinMap_panel_node"))
{
    QVBoxLayout *layout = new QVBoxLayout;
    show_polygon_button_ = new QPushButton("Show Polygon");
    define_polygon_button_ = new QPushButton("Define Polygon");
    add_current_position_button_ = new QPushButton("Add Current Position");
    layout->addWidget(show_polygon_button_);
    layout->addWidget(define_polygon_button_);
    layout->addWidget(add_current_position_button_);
    setLayout(layout);

    connect(show_polygon_button_, SIGNAL(clicked()), this, SLOT(onShowPolygonButtonClicked()));
    connect(define_polygon_button_, SIGNAL(clicked()), this, SLOT(onDefinePolygonButtonClicked()));
    connect(add_current_position_button_, SIGNAL(clicked()), this, SLOT(onAddCurrentPositionButtonClicked()));

    polygon_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>("polygon_topic", 10);
    polygon_client_ = node_->create_client<stihl_nav_msgs::srv::PostPolygonToMap>("global_costmap/post_polygon");
    active_pub = node_->create_publisher<std_msgs::msg::Bool>("/restricted_area/active", 10);

    context_ = nullptr;
}

void TeachinMapPanel::onInitialize()
{
    context_ = getDisplayContext();
    context_->getToolManager()->setCurrentTool(getToolByName("Interact"));
}

void TeachinMapPanel::onShowPolygonButtonClicked()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    for (int i = 0; i < 3; ++i) {
        active_pub->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds(100)); // add delay to make sure
    }

    // auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
    // request->data = true;

    // polygon_client_->async_send_request(request);
}

void TeachinMapPanel::onDefinePolygonButtonClicked()
{
    polygon_points_.clear();
    QDialog dialog(this);
    QFormLayout form(&dialog);

    form.addRow(new QLabel("Enter Polygon Points:"));

    std::vector<QLineEdit*> pointEdits;
    for (int i = 0; i < 6; ++i) {
        QLineEdit* lineEdit = new QLineEdit(&dialog);
        pointEdits.push_back(lineEdit);
        form.addRow(QString("Point %1 (x,y,z):").arg(i + 1), lineEdit);
    }

    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, &dialog);
    form.addRow(&buttonBox);

    connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
    connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));

    if (dialog.exec() == QDialog::Accepted) {
        for (auto& edit : pointEdits) {
            if (edit->text().isEmpty()) break;
            QStringList coordinates = edit->text().split(",");
            if (coordinates.size() != 3) continue;
            geometry_msgs::msg::Point p;
            p.x = coordinates[0].toDouble();
            p.y = coordinates[1].toDouble();
            p.z = coordinates[2].toDouble();
            polygon_points_.push_back(p);
        }
        if (polygon_points_.size() >= 3) {
            postPolygon();
        }
    }
}

void TeachinMapPanel::handleMouseClick(const Ogre::Vector3& point)
{
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    polygon_points_.push_back(p);

}

void TeachinMapPanel::onAddCurrentPositionButtonClicked()
{

    geometry_msgs::msg::Point current_position;

    current_position.x = 1.0; // 这是一个示例
    current_position.y = 1.0; // 这是一个示例
    current_position.z = 0.0; // 这是一个示例

    polygon_points_.push_back(current_position);
    auto request = std::make_shared<stihl_nav_msgs::srv::PostPolygonToMap::Request>();
    request->action = "add_point";
    geometry_msgs::msg::Point32 p;
    p.x = current_position.x;
    p.y = current_position.y;
    p.z = current_position.z;
    request->polygon.points.push_back(p);

    auto result = polygon_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Current position added successfully");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to add current position");
    }
}

void TeachinMapPanel::postPolygon()
{
    auto request = std::make_shared<stihl_nav_msgs::srv::PostPolygonToMap::Request>();
    request->action = "update_polygon";

    for (const auto& point : polygon_points_) {
        geometry_msgs::msg::Point32 p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        request->polygon.points.push_back(p);
    }

    auto result = polygon_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Polygon sent successfully");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send polygon");
    }
}

rviz_common::Tool* TeachinMapPanel::getToolByName(const std::string& name)
{
    auto tool_manager = context_->getToolManager();
    for (int i = 0; i < tool_manager->numTools(); ++i) {
        auto tool = tool_manager->getTool(i);
        if (tool->getName() == QString::fromStdString(name)) {
            return tool;
        }
    }
    return nullptr;
}

} // namespace teachinmap_rviz_plugin

#include "teachinMap_panel.moc"
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(teachinmap_rviz_plugin::TeachinMapPanel, rviz_common::Panel)
