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
}  // namespace teachinmap_rviz_plugin
