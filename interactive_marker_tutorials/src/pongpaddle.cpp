#include <cstdio>
#include <thread>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "tf2/LinearMath/Quaternion.h"

static const float FIELD_WIDTH = 12.0;
static const float FIELD_HEIGHT = 8.0;
static const float BORDER_SIZE = 0.5;
static const float PADDLE_SIZE = 2.0;
static const float UPDATE_RATE = 1.0 / 30.0;
static const float PLAYER_X = FIELD_WIDTH * 0.5 + BORDER_SIZE;
static const float AI_SPEED_LIMIT = 0.25;
static const double QUATERNION_TOLERANCE = 0.1f;

using std::placeholders::_1;
using namespace std::chrono_literals;
class PongPaddle : public rclcpp::Node
{
public:
    PongPaddle() : Node("PongPaddle")
    {
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("PongPaddle",
                                                                                 get_node_base_interface(),
                                                                                 get_node_clock_interface(),
                                                                                 get_node_logging_interface(),
                                                                                 get_node_topics_interface(),
                                                                                 get_node_services_interface());
    }

    inline void
    applyChanges()
    {
        server_->applyChanges();
    }


    void makePaddleMarkers()
    {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_link";
        // Add a control for moving the paddle
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        tf2::Quaternion orien(0.0, 0.0, 1.0, 1.0);
        orien.normalize();
        quaternionTFToMsg(orien, control.orientation);

        // Add a visualization marker
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.scale.x = BORDER_SIZE + 0.1;
        marker.scale.y = PADDLE_SIZE + 0.1;
        marker.scale.z = BORDER_SIZE + 0.1;
        marker.pose.position.z = 0;
        marker.pose.position.y = 0;

        control.markers.push_back(marker);

        int_marker.controls.push_back(control);

        // Control for player 1
        int_marker.name = "paddle0";
        int_marker.pose.position.x = -PLAYER_X;
        server_->insert(int_marker);
        // server_->setCallback(int_marker.name, std::bind(&PongGame::processPaddleFeedback, this, 0, _1));

        // Make display markers
        marker.scale.x = BORDER_SIZE;
        marker.scale.y = PADDLE_SIZE;
        marker.scale.z = BORDER_SIZE;
        marker.color.r = 0.5;
        marker.color.a = 1.0;

        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
        control.always_visible = true;


        int_marker.name = "paddle0_display";
        int_marker.pose.position.x = -PLAYER_X;

        marker.color.g = 1.0;
        marker.color.b = 0.5;

        int_marker.controls.clear();
        control.markers.clear();
        control.markers.push_back(marker);
        int_marker.controls.push_back(control);
        server_->insert(int_marker);
    }

    static inline void quaternionTFToMsg(const tf2::Quaternion &bt, geometry_msgs::msg::Quaternion &msg)
    {
        if (fabs(bt.length2() - 1) > QUATERNION_TOLERANCE)
        {
            tf2::Quaternion bt_temp = bt;
            bt_temp.normalize();
            msg.x = bt_temp.x();
            msg.y = bt_temp.y();
            msg.z = bt_temp.z();
            msg.w = bt_temp.w();
        }
        else
        {
            msg.x = bt.x();
            msg.y = bt.y();
            msg.z = bt.z();
            msg.w = bt.w();
        }
    };
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto pong_game = std::make_shared<PongPaddle>();

    pong_game->makePaddleMarkers();
    pong_game->applyChanges();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(pong_game);
    RCLCPP_INFO(pong_game->get_logger(), "Ready");
    executor.spin();
    rclcpp::shutdown();

    return 0;
}