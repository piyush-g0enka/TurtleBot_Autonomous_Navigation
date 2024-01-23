#pragma once

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include "utils.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief Class WaypointsPublisher: Extracts waypoints from Aruco ID and Parts poses. Then publishes them to the network.
 *
 */
class WaypointsPublisher : public rclcpp::Node
{
public:
    // ==================== constructors ====================
    /**
     * @brief Construct a new WaypointsPublisher object
     *
     * @param node_name
     */
    WaypointsPublisher(std::string node_name) : Node(node_name)
    {

        RCLCPP_INFO(this->get_logger(), "Waypoints Publisher started...");

        // Initialize aruco message
        aruco_message = nullptr;

        // declare waypoint parameters
        declare_parameters();

        // Subscriber for Aruco Marker messages
        aruco_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers", 10, std::bind(&WaypointsPublisher::aruco_callback, this, _1));

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create publisher which publishes waypoint information to ROS network
        wp_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("waypoints_pose", 10);

        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();

        // // timer to compute waypoints
        compute_wp_timer_ = this->create_wall_timer(
            2s, std::bind(&WaypointsPublisher::compute_waypoint_information, this));
    }

private:
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /*!< Utils object to access utility functions*/
    std::shared_ptr<Utils> utils_ptr_;

    /*!< Wall timer object for the static broadcaster*/
    rclcpp::TimerBase::SharedPtr compute_wp_timer_;

    /*!< Aruco subscriber*/
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription_;

    /*!< Store latest Aruco message*/
    ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_message;

    /*!< TF listener object*/
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

    /*!< Store the aruco marker id */
    int aruco_marker_id;

    /*!< Waypoint publisher object*/
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr wp_publisher_;

    /*!< Array to store waypoint names in order*/
    std::string waypoint_names[5];

    /**
     * @brief Callback functin for Aruco Subscriber
     *
     */
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief Function which declares parameters in the waypoint_params file
     *
     */
    void declare_parameters();

    /**
     * @brief Function which retrieves a part's pose from TF tree in the /map frame using its name
     *
     */
    geometry_msgs::msg::Pose get_part_pose(std::string part_name);

    /**
     * @brief Function which computes waypoints from aruco and parts data. Then publishes waypoint information.
     *
     */
    void compute_waypoint_information();

    /**
     * @brief Function which extratcs parts names from param file with aruco id
     *
     */
    void get_parts_from_params();
};
