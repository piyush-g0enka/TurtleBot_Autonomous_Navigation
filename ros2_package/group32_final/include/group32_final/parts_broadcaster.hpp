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
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief PartsBroadcaster broadcasts detected parts pose in TF
 * @note It also send a message to our controller notifying of the parts detected
 */
class PartsBroadcaster : public rclcpp::Node
{
public:
    // ==================== constructors ====================
    /**
     * @brief Construct a new Parts Broadcaster object
     *
     * @param node_name
     */
    PartsBroadcaster(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Parts TF Broadcaster started");

        // Created Maps to extract parts color and type from int data
        parts_color_map[0] = "red";
        parts_color_map[1] = "green";
        parts_color_map[2] = "blue";
        parts_color_map[3] = "orange";
        parts_color_map[4] = "purple";

        parts_type_map[10] = "battery";
        parts_type_map[11] = "pump";
        parts_type_map[12] = "sensor";
        parts_type_map[13] = "regulator";

        // Create a compatible Quality of Service with the sensor
        rclcpp::QoS qos = rclcpp::SensorDataQoS();

        // Subscribe to Logical cameras
        camera1_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera1/image", qos, std::bind(&PartsBroadcaster::camera1_callback, this, _1));

        camera2_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera2/image", qos, std::bind(&PartsBroadcaster::camera2_callback, this, _1));

        camera3_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera3/image", qos, std::bind(&PartsBroadcaster::camera3_callback, this, _1));

        camera4_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera4/image", qos, std::bind(&PartsBroadcaster::camera4_callback, this, _1));

        camera5_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera5/image", qos, std::bind(&PartsBroadcaster::camera5_callback, this, _1));

        // Initialize the static transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);

        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();
    }

private:
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /*!< Static broadcaster object */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    /*!< Utils object to access utility functions*/
    std::shared_ptr<Utils> utils_ptr_;

    /*!< Subscriber for Logical Camera 1*/
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera1_subscription_;

    /*!< Subscriber for Logical Camera 2*/
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera2_subscription_;

    /*!< Subscriber for Logical Camera 3*/
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera3_subscription_;

    /*!< Subscriber for Logical Camera 4*/
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera4_subscription_;

    /*!< Subscriber for Logical Camera 5*/
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera5_subscription_;

    /*!< Maps declaration*/
    std::map<int, std::string> parts_color_map;
    std::map<int, std::string> parts_type_map;

    /*!< Publisher to send detected part names to controller*/
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;

    /**
     * @brief Broadcast the transform of the detected part
     *
     * @private
     */
    void static_broadcast_cb_(const std::string &camera_frame_name, const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for logical camera 1 subscriber
     *
     * @param msg
     */
    void camera1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for logical camera 2 subscriber
     *
     * @param msg
     */
    void camera2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for logical camera 3 subscriber
     *
     * @param msg
     */
    void camera3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for logical camera 4 subscriber
     *
     * @param msg
     */
    void camera4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for logical camera 5 subscriber
     *
     * @param msg
     */
    void camera5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
};
