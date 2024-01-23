#include "parts_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "utils.hpp"
#include <tf2/exceptions.h>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <iostream>
#include "mage_msgs/msg/part.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

//==============================================================================
/**
 * Broadcasts parts-pose and sensor-pose to TF
 */
void PartsBroadcaster::static_broadcast_cb_(const std::string &camera_frame_name, const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr parts_message)
{

    geometry_msgs::msg::TransformStamped static_transform_stamped;

    // Part name is extracted from Maps
    std::string part_color = parts_color_map[parts_message->part_poses[0].part.color];
    std::string part_type = parts_type_map[parts_message->part_poses[0].part.type];
    std::string detected_part = part_color + "_" + part_type;

    static_transform_stamped.header.stamp = this->get_clock()->now();

    static_transform_stamped.header.frame_id = camera_frame_name;
    static_transform_stamped.child_frame_id = detected_part;

    static_transform_stamped.transform.translation.x = parts_message->part_poses[0].pose.position.x;
    static_transform_stamped.transform.translation.y = parts_message->part_poses[0].pose.position.y;
    static_transform_stamped.transform.translation.z = parts_message->part_poses[0].pose.position.z;

    static_transform_stamped.transform.rotation.x = parts_message->part_poses[0].pose.orientation.x;
    static_transform_stamped.transform.rotation.y = parts_message->part_poses[0].pose.orientation.y;
    static_transform_stamped.transform.rotation.z = parts_message->part_poses[0].pose.orientation.z;
    static_transform_stamped.transform.rotation.w = parts_message->part_poses[0].pose.orientation.w;

    // Send the transform
    tf_static_broadcaster_->sendTransform(static_transform_stamped);
    RCLCPP_INFO_STREAM(this->get_logger(), "Parts Broadcasted " << part_color + "_" + part_type);

    // Camera pose is published in TF
    geometry_msgs::msg::TransformStamped static_transform_stamped_1;

    static_transform_stamped_1.header.stamp = this->get_clock()->now();

    static_transform_stamped_1.header.frame_id = "map";
    static_transform_stamped_1.child_frame_id = camera_frame_name;

    static_transform_stamped_1.transform.translation.x = parts_message->sensor_pose.position.x;
    static_transform_stamped_1.transform.translation.y = parts_message->sensor_pose.position.y;
    static_transform_stamped_1.transform.translation.z = parts_message->sensor_pose.position.z;

    static_transform_stamped_1.transform.rotation.x = parts_message->sensor_pose.orientation.x;
    static_transform_stamped_1.transform.rotation.y = parts_message->sensor_pose.orientation.y;
    static_transform_stamped_1.transform.rotation.z = parts_message->sensor_pose.orientation.z;
    static_transform_stamped_1.transform.rotation.w = parts_message->sensor_pose.orientation.w;

    // Send the transform
    tf_static_broadcaster_->sendTransform(static_transform_stamped_1);


    // Odom transform
    geometry_msgs::msg::TransformStamped static_transform_stamped_2;
    static_transform_stamped_2.header.stamp = this->get_clock()->now();
    static_transform_stamped_2.header.frame_id = "world";
    static_transform_stamped_2.child_frame_id = "odom";
    static_transform_stamped_2.transform.rotation.x = 0;
    static_transform_stamped_2.transform.rotation.y = 0;
    static_transform_stamped_2.transform.rotation.z = 0;
    static_transform_stamped_2.transform.rotation.w = 1;

    // Send the transform
    tf_static_broadcaster_->sendTransform(static_transform_stamped_2);
}

//==============================================================================
/**
 * Callback to camera1 subscriber
 * Saves a local copy of the recent detected parts
 */
void PartsBroadcaster::camera1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    if (!msg->part_poses.empty())
    {
        static_broadcast_cb_("camera1_frame", msg);
        camera1_subscription_.reset();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Parts message is empty.");  
    }
}

//==============================================================================
/**
 * Callback to camera2 subscriber
 * Saves a local copy of the recent detected parts
 */
void PartsBroadcaster::camera2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    if (!msg->part_poses.empty())
    {
       
        static_broadcast_cb_("camera2_frame", msg);
        camera2_subscription_.reset();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Parts message is empty.");
      
    }
}

//==============================================================================
/**
 * Callback to camera3 subscriber
 * Saves a local copy of the recent detected parts
 */
void PartsBroadcaster::camera3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    if (!msg->part_poses.empty())
    {
        
        static_broadcast_cb_("camera3_frame", msg);
        camera3_subscription_.reset();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Parts message is empty.");
        
    }
}

//==============================================================================
/**
 * Callback to camera4 subscriber
 * Saves a local copy of the recent detected parts
 */
void PartsBroadcaster::camera4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    if (!msg->part_poses.empty())
    {
        static_broadcast_cb_("camera4_frame", msg);
        camera4_subscription_.reset();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Parts message is empty.");
        
    }
}

//==============================================================================
/**
 * Callback to camera5 subscriber
 * Saves a local copy of the recent detected parts
 */
void PartsBroadcaster::camera5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    if (!msg->part_poses.empty())
    {
        static_broadcast_cb_("camera5_frame", msg);
        camera5_subscription_.reset();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Parts message is empty.");
        
    }
}


//==============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PartsBroadcaster>("Parts_Broadcaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
}