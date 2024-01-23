#include "wp_publisher.hpp"
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
#include <cmath>
#include "geometry_msgs/msg/pose_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

//==============================================================================
/*
 * This function is called by our aruco subscriber whenever aruco is detected.
 * Here we save a copy of the message received so that the above function can process it.
 */
void WaypointsPublisher::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    // check if marker id is empty or not
    if (!msg->marker_ids.empty())
    {
        // RCLCPP_INFO(this->get_logger(), "ARUCO Subscribed");
        aruco_message = msg; // Save message data in member attribute

        size_t arrayLength = sizeof(aruco_message->marker_ids) / sizeof(aruco_message->marker_ids[0]);
        size_t minIndex = -1;
        double mZ = 9999.9; // Initialize to a large value

        // Traverse the array using a for loop
        for (size_t i = 0; i < (arrayLength - 1); ++i)
        {
            // Check if the current z value is smaller than the minimum and if it is greater than 0.5 [min range of camera]
            if ((std::round(aruco_message->poses[i].position.z * 1000) / 1000.0 <= mZ) && (std::round(aruco_message->poses[i].position.z * 1000) / 1000.0 >= 0.5))
            {
                mZ = aruco_message->poses[i].position.z;
                minIndex = i; // Update the index of the minimum z value
            }
        }

        aruco_marker_id = minIndex;
        RCLCPP_INFO_STREAM(this->get_logger(), "Aruco ID identified: " << aruco_marker_id);
        aruco_subscription_.reset();
    }
    else
    {
        // RCLCPP_WARN(this->get_logger(), "ArucoMarkers message is empty.");
        ;
    }
}

//==============================================================================
/*
 * This function is called when we initialise the node.
 * Here, we declare the parameters used in the params file.
 */
void WaypointsPublisher::declare_parameters()
{
    for (int id = 0; id < 2; id++)
    {
        for (int wp_no = 1; wp_no < 6; wp_no++)
        {
            this->declare_parameter<std::string>("aruco_" + std::to_string(id) + ".wp" + std::to_string(wp_no) + ".type", "a");
            this->declare_parameter<std::string>("aruco_" + std::to_string(id) + ".wp" + std::to_string(wp_no) + ".color", "b");
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "declared params");
}

//==============================================================================
/*
 * This function is called when computing the waypoint information.
 * Here we extract the parts names in the corresponding waypoints from the parameter server using aruco id.
 */
void WaypointsPublisher::get_parts_from_params()
{
    for (int i = 1; i < 6; i++)
    {

        std::string type;
        std::string color;
        std::string part_name;
        type = this->get_parameter("aruco_" + std::to_string(aruco_marker_id) + "." + "wp" + std::to_string(i) + ".type").as_string();
        color = this->get_parameter("aruco_" + std::to_string(aruco_marker_id) + "." + "wp" + std::to_string(i) + ".color").as_string();
        part_name = color + "_" + type;
        waypoint_names[i - 1] = part_name;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Extracted WP names");
}

//==============================================================================
/**
 * Get TF pose w.r.t /map frame of a provided part_name (frame name)
 */
geometry_msgs::msg::Pose WaypointsPublisher::get_part_pose(std::string part_name)
{

    std::string target_frame = part_name;
    std::string source_frame = "map";
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return pose_out;
    }
    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;

    tf2::Quaternion tfQuaternion;

    tf2::fromMsg(t_stamped.transform.rotation, tfQuaternion);
    std::array<double, 3> rpy = utils_ptr_->set_euler_from_quaternion(tfQuaternion);
    rpy.at(0) = 3.14 - rpy.at(0);
    pose_out.orientation = utils_ptr_->set_quaternion_from_euler(rpy.at(0) , rpy.at(1) , rpy.at(2) );

    // ----------------------------------------------------------------------------------

    RCLCPP_INFO_STREAM(this->get_logger(), "position of part " << part_name << " " << pose_out.position.x << " " << pose_out.position.y << " " << pose_out.position.z << " ");

    return pose_out;
}


//==============================================================================
/*
 * This function is called in our timer.
 * Here we extract the parts names from paramreter server, then their poses and finally publish a
 * geometry_msgs::PoseArray message with the waypoints poses in order.
 */
void WaypointsPublisher::compute_waypoint_information()
{
    // extract part names
    get_parts_from_params();

    // Message comprising of waypoint poses to be published
    geometry_msgs::msg::PoseArray posearray;
    for (int i = 0; i < 5; i++)
    {
        // push in array 
        posearray.poses.push_back(get_part_pose(waypoint_names[i]));
    }

    for (const auto &pose : posearray.poses)
    {
        // Do something with each pose...
        RCLCPP_INFO_STREAM(this->get_logger(), "Pose: x=" << pose.position.x << ", y=" << pose.position.y << ", z=" << pose.position.z);
    }
    wp_publisher_->publish(std::move(posearray));
}

//==============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsPublisher>("wp_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
}