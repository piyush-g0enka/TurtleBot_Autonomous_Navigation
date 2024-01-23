#include <memory>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <action_msgs/msg/goal_status.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "utils.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_array.hpp>

/**
 * @brief Action client to NavigateThroughPoses action server
 * @note It sends goals to the action server and handles the result and feedback
 *
 */
class NavigateThroughPosesClient : public rclcpp::Node
{
public:
    explicit NavigateThroughPosesClient(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
        : Node("navigate_through_poses_client", node_options)
    {

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        isInitialPoseSet = false;

        // Create a client to the action server
        client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this,"navigate_through_poses");

        // Create a publisher to publish initial pose
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10);

        // Create a subscriber to get waypoints
        waypoints_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "waypoints_pose", 10, std::bind(&NavigateThroughPosesClient::waypointsCallback, this, std::placeholders::_1));
    }

    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    /**
     * @brief Send goals to the action server
     *
     * @param poses
     */
    void send_goal(const std::vector<geometry_msgs::msg::PoseStamped> &poses);

    /**
     * @brief Set initial pose of the robot
     *
     */
    void set_initial_pose();
    bool isInitialPoseSet;

private:
    /*!< Pointer to the action client */
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;

    /*!< Publisher to publish initial pose */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /*!< TF listener object*/
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

    /*!< Subscriber to get waypoints */
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pose_subscriber_;

    /*!< Feedback callback to from action server*/
    void feedback_callback(
        GoalHandleNavigateThroughPoses::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);

    /*!< Result callback from action server*/
    void result_callback(const GoalHandleNavigateThroughPoses::WrappedResult &result);

    /*!< Callback function for waypoints subscriber*/
    void waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
};
