#include "robot_move.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

//==============================================================================
/**
 * Send goals to the action server
 */
void NavigateThroughPosesClient::send_goal(const std::vector<geometry_msgs::msg::PoseStamped> &poses)
{
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = poses;

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.feedback_callback =
        std::bind(&NavigateThroughPosesClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavigateThroughPosesClient::result_callback, this, std::placeholders::_1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

//==============================================================================
/**
 * Set initial pose of the robot
 */
void NavigateThroughPosesClient::set_initial_pose()
{

    geometry_msgs::msg::TransformStamped t_stamped;
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
    try
    {
        t_stamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero, 100ms);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got transform between"
                                                   << " odom and base_link");
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "Could not get transform between"
                                                   << " odom and base_link"
                                                   << ": " << ex.what());
    }

    message.header.frame_id = "odom";
    message.pose.pose.position.x = t_stamped.transform.translation.x;
    message.pose.pose.position.y = t_stamped.transform.translation.y;
    message.pose.pose.orientation.x = t_stamped.transform.rotation.x;
    message.pose.pose.orientation.y = t_stamped.transform.rotation.y;
    message.pose.pose.orientation.z = t_stamped.transform.rotation.z;
    message.pose.pose.orientation.w = t_stamped.transform.rotation.w;
    initial_pose_pub_->publish(message);
}

//==============================================================================
/**
 * Feedback callback from action server
 */
void NavigateThroughPosesClient::feedback_callback(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
    RCLCPP_INFO_STREAM(this->get_logger(), feedback<< " Robot is driving towards the goal..." );


}

//==============================================================================
/**
 * Result callback from action server
 */
void NavigateThroughPosesClient::result_callback(const GoalHandleNavigateThroughPoses::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal reached");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal aborted");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal canceled");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown code");
        break;
    }
}

//==============================================================================
/**
 * Callback function for waypoints subscriber
 */
void NavigateThroughPosesClient::waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // check if initial pose of robot is set or not
    if (isInitialPoseSet == true)
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses_list;

        // Iterate through the poses in the PoseArray
        for (const auto &pose : msg->poses)
        {
            geometry_msgs::msg::PoseStamped new_pose;
            new_pose.header.frame_id = "map";
            new_pose.pose.position.x = pose.position.x;
            new_pose.pose.position.y = pose.position.y;
            new_pose.pose.orientation.w = pose.orientation.w;
            new_pose.pose.orientation.x = pose.orientation.x;
            new_pose.pose.orientation.y = pose.orientation.y;
            new_pose.pose.orientation.z = pose.orientation.z;
            poses_list.push_back(new_pose);
        }

        // send the waypoint poses to Action Server
        this->send_goal(poses_list);
        isInitialPoseSet = false;

        // Reset subscriber since we do not require it now
        waypoints_pose_subscriber_.reset();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateThroughPosesClient>();

    // pause for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));
    node->set_initial_pose();
    node->isInitialPoseSet = true;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
