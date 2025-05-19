#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "gen3_lite_pose_monitor",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("gen3_lite_pose_monitor");

  // Set up TF listener
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);
  
  // Give TF time to receive data
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");
  
  // Get the end effector link name
  std::string end_effector_link = move_group_interface.getEndEffectorLink();
  std::string reference_frame = "base_link";
  
  RCLCPP_INFO(logger, "Monitoring pose of end effector: %s", end_effector_link.c_str());
  RCLCPP_INFO(logger, "Pose monitor started. Press Ctrl+C to stop.");

  // Set up a rate to check the pose
  rclcpp::Rate rate(10); // 10 Hz

  while(rclcpp::ok()) {
    try {
      // First try with TF2
      geometry_msgs::msg::Pose current_pose;
      
      try {
        // Look up transform using TF2
        geometry_msgs::msg::TransformStamped transformStamped = 
            tf_buffer.lookupTransform(reference_frame, end_effector_link, tf2::TimePointZero);
        
        // Convert transform to pose
        current_pose.position.x = transformStamped.transform.translation.x;
        current_pose.position.y = transformStamped.transform.translation.y;
        current_pose.position.z = transformStamped.transform.translation.z;
        current_pose.orientation = transformStamped.transform.rotation;
        
        RCLCPP_INFO(logger, "Current Pose (from TF):");
      }
      catch (const tf2::TransformException& ex) {
        // Fallback to MoveIt if TF fails
        RCLCPP_WARN_THROTTLE(logger, *node->get_clock(), 5000, 
            "Could not get transform from %s to %s: %s. Using MoveIt instead.", 
            reference_frame.c_str(), end_effector_link.c_str(), ex.what());
            
        // Process callbacks to ensure MoveIt has latest state
        rclcpp::spin_some(node);
        
        // Get current pose from MoveIt
        current_pose = move_group_interface.getCurrentPose(end_effector_link).pose;
        
        RCLCPP_INFO(logger, "Current Pose (from MoveIt):");
      }
      
      // Print the pose info
      RCLCPP_INFO(logger, "  Position: x=%.3f, y=%.3f, z=%.3f", 
                 current_pose.position.x, 
                 current_pose.position.y, 
                 current_pose.position.z);
      RCLCPP_INFO(logger, "  Orientation: w=%.3f, x=%.3f, y=%.3f, z=%.3f", 
                 current_pose.orientation.w,
                 current_pose.orientation.x, 
                 current_pose.orientation.y, 
                 current_pose.orientation.z);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(logger, *node->get_clock(), 5000,
                "Error getting current pose: %s", e.what());
    }
    
    // Sleep to maintain the loop rate
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}