#include <memory>
#include <thread>
#include <chrono>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

// Function to execute shell command for gripper control
int executeGripperCommand(float position, float effort = 50.0) {
    std::stringstream cmd;
    cmd << "ros2 action send_goal /gen3_lite_2f_gripper_controller/gripper_cmd "
        << "control_msgs/action/GripperCommand \"{command: {position: " 
        << position << ", max_effort: " << effort << "}}\" -f";
        
    return system(cmd.str().c_str());
}

// Function for moving the robot to a specific position
bool moveTo(const rclcpp::Logger& logger, 
           moveit::planning_interface::MoveGroupInterface& move_group_interface, 
           const std::vector<double>& position,
           const std::string& position_name = "target",
           auto delay = std::chrono::seconds(1)) {
  
  RCLCPP_INFO(logger, "Moving to %s position...", position_name.c_str());
  
  move_group_interface.setJointValueTarget(position);
  auto plan = moveit::planning_interface::MoveGroupInterface::Plan();
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  // Execute the movement if planning was successful
  if (success) {
    RCLCPP_INFO(logger, "Moving to %s position - executing plan", position_name.c_str());
    move_group_interface.execute(plan);
    
    // Add a delay to ensure the movement completes
    rclcpp::sleep_for(delay);
    return true;
  } else {
    RCLCPP_ERROR(logger, "Failed to plan movement to %s position!", position_name.c_str());
    return false;
  }
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "gen3_lite_position_control",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create ROS logger
  auto logger = rclcpp::get_logger("gen3_lite_position_control");

  // Create MoveIt MoveGroup interface for the robot arm
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Set velocity and acceleration scaling factors
  move_group_interface.setMaxVelocityScalingFactor(0.3);
  move_group_interface.setMaxAccelerationScalingFactor(0.3);

  // Define positions
  std::vector<double> home_position = {
    -0.003422694632969403, -0.002761165418194267, 7.190534943213943e-06,
    0.000506799555442079, -0.000536360643542011, -0.0002918824554729582
  };

  std::vector<double> mid_target1_position = {
    -0.13056573349887834, -0.6636405688878977, 2.103391260555483,
    1.5943120392202963, -1.192837309097543, -1.609207631829328
  };

  std::vector<double> target1_position = {
    -0.1510981728711709, -1.3963891027989153, 1.7972587627220027,
    1.5726147331872027, -1.6268212466468936, -1.5995829676497815
  };

  std::vector<double> mixte_position = {
    -0.13026905735344396, -0.6658169041307103, 2.106448569486894,
    1.5933138864437344, -1.1872627803037892, -0.01748418518858763
  };

  std::vector<double> mixte_side1_position = {
    -0.4048393678439597, -0.7555345401977354, 2.0540109276232337,
    1.3008356818351992, -1.2271473460532523, 0.38945401991046263
  };

  std::vector<double> mixte_side2_position = {
    0.18479009013787315, -0.8330458437244923, 2.041348129272125,
    1.9192437911886462, -1.295372206858902, -0.49093137640295126
  };

  rclcpp::sleep_for(std::chrono::seconds(5));

  // Step 1: Move to mid target position
  if (!moveTo(logger, move_group_interface, mid_target1_position, "mid target", std::chrono::seconds(0))) {
    return 1;
  }

  // Step 2: Move to target position
  if (!moveTo(logger, move_group_interface, target1_position, "target", std::chrono::seconds(1))) {
    return 1;
  }

  // Close the gripper
  RCLCPP_INFO(logger, "Closing the gripper...");
  executeGripperCommand(0.8);
  RCLCPP_INFO(logger, "Waiting for gripper to close...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // Step 3: Move back to mid target position
  if (!moveTo(logger, move_group_interface, mid_target1_position, "mid target", std::chrono::seconds(1))) {
    return 1;
  }

  // Step 4: Move to mixte position
  if (!moveTo(logger, move_group_interface, mixte_position, "mixte", std::chrono::seconds(1))) {
    return 1;
  }

  // Increase speed for the shaking movements
  move_group_interface.setMaxVelocityScalingFactor(0.7);
  move_group_interface.setMaxAccelerationScalingFactor(0.7);

  // Step 5: Perform shaking motion
  for (int i = 0; i < 3; i++) {
    if (!moveTo(logger, move_group_interface, mixte_side1_position, "side 1", std::chrono::seconds(0))) {
      return 1;
    }

    if (!moveTo(logger, move_group_interface, mixte_side2_position, "side 2", std::chrono::seconds(0))) {
      return 1;
    }
  }

  // Reset to normal speed
  move_group_interface.setMaxVelocityScalingFactor(0.3);
  move_group_interface.setMaxAccelerationScalingFactor(0.3);

  // Step 6: Return to mid target position
  if (!moveTo(logger, move_group_interface, mid_target1_position, "mid target", std::chrono::seconds(1))) {
    return 1;
  }

  // Step 7: Move to target position again
  if (!moveTo(logger, move_group_interface, target1_position, "target", std::chrono::seconds(1))) {
    return 1;
  }

  // Open the gripper
  RCLCPP_INFO(logger, "Opening the gripper...");
  executeGripperCommand(0.0);
  RCLCPP_INFO(logger, "Waiting for gripper to open...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // Step 8: Move back to mid target position
  if (!moveTo(logger, move_group_interface, mid_target1_position, "mid target", std::chrono::seconds(0))) {
    return 1;
  }

  // Step 9: Return to home position
  if (!moveTo(logger, move_group_interface, home_position, "home", std::chrono::seconds(2))) {
    return 1;
  }

  RCLCPP_INFO(logger, "Motion sequence completed successfully");

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}




/*#include <memory>
#include <cmath> // for M_PI

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "gen3_lite_hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("gen3_lite_hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Print joint limits for joint_2
  auto joint_names = move_group_interface.getJointNames();
  RCLCPP_INFO(logger, "Available joints:");
  for (const auto& name : joint_names) {
    RCLCPP_INFO(logger, " - %s", name.c_str());
  }

  // Log more information about planning capabilities
  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // Set a target Pose for the first movement
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();

  // Define the rest pose
  auto const rest_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 0.183;
    msg.orientation.x = 0.727;
    msg.orientation.y = 0.637;
    msg.orientation.z = 0.176;
    msg.position.x = 0.218;
    msg.position.y = 0.022;
    msg.position.z = 0.028;   // Position de repos
    return msg;
  }();

  // First movement - no constraints
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.clearPathConstraints();
  
  RCLCPP_INFO(logger, "Planning movement to initial target pose...");
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if(success) {
    RCLCPP_INFO(logger, "Executing movement to initial target pose...");
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Initial target pose reached");
  } else {
    RCLCPP_ERROR(logger, "Planning to initial target failed!");
    return 1;
  }

  // Try planning to the rest position without constraints first
  RCLCPP_INFO(logger, "Planning movement to rest pose (no constraints)...");
  move_group_interface.clearPathConstraints();
  move_group_interface.setPoseTarget(rest_pose);
  
  auto const [success_rest_no_constraints, rest_plan_no_constraints] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  
  // If this works, use the resulting joint values to inform our constraint
  if (success_rest_no_constraints) {
    RCLCPP_INFO(logger, "Successfully planned to rest pose without constraints");
    
    // Get the current joint values after planning
    std::vector<double> joint_values = move_group_interface.getCurrentJointValues();
    
    if (joint_names.size() == joint_values.size()) {
      for (size_t i = 0; i < joint_names.size(); i++) {
        RCLCPP_INFO(logger, "Joint %s: %.3f rad (%.1f degrees)", 
                   joint_names[i].c_str(), 
                   joint_values[i], 
                   joint_values[i] * 180.0 / M_PI);
      }
    }
  }

  // Now try with a more relaxed constraint
  RCLCPP_INFO(logger, "Trying to plan with joint_2 constraint at -147 degrees...");
  
  // Create the constraint - use a more relaxed tolerance
  moveit_msgs::msg::Constraints joint_constraints;
  moveit_msgs::msg::JointConstraint joint2_constraint;
  
  // Convert -147 degrees to radians
  double target_angle_rad = -147.0 * M_PI / 180.0;
  
  // Find the index of joint_2
  int joint2_index = -1;
  for (size_t i = 0; i < joint_names.size(); i++) {
    if (joint_names[i] == "joint_2") {
      joint2_index = i;
      break;
    }
  }
  
  // Set up the constraint with more tolerance
  joint2_constraint.joint_name = "joint_2";
  joint2_constraint.position = target_angle_rad;
  joint2_constraint.tolerance_above = 0.3;   // More tolerance (about 17 degrees)
  joint2_constraint.tolerance_below = 0.3;   // More tolerance (about 17 degrees)
  joint2_constraint.weight = 0.8;            // Slightly less priority
  
  // Add constraint to constraints message
  joint_constraints.joint_constraints.push_back(joint2_constraint);
  
  // Set path constraints
  move_group_interface.setPathConstraints(joint_constraints);
  
  // Set target again
  move_group_interface.setPoseTarget(rest_pose);
  
  // Try planning with the constraint
  RCLCPP_INFO(logger, "Planning with relaxed joint_2 constraint...");
  auto const [success_rest_with_constraint, rest_plan_with_constraint] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute if successful
  if (success_rest_with_constraint) {
    RCLCPP_INFO(logger, "Successfully planned to rest pose with joint_2 constraint");
    move_group_interface.execute(rest_plan_with_constraint);
    RCLCPP_INFO(logger, "Rest pose reached with joint_2 constraint");
  } else {
    RCLCPP_ERROR(logger, "Failed to plan with joint_2 constraint, executing fallback plan");
    
    // Execute the fallback plan we created earlier
    if (success_rest_no_constraints) {
      move_group_interface.clearPathConstraints();
      move_group_interface.execute(rest_plan_no_constraints);
      RCLCPP_INFO(logger, "Rest pose reached using fallback plan");
    } else {
      RCLCPP_ERROR(logger, "No fallback plan available!");
      return 1;
    }
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

*/







/*
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  // Initialiser ROS et créer le Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "gen3_lite_hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Créer un logger pour ROS
  auto const logger = rclcpp::get_logger("gen3_lite_hello_moveit");

  // Créer l'interface MoveIt MoveGroup pour le bras robot
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Définir un facteur de mise à l'échelle de la vitesse (1.0 = vitesse maximale)
  move_group_interface.setMaxVelocityScalingFactor(1.0);  // Vitesse maximale (100%)

  // Définir un facteur de mise à l'échelle de l'accélération (1.0 = accélération maximale)
  move_group_interface.setMaxAccelerationScalingFactor(1.0);  // Accélération maximale (100%)


  // Définir la position de la bouteille juste devant le bras
  auto const bottle_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = -1;
    msg.position.x = 0.28;  // Ajuster la position x selon la position de la bouteille
    msg.position.y = 0.0;   // Ajuster la position y selon la position de la bouteille
    msg.position.z = 0.5;   // Ajuster la position z selon la position de la bouteille
    return msg;
  }();

  // Définir la position derrière le bras à 135 degrés (environ)
  auto const behind_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = -0.40;  // Déplacer le bras derrière lui
    msg.position.y = -0.3;   // Ajuster y pour simuler la direction de 135°
    msg.position.z = 0.5;   // Hauteur similaire à la position de la bouteille
    return msg;
  }();

  // Définir la position de repos du bras
  auto const rest_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = 0.0;
    msg.position.z = 0.9;   // Position de repos
    return msg;
  }();

  // Étape 1 : Déplacer le bras pour saisir la bouteille
  move_group_interface.setPoseTarget(bottle_pose);
  auto const [success_pick, pick_plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Si la planification réussit, exécuter le mouvement
  if (success_pick) {
    move_group_interface.execute(pick_plan);
  } else {
    RCLCPP_ERROR(logger, "Échec de la planification pour saisir la bouteille!");
    return 1;
  }

  // Étape 2 : Déplacer le bras derrière lui à 135 degrés
  move_group_interface.setPoseTarget(behind_pose);
  auto const [success_place, place_plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Si la planification réussit, exécuter le mouvement
  if (success_place) {
    move_group_interface.execute(place_plan);
  } else {
    RCLCPP_ERROR(logger, "Échec de la planification pour déposer la bouteille!");
    return 1;
  }

  // Étape 3 : Retourner à la position de repos
  move_group_interface.setPoseTarget(rest_pose);
  auto const [success_rest, rest_plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Si la planification réussit, exécuter le mouvement
  if (success_rest) {
    move_group_interface.execute(rest_plan);
  } else {
    RCLCPP_ERROR(logger, "Échec de la planification pour retourner au repos!");
    return 1;
  }

  // Arrêter ROS
  rclcpp::shutdown();
  return 0;
}
*/








/*#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <std_msgs/msg/float32.hpp>

int main(int argc, char * argv[])
{
  // Initialiser ROS et créer le Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "gen3_lite_hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Créer un logger pour ROS
  auto const logger = rclcpp::get_logger("gen3_lite_hello_moveit");

  // Créer l'interface MoveIt MoveGroup pour le bras robot
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Créer un publisher pour envoyer la commande du gripper
  auto gripper_pub = node->create_publisher<std_msgs::msg::Float32>("gripper_controller/command", 10);

  // Définir un facteur de mise à l'échelle de la vitesse (1.0 = vitesse maximale)
  move_group_interface.setMaxVelocityScalingFactor(1.0);  // Vitesse maximale (100%)
  move_group_interface.setMaxAccelerationScalingFactor(1.0);  // Accélération maximale (100%)

  // Définir la position de la bouteille juste devant le bras
  auto const bottle_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = -1;
    msg.position.x = 0.28;
    msg.position.y = 0.0;
    msg.position.z = 0.5;
    return msg;
  }();

  // Définir la position derrière le bras à 135 degrés (environ)
  auto const behind_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = -0.40;
    msg.position.y = -0.3;
    msg.position.z = 0.5;
    return msg;
  }();

  // Définir la position de repos du bras
  auto const rest_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = 0.0;
    msg.position.z = 0.9;
    return msg;
  }();

  // Fonction pour fermer le gripper (saisir l'objet)
  auto close_gripper = [&gripper_pub]() {
    std_msgs::msg::Float32 msg;
    msg.data = 0.0;  // 0.0 pour fermer le gripper
    gripper_pub->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("Gripper"), "Gripper fermé");
  };

  // Fonction pour ouvrir le gripper (relâcher l'objet)
  auto open_gripper = [&gripper_pub]() {
    std_msgs::msg::Float32 msg;
    msg.data = 0.8;  // 1.0 pour ouvrir le gripper
    gripper_pub->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("Gripper"), "Gripper ouvert");
  };

  // Étape 1 : Déplacer le bras pour saisir la bouteille
  move_group_interface.setPoseTarget(bottle_pose);
  auto const [success_pick, pick_plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Si la planification réussit, exécuter le mouvement
  if (success_pick) {
    move_group_interface.execute(pick_plan);
    open_gripper();  // Ouvrir le gripper avant de saisir
    rclcpp::sleep_for(std::chrono::seconds(1)); // Attendre que le gripper soit ouvert
    close_gripper(); // Fermer le gripper pour saisir la bouteille
    rclcpp::sleep_for(std::chrono::seconds(1)); // Attendre que la bouteille soit saisie
  } else {
    RCLCPP_ERROR(logger, "Échec de la planification pour saisir la bouteille!");
    return 1;
  }

  // Étape 2 : Déplacer le bras derrière lui à 135 degrés
  move_group_interface.setPoseTarget(behind_pose);
  auto const [success_place, place_plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Si la planification réussit, exécuter le mouvement
  if (success_place) {
    move_group_interface.execute(place_plan);
    open_gripper();  // Relâcher la bouteille en ouvrant le gripper
    rclcpp::sleep_for(std::chrono::seconds(1)); // Attendre que la bouteille soit relâchée
  } else {
    RCLCPP_ERROR(logger, "Échec de la planification pour déposer la bouteille!");
    return 1;
  }

  // Étape 3 : Retourner à la position de repos
  move_group_interface.setPoseTarget(rest_pose);
  auto const [success_rest, rest_plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Si la planification réussit, exécuter le mouvement
  if (success_rest) {
    move_group_interface.execute(rest_plan);
  } else {
    RCLCPP_ERROR(logger, "Échec de la planification pour retourner au repos!");
    return 1;
  }

  // Arrêter ROS
  rclcpp::shutdown();
  return 0;
}*/