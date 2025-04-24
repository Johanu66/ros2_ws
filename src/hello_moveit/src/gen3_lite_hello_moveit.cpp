/*#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

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

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}*/









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








#include <memory>
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
}
