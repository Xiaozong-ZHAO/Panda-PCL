#include "cw2_class.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>

/**
 * @brief Controls the gripper to open or close to a target width.
 * 
 * @param width Target opening width in meters. Clamped between 0.01 and 0.10.
 * @return true if the gripper motion was successful.
 * @return false if planning or execution failed.
 */
bool cw2::move_gripper(float width)
{
  const float gripper_max = 0.10;
  const float gripper_min = 0.01;

  // Clamp the width to the safe range
  width = std::max(gripper_min, std::min(gripper_max, width));
  std::vector<double> joint_targets(2, width / 2.0);

  hand_group_.setJointValueTarget(joint_targets);
  hand_group_.setMaxVelocityScalingFactor(1.0);   // full speed
  hand_group_.setMaxAccelerationScalingFactor(1.0);
  hand_group_.setPlanningTime(3.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (hand_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    hand_group_.move();  // execute the plan
  } else {
    ROS_ERROR("Gripper planning failed!");
  }
  return success;
}

/**
 * @brief Moves the robot arm to a target position with optional Z offset and orientation reset.
 * 
 * @param target Target point in the world frame.
 * @param z_offset Z-axis offset (in meters) to add to the target.
 * @param reset_orientation If true, resets end-effector orientation to a fixed downward pose.
 * @return true if motion planning and execution succeeded.
 * @return false if planning failed.
 */
bool cw2::move_to_pose(const geometry_msgs::PointStamped& target, double z_offset, bool reset_orientation)
{
  geometry_msgs::Pose target_pose;
  // set desired position
  target_pose.position.x = target.point.x;
  target_pose.position.y = target.point.y;
  target_pose.position.z = target.point.z + z_offset;

  if (reset_orientation) {
    // fixed downward orientation
    target_pose.orientation.x = 0.9238795;
    target_pose.orientation.y = -0.3826834;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;
  } else {
    // keep current orientation
    geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
    target_pose.orientation = current_pose.pose.orientation;
  }

  arm_group_.setPoseTarget(target_pose);
  arm_group_.setMaxVelocityScalingFactor(0.7);
  arm_group_.setMaxAccelerationScalingFactor(0.7);
  arm_group_.setPlanningTime(5.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    arm_group_.move();  // execute trajectory
  } else {
    ROS_ERROR("In move_to_pose(): Arm planning failed!");
  }

  return success;
}

/**
 * @brief Rotates the end-effector around Z to a specific yaw angle, preserving roll/pitch.
 * 
 * @param target_yaw Target yaw angle in radians.
 * @return true if rotation planning and execution succeeded.
 * @return false otherwise.
 */
bool cw2::rotate_end_effector(double target_yaw)
{
  // get current pose and orientation
  geometry_msgs::PoseStamped current_pose_stamped = arm_group_.getCurrentPose();
  geometry_msgs::Pose current_pose = current_pose_stamped.pose;

  // decompose quaternion to roll, pitch, yaw
  tf2::Quaternion current_q;
  tf2::fromMsg(current_pose.orientation, current_q);
  double roll, pitch, yaw_now;
  tf2::Matrix3x3(current_q).getRPY(roll, pitch, yaw_now);

  // build new quaternion with desired yaw
  tf2::Quaternion new_q;
  new_q.setRPY(roll, pitch, target_yaw);
  new_q.normalize();

  geometry_msgs::Pose new_pose = current_pose;
  new_pose.orientation = tf2::toMsg(new_q);

  arm_group_.setPoseTarget(new_pose);
  arm_group_.setMaxVelocityScalingFactor(0.7);
  arm_group_.setMaxAccelerationScalingFactor(0.7);
  arm_group_.setPlanningTime(5.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    arm_group_.move();
  } else {
    ROS_ERROR("rotate_end_effector_to planning failed!");
  }

  return success;
}

/**
 * @brief Rotates a specified joint by a given delta angle.
 * 
 * @param joint_name Name of the joint to rotate.
 * @param delta_angle_rad Angle offset in radians.
 * @return true if planning and execution succeeded.
 * @return false on error or invalid joint.
 */
bool cw2::rotate_joint(const std::string& joint_name, double delta_angle_rad)
{
  // map names to indices
  std::map<std::string, int> joint_name_to_index = {
    {"base", 0}, {"shoulder", 1}, {"upper_arm", 2},
    {"elbow", 3}, {"forearm", 4}, {"wrist", 5}, {"eef", 6}
  };

  auto it = joint_name_to_index.find(joint_name);
  if (it == joint_name_to_index.end()) {
    ROS_ERROR("Invalid joint name: %s", joint_name.c_str());
    return false;
  }

  int joint_index = it->second;
  std::vector<double> joint_values = arm_group_.getCurrentJointValues();
  if (joint_values.empty() || joint_index < 0 || joint_index >= joint_values.size()) {
    ROS_ERROR("Joint index %d is out of range.", joint_index);
    return false;
  }

  // apply delta
  joint_values[joint_index] += delta_angle_rad;

  // plan and execute
  arm_group_.setJointValueTarget(joint_values);
  arm_group_.setPlanningTime(5.0);
  arm_group_.setMaxVelocityScalingFactor(0.7);
  arm_group_.setMaxAccelerationScalingFactor(0.7);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    arm_group_.execute(plan);
    ROS_INFO("Rotated joint [%s] by %.2f degrees.", joint_name.c_str(), delta_angle_rad * 180.0 / M_PI);
    return true;
  } else {
    ROS_ERROR("Failed to plan joint rotation.");
    return false;
  }
}

/**
 * @brief Performs a pick-and-place operation using Cartesian path planning.
 * 
 * @param grasp_point The point to grasp the object.
 * @param place_point The point to place the object.
 * @param rotate_angle Target yaw for the end-effector at grasp time.
 * @return true if the full operation succeeded.
 * @return false if any step failed.
 */
bool cw2::cartesian_grasp_and_place(
  const geometry_msgs::PointStamped& grasp_point,
  const geometry_msgs::PointStamped& place_point,
  double rotate_angle)
{
// 1. open gripper
move_gripper(0.10);

// 2. move above grasp point
if (!move_to_pose(grasp_point, 0.15, true)) {
  ROS_ERROR("Failed to move above the object.");
  return false;
}

// 3. rotate end-effector
if (!rotate_end_effector(rotate_angle)) {
  ROS_ERROR("Failed to rotate end-effector.");
  return false;
}

// 4. attempt Cartesian straight-down descent to grasp height
{
  // build single waypoint: directly above to grasp height
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped current_pose_stamped = arm_group_.getCurrentPose();
  geometry_msgs::Pose down_pose = current_pose_stamped.pose;
  down_pose.position.x = grasp_point.point.x;
  down_pose.position.y = grasp_point.point.y;
  down_pose.position.z = grasp_point.point.z + 0.07;
  waypoints.push_back(down_pose);

  moveit_msgs::RobotTrajectory cartesian_trajectory;
  double fraction = arm_group_.computeCartesianPath(waypoints, 0.01, cartesian_trajectory, true);
  if (fraction >= 0.95) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = cartesian_trajectory;
    arm_group_.execute(cart_plan);
  } else {
    ROS_WARN("Cartesian descent failed (%.2f%%). Falling back to move_to_pose.", fraction * 100.0);
    // fallback to original point-to-point descent
    if (!move_to_pose(grasp_point, 0.07, false)) {
      ROS_ERROR("Fallback move_to_pose to grasp height failed.");
      return false;
    }
  }
}

// 5. close gripper
move_gripper(0.01);

// 6. record current pose for Cartesian planning
geometry_msgs::PoseStamped base_pose_stamped = arm_group_.getCurrentPose();
geometry_msgs::Pose base_pose = base_pose_stamped.pose;

// build waypoints for transfer and placement
std::vector<geometry_msgs::Pose> waypoints;
geometry_msgs::Pose lift_pose = base_pose;
lift_pose.position.z = grasp_point.point.z + 0.4;
waypoints.push_back(lift_pose);

geometry_msgs::Pose move_y_pose = lift_pose;
move_y_pose.position.y = place_point.point.y;
waypoints.push_back(move_y_pose);

geometry_msgs::Pose move_x_pose = move_y_pose;
move_x_pose.position.x = place_point.point.x;
waypoints.push_back(move_x_pose);

geometry_msgs::Pose lower_pose = move_x_pose;
lower_pose.position.z = place_point.point.z + 0.4;
waypoints.push_back(lower_pose);

// 7. compute Cartesian path for transfer and placement
moveit_msgs::RobotTrajectory trajectory;
double fraction = arm_group_.computeCartesianPath(waypoints, 0.01, trajectory, true);
if (fraction >= 0.95) {
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  arm_group_.execute(plan);
} else {
  ROS_WARN("Cartesian path planning failed (%.2f%%). Falling back to point-to-point planning.", fraction * 100.0);

  geometry_msgs::PointStamped mid1 = grasp_point;
  mid1.point.z = grasp_point.point.z + 0.4;
  geometry_msgs::PointStamped mid2 = place_point;
  mid2.point.z = place_point.point.z + 0.4;

  // fallback moves
  if (!move_to_pose(mid1, 0.0, false)) {
    ROS_ERROR("Fallback move_to_pose: mid1 failed.");
    return false;
  }
  if (!move_to_pose(mid2, 0.0, false)) {
    ROS_ERROR("Fallback move_to_pose: mid2 failed.");
    return false;
  }
}

// 8. release object
move_gripper(0.10);
return true;
}

/**
 * @brief Saves the current joint values and end-effector pose.
 * 
 * @return true after saving the initial state.
 */
bool cw2::save_initial_joint_and_pose()
{
  initial_joint_values_ = arm_group_.getCurrentJointValues();
  initial_ee_pose_      = arm_group_.getCurrentPose().pose;
  ROS_INFO("Initial joint angles and pose saved.");
  return true;
}

/**
 * @brief Returns the arm to the previously saved joint configuration.
 * 
 * @return true if planning and execution succeeded.
 * @return false if no initial state or planning failed.
 */
bool cw2::go_to_initial_state()
{
  if (initial_joint_values_.empty()) {
    ROS_WARN("Initial joint values are not recorded yet!");
    return false;
  }

  arm_group_.setJointValueTarget(initial_joint_values_);
  arm_group_.setPlanningTime(5.0);
  arm_group_.setMaxVelocityScalingFactor(0.7);
  arm_group_.setMaxAccelerationScalingFactor(0.7);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    arm_group_.execute(plan);
    ROS_INFO("Returned to initial joint state.");
    return true;
  } else {
    ROS_ERROR("Failed to plan return to initial joint state.");
    return false;
  }
}

/**
 * @brief Executes a Cartesian scan of a sub-area defined by 4 corner points.
 * 
 * @param corners A vector of exactly 4 corner points in order.
 * @return true if the Cartesian scan succeeded.
 * @return false if planning or execution failed.
 */
bool cw2::scan_sub_area(const std::vector<geometry_msgs::PointStamped>& corners)
{
  if (corners.size() != 4) {
    ROS_ERROR("scan_sub_area: exactly 4 corner points are required.");
    return false;
  }

  // move to first corner
  if (!move_to_pose(corners[0], 0.0, true)) {
    ROS_ERROR("Failed to move to first corner.");
    return false;
  }
  ros::Duration(1.0).sleep();  // optional wait

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose base_pose = arm_group_.getCurrentPose().pose;

  // build remaining corner waypoints
  for (int i = 1; i < 4; ++i) {
    geometry_msgs::Pose pose;
    pose.position.x = corners[i].point.x;
    pose.position.y = corners[i].point.y;
    pose.position.z = corners[i].point.z;
    pose.orientation = base_pose.orientation;  // keep same orientation
    waypoints.push_back(pose);
  }

  // plan and execute Cartesian path
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = arm_group_.computeCartesianPath(waypoints, 0.01, trajectory, true);
  if (fraction < 0.95) {
    ROS_WARN("Cartesian path planning only %.2f%% success.", fraction * 100.0);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  is_scanning_ = true;
  arm_group_.execute(plan);  // perform scan
  is_scanning_ = false;

  return true;
}
