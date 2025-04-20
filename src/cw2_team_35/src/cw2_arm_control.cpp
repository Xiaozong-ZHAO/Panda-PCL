#include "cw2_class.h"

bool cw2::cartesian_grasp_and_place(
    const geometry_msgs::PointStamped& grasp_point,
    const geometry_msgs::PointStamped& place_point,
    double rotate_angle)
  {
    // 1. 打开夹爪
    move_gripper(0.10);
  
    // 2. 移动到抓取点上方
    if (!move_to_pose(grasp_point, 0.15, true)) {
      ROS_ERROR("Failed to move above the object.");
      return false;
    }
  
    // 3. 旋转末端执行器
    if (!rotate_end_effector(rotate_angle)) {
      ROS_ERROR("Failed to rotate end-effector.");
      return false;
    }
  
    // 4. 降低准备抓取
    if (!move_to_pose(grasp_point, 0.07, false)) {
      ROS_ERROR("Failed to move to grasp height.");
      return false;
    }
  
    // 5. 闭合夹爪
    move_gripper(0.01);
  
    // 6. 记录当前抓取后的姿态作为统一姿态
    geometry_msgs::PoseStamped base_pose_stamped = arm_group_.getCurrentPose();
    geometry_msgs::Pose base_pose = base_pose_stamped.pose;
  
    // 7. 构造 Cartesian waypoints
    std::vector<geometry_msgs::Pose> waypoints;
  
    geometry_msgs::Pose lift_pose = base_pose;
    lift_pose.position.z = grasp_point.point.z + 0.5;
    lift_pose.orientation = base_pose.orientation;
    waypoints.push_back(lift_pose);
  
    geometry_msgs::Pose move_y_pose = lift_pose;
    move_y_pose.position.y = place_point.point.y;
    move_y_pose.orientation = base_pose.orientation;
    waypoints.push_back(move_y_pose);
  
    geometry_msgs::Pose move_x_pose = move_y_pose;
    move_x_pose.position.x = place_point.point.x;
    move_x_pose.orientation = base_pose.orientation;
    waypoints.push_back(move_x_pose);
  
    geometry_msgs::Pose lower_pose = move_x_pose;
    lower_pose.position.z = place_point.point.z + 0.5;
    lower_pose.orientation = base_pose.orientation;
    waypoints.push_back(lower_pose);
  
    // 8. 计算笛卡尔路径（只尝试一次）
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;
  
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, trajectory, true);
    if (fraction >= 0.95) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      arm_group_.execute(plan);
    } else {
      ROS_WARN("Cartesian path planning failed (%.2f%%). Falling back to point-to-point planning.", fraction * 100.0);
  
      // fallback：使用 move_to_pose 分段移动
      // geometry_msgs::PointStamped mid1 = place_point;
      geometry_msgs::PointStamped mid1 = grasp_point;
      mid1.point.z = grasp_point.point.z + 0.5;
  
      geometry_msgs::PointStamped mid2 = place_point;
      mid2.point.z = place_point.point.z + 0.5;
  
      if (!move_to_pose(mid1, 0.0, false)) {
        ROS_ERROR("Fallback move_to_pose: mid1 failed.");
        return false;
      }
  
      if (!move_to_pose(mid2, 0.0, false)) {
        ROS_ERROR("Fallback move_to_pose: mid2 failed.");
        return false;
      }
    }
  
    // 9. 放下物体
    move_gripper(0.10);
  
    // 10. 抬起离开
    geometry_msgs::PointStamped lift_after_place = place_point;
    lift_after_place.point.z = place_point.point.z + 0.5;
  
    if (!move_to_pose(lift_after_place, 0.0, false)) {
      ROS_ERROR("Failed to lift after placing.");
      return false;
    }
  
    return true;
  }