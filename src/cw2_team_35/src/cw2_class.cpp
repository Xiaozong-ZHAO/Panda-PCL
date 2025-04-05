#include <cw2_class.h>
#include <geometry_msgs/Pose.h>

// PCL transform utils
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>  // 如果你需要 transformPointCloud

///////////////////////////////////////////////////////////////////////////////
// 构造函数
///////////////////////////////////////////////////////////////////////////////
cw2::cw2(ros::NodeHandle nh)
  : nh_(nh),
    arm_group_("panda_arm"),
    hand_group_("hand"),
    tf_listener_(tf_buffer_),
    cloud_received_(false)
{
  // 注册服务
  t1_service_ = nh_.advertiseService("/task1_start", &cw2::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start", &cw2::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start", &cw2::t3_callback, this);

  // 发布滤波后点云
  filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/r200/camera/depth_registered/filtered_cloud", 1, true);
  // 订阅点云
  cloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1,
    &cw2::pointCloudCallback, this);

  latest_cloud_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  latest_cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
  model_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  ROS_INFO("cw2 class initialised (template-based PCL).");
}

///////////////////////////////////////////////////////////////////////////////
// Task 1 回调函数
///////////////////////////////////////////////////////////////////////////////
bool cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
                      cw2_world_spawner::Task1Service::Response &response)
{
  ROS_INFO("=== Task 1 Callback Triggered ===");

  // 1. 保存 shape_type，用于加载模型
  std::string pkg_path = ros::package::getPath("cw2_team_35");
  std::string model_path;
  if (request.shape_type == "nought")
    model_path = pkg_path + "/data/nought_40mm.pcd";
  else if (request.shape_type == "cross")
    model_path = pkg_path + "/data/cross_40mm.pcd";
  else {
    ROS_ERROR("Unknown shape type: %s", request.shape_type.c_str());
    return false;
  }

  // 2. 移动到物体正上方
  if (!move_arm(request.object_point)) {
    ROS_ERROR("Failed to move above the object.");
    return false;
  }

  // 3. 等待点云
  ros::Rate rate(10);
  int wait = 0;
  while (!cloud_received_ && wait++ < 50) {
    ROS_INFO_THROTTLE(1.0, "Waiting for point cloud...");
    rate.sleep();
  }
  if (!cloud_received_) {
    ROS_ERROR("No point cloud received after arm moved.");
    return false;
  }

  // 4. 使用模板函数对点云滤波 + TF转换到 "world"
  //    （假设我们只需要处理 latest_cloud_xyz）
  auto pass_filtered = filterPassThrough<pcl::PointXYZ>(
      latest_cloud_xyz,    // 输入点云
      "z",                 // 过滤字段
      0.0f,                // z最小
      0.4f                 // z最大
  );
  auto voxel_filtered = filterVoxelGrid<pcl::PointXYZ>(
      pass_filtered,       // 输入
      0.001f               // voxel分辨率
  );

  PointCloudXYZ::Ptr transformed(new PointCloudXYZ);
  pcl_ros::transformPointCloud("world", *pass_filtered, *transformed, tf_buffer_);

  // 发布滤波后的点云
  publishCloud<pcl::PointXYZ>(transformed, "world");

  // 5. 模板函数做模型匹配并估计 transform
  //    （这里假设你将位姿估计函数命名为 estimatePoseByMatchingGeneric）
  Eigen::Matrix4f transform;
  if (!estimatePoseByMatchingGeneric<pcl::PointXYZ>(transformed, model_path, transform)) {
    ROS_ERROR("Pose estimation by matching failed.");
    return false;
  }

  // 6. 提取 yaw 角度（Z轴旋转）
  Eigen::Matrix3f rotation = transform.block<3,3>(0,0);
  float yaw = std::atan2(rotation(1,0), rotation(0,0));
  ROS_INFO("Estimated yaw: %.3f radians", yaw);

  // 7. 旋转末端执行器
  if (!rotate_end_effector(yaw)){
    ROS_ERROR("Failed to rotate end-effector.");
    return false;
  }

  // 8. 张开夹爪
  move_gripper(0.10);

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Task 2 回调函数
///////////////////////////////////////////////////////////////////////////////
bool cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
                      cw2_world_spawner::Task2Service::Response &response)
{
  ROS_INFO("Task 2 callback triggered.");
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Task 3 回调函数
///////////////////////////////////////////////////////////////////////////////
bool cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
                      cw2_world_spawner::Task3Service::Response &response)
{
  ROS_INFO("Task 3 callback triggered.");
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// 点云回调函数
///////////////////////////////////////////////////////////////////////////////
void cw2::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // 同时保存XYZRGB与XYZ以供后续使用
  pcl::fromROSMsg(*msg, *latest_cloud_rgb);
  pcl::fromROSMsg(*msg, *latest_cloud_xyz);
  cloud_received_ = true;
}

///////////////////////////////////////////////////////////////////////////////
// move_arm: 机械臂移动到目标点正上方
///////////////////////////////////////////////////////////////////////////////
bool cw2::move_arm(const geometry_msgs::PointStamped& target)
{
  geometry_msgs::Pose target_pose;
  target_pose.position.x = target.point.x;
  target_pose.position.y = target.point.y;
  target_pose.position.z = target.point.z + 0.5;

  // 简单设定一个竖直向下的姿态 (仅供示例)
  target_pose.orientation.x = 0.9238795;
  target_pose.orientation.y = -0.3826834;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;

  arm_group_.setPoseTarget(target_pose);
  arm_group_.setMaxVelocityScalingFactor(0.7);
  arm_group_.setMaxAccelerationScalingFactor(0.7);
  arm_group_.setPlanningTime(5.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = 
      (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    arm_group_.move();
  } else {
    ROS_ERROR("Arm planning failed!");
  }
  return success;
}

///////////////////////////////////////////////////////////////////////////////
// move_gripper: 控制夹爪张开/闭合
///////////////////////////////////////////////////////////////////////////////
bool cw2::move_gripper(float width)
{
  const float gripper_max = 0.10;
  const float gripper_min = 0.028;

  // clamp一下
  width = std::max(gripper_min, std::min(gripper_max, width));
  std::vector<double> joint_targets(2, width / 2.0);

  hand_group_.setJointValueTarget(joint_targets);
  hand_group_.setMaxVelocityScalingFactor(1.0);
  hand_group_.setMaxAccelerationScalingFactor(1.0);
  hand_group_.setPlanningTime(3.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = 
      (hand_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    hand_group_.move();
  } else {
    ROS_ERROR("Gripper planning failed!");
  }
  return success;
}

///////////////////////////////////////////////////////////////////////////////
// rotate_end_effector: 绕Z轴旋转到给定yaw
///////////////////////////////////////////////////////////////////////////////
bool cw2::rotate_end_effector(double yaw)
{
  geometry_msgs::PoseStamped current_pose_stamped = arm_group_.getCurrentPose();
  geometry_msgs::Pose current_pose = current_pose_stamped.pose;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();

  geometry_msgs::Pose new_pose = current_pose;
  new_pose.orientation = tf2::toMsg(q);

  arm_group_.setPoseTarget(new_pose);
  arm_group_.setMaxVelocityScalingFactor(0.7);
  arm_group_.setMaxAccelerationScalingFactor(0.7);
  arm_group_.setPlanningTime(5.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =
      (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    arm_group_.move();
  } else {
    ROS_ERROR("rotate_end_effector planning failed!");
  }
  return success;
}
