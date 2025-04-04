#include <cw2_class.h>
#include <geometry_msgs/Pose.h>

// PCL transform utils
#include <pcl/common/transforms.h>

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

  latest_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  model_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>); // 之后可以加载目标物体点云

  // 订阅点云
  cloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1,
                             &cw2::pointCloudCallback, this);

  ROS_INFO("cw2 class initialised (no plane RANSAC).");
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
  if (!move_arm(request.object_point)){
    ROS_ERROR("Failed to move above the object.");
    return false;
  }

  // 3. 等待点云
  ros::Rate rate(10);
  int wait = 0;
  while (!cloud_received_ && wait++ < 50){
    ROS_INFO_THROTTLE(1.0, "Waiting for point cloud...");
    rate.sleep();
  }
  if (!cloud_received_) {
    ROS_ERROR("No point cloud received after arm moved.");
    return false;
  }

  // 4. 滤波 + TF转换到 "world"
  auto filtered = filter_voxel_grid(filter_pass_through(latest_cloud_));
  PointCloudRGB::Ptr transformed(new PointCloudRGB);
  pcl_ros::transformPointCloud("world", *filtered, *transformed, tf_buffer_);

  publish_cloud(transformed, "world");

  // 5. 模型匹配并估计 transform
  Eigen::Matrix4f transform;
  if (!estimatePoseByMatching(transformed, model_path, transform)){
    ROS_ERROR("Pose estimation by matching failed.");
    return false;
  }

  // 6. 提取 yaw 角度（Z轴旋转）
  Eigen::Matrix3f rotation = transform.block<3,3>(0,0);
  float yaw = std::atan2(rotation(1,0), rotation(0,0));  // yaw from rotation matrix
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
  pcl::fromROSMsg(*msg, *latest_cloud_);
  cloud_received_ = true;
}

bool cw2::estimatePoseByMatching(const PointCloudRGB::Ptr& scene_cloud,
  const std::string& model_path,
  Eigen::Matrix4f& transform_out)
{
  // 加载模型点云
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(model_path, *model_cloud_) < 0)
  {
  ROS_ERROR("❌ Failed to load model point cloud from %s", model_path.c_str());
  return false;
  }

  // ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
  // ↓↓↓↓↓↓↓↓ 1. 计算法线 ↓↓↓↓↓↓↓↓
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, NormalType> ne;
  ne.setKSearch(10);
  pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>);
  pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>);
  ne.setInputCloud(model_cloud_);
  ne.compute(*model_normals);
  ne.setInputCloud(scene_cloud);
  ne.compute(*scene_normals);

  // ↓↓↓↓↓↓↓↓ 2. 提取关键点 ↓↓↓↓↓↓↓↓
  pcl::UniformSampling<pcl::PointXYZRGB> us;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
  us.setRadiusSearch(0.01f); // 可调
  us.setInputCloud(model_cloud_);
  us.filter(*model_keypoints);
  us.setInputCloud(scene_cloud);
  us.filter(*scene_keypoints);

  // ↓↓↓↓↓↓↓↓ 3. 计算 SHOT 描述子 ↓↓↓↓↓↓↓↓
  pcl::SHOTEstimationOMP<pcl::PointXYZRGB, NormalType, DescriptorType> shot;
  shot.setRadiusSearch(0.02f); // 可调
  DescriptorCloud::Ptr model_desc(new DescriptorCloud);
  DescriptorCloud::Ptr scene_desc(new DescriptorCloud);
  shot.setInputNormals(model_normals);
  shot.setSearchSurface(model_cloud_);
  shot.setInputCloud(model_keypoints);
  shot.compute(*model_desc);
  shot.setInputNormals(scene_normals);
  shot.setSearchSurface(scene_cloud);
  shot.setInputCloud(scene_keypoints);
  shot.compute(*scene_desc);

  // ↓↓↓↓↓↓↓↓ 4. 匹配 SHOT 描述子（KNN） ↓↓↓↓↓↓↓↓
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud(model_desc);
  for (std::size_t i = 0; i < scene_desc->size(); ++i)
  {
  std::vector<int> neigh_indices(1);
  std::vector<float> neigh_sqr_dists(1);
  if (!std::isfinite(scene_desc->at(i).descriptor[0]))
  continue;
  if (match_search.nearestKSearch(scene_desc->at(i), 1, neigh_indices, neigh_sqr_dists) > 0 &&
  neigh_sqr_dists[0] < 0.25f)
  {
  correspondences->emplace_back(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
  }
  }

  // ↓↓↓↓↓↓↓↓ 5. Hough 3D 分组识别位姿 ↓↓↓↓↓↓↓↓
  pcl::Hough3DGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::ReferenceFrame, pcl::ReferenceFrame> hough;
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame>);
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame>);
  pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZRGB, NormalType, pcl::ReferenceFrame> rf_est;
  rf_est.setRadiusSearch(0.015f);
  rf_est.setInputCloud(model_keypoints);
  rf_est.setInputNormals(model_normals);
  rf_est.setSearchSurface(model_cloud_);
  rf_est.compute(*model_rf);
  rf_est.setInputCloud(scene_keypoints);
  rf_est.setInputNormals(scene_normals);
  rf_est.setSearchSurface(scene_cloud);
  rf_est.compute(*scene_rf);

  hough.setInputCloud(model_keypoints);
  hough.setInputRf(model_rf);
  hough.setSceneCloud(scene_keypoints);
  hough.setSceneRf(scene_rf);
  hough.setModelSceneCorrespondences(correspondences);
  hough.setHoughBinSize(0.01f);
  hough.setHoughThreshold(5.0f);

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> transforms;
  std::vector<pcl::Correspondences> clustered_corrs;
  hough.recognize(transforms, clustered_corrs);

  if (transforms.empty())
  {
    ROS_WARN("⚠️ No instance of the model was recognized in the scene.");
  return false;
  }

  transform_out = transforms[0];
  return true;
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

  // 简单设定一个竖直向下的姿态
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

///////////////////////////////////////////////////////////////////////////////
// filter_pass_through
///////////////////////////////////////////////////////////////////////////////
PointCloudRGB::Ptr cw2::filter_pass_through(const PointCloudRGB::Ptr& cloud_in)
{
  PointCloudRGB::Ptr cloud_out(new PointCloudRGB);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 0.4); // 这里可按需调整
  pass.filter(*cloud_out);
  return cloud_out;
}

///////////////////////////////////////////////////////////////////////////////
// filter_voxel_grid
///////////////////////////////////////////////////////////////////////////////
PointCloudRGB::Ptr cw2::filter_voxel_grid(const PointCloudRGB::Ptr& cloud_in)
{
  PointCloudRGB::Ptr cloud_out(new PointCloudRGB);
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
  voxel.setInputCloud(cloud_in);
  voxel.setLeafSize(0.001f, 0.001f, 0.001f);
  voxel.filter(*cloud_out);
  return cloud_out;
}

///////////////////////////////////////////////////////////////////////////////
// publish_cloud
///////////////////////////////////////////////////////////////////////////////
void cw2::publish_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                        const std::string& frame_id)
{
  sensor_msgs::PointCloud2 ros_cloud_msg;
  pcl::toROSMsg(*cloud, ros_cloud_msg);
  ros_cloud_msg.header.frame_id = frame_id;
  ros_cloud_msg.header.stamp = ros::Time::now();
  filtered_cloud_pub_.publish(ros_cloud_msg);
}

