#include "cw2_class.h"

/**
 * @brief Constructor: initializes ROS interfaces, MoveIt groups, and internal state.
 * 
 * @param nh ROS node handle.
 */
cw2::cw2(ros::NodeHandle nh)
  : nh_(nh),
    arm_group_("panda_arm"),            ///< MoveIt arm group for planning
    hand_group_("hand"),                ///< MoveIt hand (gripper) group
    tf_listener_(tf_buffer_),           ///< TF listener for transforms
    cloud_received_(false)              ///< Flag for incoming cloud
{
  // Advertise Task services
  t1_service_ = nh_.advertiseService("/task1_start", &cw2::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start", &cw2::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start", &cw2::t3_callback, this);

  // Publishers and subscriber for point clouds and markers
  filtered_cloud_pub_   = nh_.advertise<sensor_msgs::PointCloud2>(
      "/r200/camera/depth_registered/filtered_cloud", 1, true);
  cloud_sub_            = nh_.subscribe(
      "/r200/camera/depth_registered/points", 1,
      &cw2::pointCloudCallback, this);
  marker_pub_           = nh_.advertise<visualization_msgs::Marker>(
      "/convex_hull_marker", 1);
  grasp_point_pub_      = nh_.advertise<visualization_msgs::Marker>(
      "/grasp_point_marker", 1);
  grasp_arrow_pub_      = nh_.advertise<visualization_msgs::Marker>(
      "/grasp_orientation_arrow", 1);
  centroid_pub_         = nh_.advertise<visualization_msgs::Marker>(
      "/centroid_marker", 1);
  octomap_pub_          = nh_.advertise<octomap_msgs::Octomap>(
      "/constructed_octomap", 1, true);
  accumulated_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/accumulated_cloud", 1, false);
  cluster_marker_pub_   = nh_.advertise<visualization_msgs::MarkerArray>(
      "/cluster_centroids", 1);
  basket_pub_           = nh_.advertise<visualization_msgs::Marker>(
      "/basket_marker", 1);

  // Initialize cloud pointers
  latest_cloud_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  latest_cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
  model_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Seed random for tie-breaking
  std::srand(static_cast<unsigned>(std::time(nullptr)));
}

/**
 * @brief Task 1 service callback: detect, grasp, and place a single object.
 * 
 * @param request Contains object_point, shape_type, and goal_point.
 * @param response Unused.
 * @return true if Task 1 completes successfully.
 * @return false on any failure.
 */
bool cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
                      cw2_world_spawner::Task1Service::Response &response)
{
  ROS_INFO("=== Task 1 Callback Triggered ===");

  // Move to a safe initial pose and record it
  auto initial = make_point(0.5, 0.0, 0.5);
  if (!move_to_pose(initial, 0.0, true)) return false;
  save_initial_joint_and_pose();

  // Move above the requested object point
  if (!move_to_pose(request.object_point, 0.5, true)) {
    ROS_ERROR("Failed to move above the object.");
    return false;
  }

  // Wait for a point cloud to arrive
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

  // Transform cloud to world frame and apply pass-through filter
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_ros::transformPointCloud("world", *latest_cloud_rgb, *transformed, tf_buffer_);
  auto pass_filtered = filterPassThrough<pcl::PointXYZRGB>(
      transformed, "z", 0.03f, 0.4f);
  publishCloud<pcl::PointXYZRGB>(pass_filtered, "world");

  // Compute convex hull and extract grasp corners
  auto [hull, polygon] = computeConvexHull(pass_filtered);
  auto corners = extract_corner_points(hull, polygon, 1, "world");
  publishConvexHullMarker(corners, marker_pub_, 0);

  // Compute grasp and place points with offsets
  geometry_msgs::Point centroid = computeCentroid(hull);
  auto grasp_point = computeGraspPoint(centroid, corners, request.shape_type);
  publishGraspPointMarker(grasp_point, grasp_point_pub_, 1);
  auto offset = computeOffsetVector(grasp_point, centroid);
  auto place_point = computeAdjustedPlacementPoint(request.goal_point, offset);

  // Compute orientation and perform Cartesian pick-and-place
  double yaw = compute_orientation(hull, grasp_point);
  double yaw_offset = compute_yaw_offset(request.shape_type);
  publishOrientationArrow(grasp_point, yaw, grasp_arrow_pub_, 1);
  if (!cartesian_grasp_and_place(grasp_point, place_point, yaw + yaw_offset)) {
    ROS_ERROR("Cartesian grasping failed!");
    return false;
  }

  // Return to initial pose
  go_to_initial_state();
  return true;
}

/**
 * @brief Task 2 service callback: classify reference objects and a mystery object.
 * 
 * @param request Contains lists of reference and mystery object points.
 * @param response Returns the matched mystery object index.
 * @return true if Task 2 completes successfully.
 * @return false on any failure.
 */
bool cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
                      cw2_world_spawner::Task2Service::Response &response)
{
  ROS_INFO("Task 2 callback triggered.");
  std::vector<std::string> ref_shapes;

  // Detect and classify each reference object
  for (size_t i = 0; i < request.ref_object_points.size(); ++i) {
    const auto& pt = request.ref_object_points[i];
    ROS_INFO_STREAM("Processing reference object #" << i);
    if (!move_to_pose(pt, 0.5, true)) {
      ROS_ERROR_STREAM("Failed to move to reference point #" << i);
      ref_shapes.push_back("unknown");
      continue;
    }

    // Transform and filter top layer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud("world", *latest_cloud_rgb, *tmp, tf_buffer_);
    auto pass = filterTopLayer(tmp);

    // Compute convex hull and centroid
    auto [hull, polys] = computeConvexHull(pass);
    auto centroid = computeCentroid(hull);
    publishCentroidCircleMarker(centroid, 0.005, centroid_pub_, 10 + i);

    // Classify shape regionally
    std::string shape = classifyShapeByCentroidRegion(pass, centroid, 0.005, 20);
    ref_shapes.push_back(shape);
  }

  // Process mystery object similarly
  ROS_INFO("Processing mystery object...");
  std::string mystery_shape = "unknown";
  if (move_to_pose(request.mystery_object_point, 0.5, true)) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud("world", *latest_cloud_rgb, *tmp, tf_buffer_);
    auto pass = filterTopLayer(tmp);
    auto [hull, polys] = computeConvexHull(pass);
    auto centroid = computeCentroid(hull);
    publishCentroidCircleMarker(centroid, 0.005, centroid_pub_, 99);
    mystery_shape = classifyShapeByCentroidRegion(pass, centroid, 0.005, 20);
  } else {
    ROS_ERROR("Failed to move to mystery object point.");
    return false;
  }

  // Match mystery shape to references
  bool matched = false;
  int64_t mystery_index = 0;
  for (size_t i = 0; i < ref_shapes.size(); ++i) {
    if (mystery_shape == ref_shapes[i]) {
      mystery_index = i + 1;
      matched = true;
      break;
    }
  }
  response.mystery_object_num = mystery_index;

  // Log summary
  ROS_INFO("/////////////////////////////////////////////////////////////////////");
  for (size_t i = 0; i < ref_shapes.size(); ++i) {
    ROS_INFO_STREAM("Reference object #" << (i + 1) << ": " << ref_shapes[i]);
  }
  ROS_INFO_STREAM("Mystery object detected as: " << mystery_shape);
  if (matched) {
    ROS_INFO_STREAM("=> Mystery object matches reference object #" << mystery_index);
  } else {
    ROS_WARN("=> Mystery object does not match any reference object.");
  }
  ROS_INFO("/////////////////////////////////////////////////////////////////////");

  return true;
}

/**
 * @brief Task 3 service callback: perform full-area scans, build octomap, and pick & place.
 * 
 * @param request Unused.
 * @param response Unused.
 * @return true if Task 3 completes successfully.
 * @return false on any failure.
 */
bool cw2::t3_callback(cw2_world_spawner::Task3Service::Request &,
                      cw2_world_spawner::Task3Service::Response &)
{
  ROS_INFO("Task-3 start.");

  // Clear previous data
  accumulated_cloud_->clear();
  cloud_received_ = false;
  if (latest_octree_) latest_octree_->clear();

  // Move to initial pose and record state
  auto initial = make_point(0.5, 0.0, 0.5);
  if (!move_to_pose(initial, 0.0, true)) return false;
  save_initial_joint_and_pose();

  // Perform four sub-area scans at different base rotations
  scan_sub_area({make_point(0.5,  0.45, 0.5), make_point(0.5, -0.45, 0.5),
                 make_point(0.3, -0.45, 0.5), make_point(0.3,  0.45, 0.5)});
  go_to_initial_state();
  rotate_joint("base",  M_PI / 2);
  scan_sub_area({make_point(0.10,  0.45, 0.5), make_point(0.10,  0.40, 0.5),
                 make_point(-0.10, 0.40, 0.5), make_point(-0.10, 0.45, 0.5)});
  go_to_initial_state();
  rotate_joint("base", -M_PI / 2);
  scan_sub_area({make_point(0.10, -0.45, 0.5), make_point(0.10, -0.40, 0.5),
                 make_point(-0.10, -0.40, 0.5), make_point(-0.10, -0.45, 0.5)});
  go_to_initial_state();
  scan_sub_area({make_point(-0.5,  -0.45, 0.5), make_point(-0.5,  0.45, 0.5),
                 make_point(-0.40, 0.45, 0.5), make_point(-0.40, -0.45, 0.5)});

  // Build and publish octomap
  build_octomap_from_accumulated_clouds();
  publishAccumulatedCloud();

  // Extract objects from octomap
  std::vector<DetectedObj> det;
  extract_objects(*latest_octree_, true, det);

  // Count noughts vs crosses
  int cnt_nought = 0, cnt_cross = 0;
  for (auto &d : det) {
    if (d.category == "object") {
      if (d.shape == "nought") ++cnt_nought;
      else if (d.shape == "cross") ++cnt_cross;
    }
  }
  if (cnt_nought + cnt_cross == 0) {
    ROS_ERROR("No object detected.");
    return false;
  }

  // Decide target shape randomly if tie
  std::string target_shape = (cnt_nought > cnt_cross) ? "nought" :
                             (cnt_cross > cnt_nought) ? "cross" :
                             ((std::rand() % 2) ? "nought" : "cross");

  // Find basket and target
  DetectedObj basket{}, target{};
  bool basket_ok = false, target_ok = false;
  for (auto &d : det) {
    if (!basket_ok && d.category == "basket") {
      basket = d; basket_ok = true;
    }
    if (!target_ok && d.category == "object" && d.shape == target_shape) {
      target = d; target_ok = true;
    }
  }
  if (!basket_ok || !target_ok) {
    ROS_ERROR("Basket or target object missing.");
    return false;
  }

  go_to_initial_state();

  // Move above target and wait for cloud
  geometry_msgs::PointStamped obj_pt = make_point(
      target.centroid.x, target.centroid.y, target.centroid.z);
  if (!move_to_pose(obj_pt, 0.50, true)) return false;
  ros::Rate rate(10); int wait=0;
  while (!cloud_received_ && wait++<50) {
    ROS_INFO_THROTTLE(1.0, "Waiting for point cloud...");
    rate.sleep();
  }
  if (!cloud_received_) {
    ROS_ERROR("No point cloud received after arm moved.");
    return false;
  }

  // Filter to top layer and by octomap voxels
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transf(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_ros::transformPointCloud("world", *latest_cloud_rgb, *transf, tf_buffer_);
  auto filtered = filterByOctomapVoxels(transf, target.voxel_keys, *latest_octree_);
  publishCloud<pcl::PointXYZRGB>(filtered, "world");
  ROS_INFO("Filtered cloud published");

  // Compute grasp corner & place point
  auto [hull2, poly2] = computeConvexHull(filtered);
  auto corners2 = extract_corner_points(hull2, poly2, 1, "world");
  auto grasp_pt = computeGraspPoint(target.centroid, corners2, target_shape);
  auto offset2 = computeOffsetVector(grasp_pt, target.centroid);
  geometry_msgs::PointStamped bask_pt = make_point(
      basket.centroid.x, basket.centroid.y, basket.centroid.z);
  auto place_pt = computeAdjustedPlacementPoint(bask_pt, offset2);

  // Visualize and perform pick-and-place
  publishConvexHullMarker(corners2, marker_pub_, 0);
  publishGraspPointMarker(grasp_pt, grasp_point_pub_, 1);
  publishBasketMarker(bask_pt, 0);
  double yaw = compute_orientation(hull2, grasp_pt) + compute_yaw_offset(target_shape);
  ROS_INFO("Place point: (%.3f, %.3f, %.3f)",
           place_pt.point.x, place_pt.point.y, place_pt.point.z);

  if (!cartesian_grasp_and_place(grasp_pt, place_pt, yaw)) {
    ROS_ERROR("Pick-place fail.");
    return false;
  }

  go_to_initial_state();
  ROS_INFO("Task-3 done: picked a %s and dropped into basket.",
           target_shape.c_str());
  return true;
}

/**
 * @brief Helper to construct a stamped Point in the world frame.
 * 
 * @param x X coordinate in meters.
 * @param y Y coordinate in meters.
 * @param z Z coordinate in meters.
 * @return geometry_msgs::PointStamped stamped in "world".
 */
geometry_msgs::PointStamped cw2::make_point(double x, double y, double z)
{
  geometry_msgs::PointStamped pt;
  pt.header.frame_id = "world";
  pt.header.stamp    = ros::Time::now();
  pt.point.x = x;
  pt.point.y = y;
  pt.point.z = z;
  return pt;
}

/**
 * @brief Callback to receive raw point clouds and accumulate when scanning.
 * 
 * @param msg Incoming PointCloud2 message.
 */
void cw2::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Convert to PCL clouds
  pcl::fromROSMsg(*msg, *latest_cloud_rgb);
  pcl::fromROSMsg(*msg, *latest_cloud_xyz);
  cloud_received_ = true;

  if (is_scanning_) {
    // Transform to world and downsample before accumulating
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    try {
      pcl_ros::transformPointCloud("world", *latest_cloud_rgb, *tmp, tf_buffer_);
      auto filtered = filterPassThrough<pcl::PointXYZRGB>(tmp, "z", 0.04f, 0.4f);

      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud(filtered);
      vg.setLeafSize(0.003f, 0.003f, 0.003f);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr down(new pcl::PointCloud<pcl::PointXYZRGB>);
      vg.filter(*down);

      *accumulated_cloud_ += *down;  // aggregate for octomap
    } catch (tf2::TransformException &ex) {
      ROS_WARN("TF transform failed: %s", ex.what());
    }
  }
}
