#include <cw1_class.h>  // 记得在 cw1_class.h 中声明 getColorClean
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <vector>

///////////////////////////////////////////////////////////////////////////////
// 静态计数器初始化
///////////////////////////////////////////////////////////////////////////////
int cw1::marker_id_ = 0;

///////////////////////////////////////////////////////////////////////////////
// 构造函数
///////////////////////////////////////////////////////////////////////////////
cw1::cw1(ros::NodeHandle &nh) : g_cloud_ptr(new PointC),      // input point cloud
                                g_cloud_filtered(new PointC), // filtered point cloud
                                g_cloud_filtered_ground(new PointC),
                                g_filter_point(new PointC),
                                g_add_cloud(new PointC),
                                detect_pc(false),
                                detect_box(false),
                                it(nh),
                                tf2_listener_(tf2_buffer_)
{
  nh_ = nh;

  // 1) Advertise 服务
  t1_service_ = nh_.advertiseService("/task1_start",
                                     &cw1::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start",
                                     &cw1::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start",
                                     &cw1::t3_callback, this);

  // 2) Marker发布器 (画出篮筐标记)
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("basket_markers", 1);

  // 3) 发布点云
  g_pub_cloud = nh_.advertise<sensor_msgs::PointCloud2>("basket_cloud", 1, true);

  // 4) VoxelGrid 尺寸
  g_vg_leaf_sz = 0.01;

  // 5) 可能的篮筐坐标
  geometry_msgs::PointStamped basket1;
  basket1.point.x = 0.59; 
  basket1.point.y = -0.34;
  baskets_locs.push_back(basket1);

  geometry_msgs::PointStamped basket2;
  basket2.point.x = 0.59; 
  basket2.point.y = 0.34;
  baskets_locs.push_back(basket2);

  geometry_msgs::PointStamped basket3;
  basket3.point.x = 0.31; 
  basket3.point.y = -0.34;
  baskets_locs.push_back(basket3);

  geometry_msgs::PointStamped basket4;
  basket4.point.x = 0.31; 
  basket4.point.y = 0.34;
  baskets_locs.push_back(basket4);

  // OpenCV 窗口 (调试可视化)
  cv::namedWindow(OPENCV_WINDOW);

  // 颜色阈值表
  lower_bounds = {
    cv::Scalar(100, 43, 46),  // 针对“blue”
    cv::Scalar(0,   127, 67), // 针对“red”
    cv::Scalar(125, 43, 46)   // 针对“purple”
  };
  upper_bounds = {
    cv::Scalar(124, 255, 255), // “blue”
    cv::Scalar(10,  255, 255), // “red”
    cv::Scalar(155, 255, 255)  // “purple”
  };
  color_names = {"🔵blue", "🔴red", "🟣purple"};

  depthImage = cv::Mat::zeros(480, 640, CV_16UC1);

  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////
// 新增函数: 安全获取颜色 + 质心 (若失败,返回 false)
///////////////////////////////////////////////////////////////////////////////
bool cw1::getColorClean(PointCPtr &in_cloud_ptr,
                        std::string &color_out,
                        geometry_msgs::PointStamped &pose_out)
{
  if (in_cloud_ptr->empty())
  {
    ROS_ERROR("[getColorClean] Input cloud is empty => cannot do color recognition!");
    return false;
  }

  // 1) 先做直通滤波
  pcl::PointCloud<pcl::PointXYZRGB> basket_cloud;
  // applyPTNew(in_cloud_ptr, basket_cloud, 0.0, 0.15);
  applyPTNew(in_cloud_ptr, basket_cloud, 0.024, 0.15);
  if (basket_cloud.empty())
  {
    ROS_WARN("[getColorClean] 0~0.15m range => no points => empty!");
    return false;
  }

  // 2) 质心
  pcl::PointXYZRGB centroid;
  pcl::computeCentroid(basket_cloud, centroid);

  // 打印RGB (cast to int 避免 %f/type mismatch)
  ROS_INFO("[getColorClean] Basket centroid => (%.3f, %.3f, %.3f)", 
           centroid.x, centroid.y, centroid.z);
  ROS_INFO("[getColorClean] Raw color => R:%d G:%d B:%d", 
           (int)centroid.r, (int)centroid.g, (int)centroid.b);

  // 默认 none
  color_out = "none";

  // 3) 判断颜色区间
  if ((centroid.r >= 100 && centroid.r <= 255) &&
      (centroid.g >=   0 && centroid.g <= 40 ) &&
      (centroid.b >=   0 && centroid.b <= 40 ))
  {
    color_out = "🔴red";
  }
  else if ((centroid.r >= 0 && centroid.r <= 40) &&
           (centroid.b >= 100 && centroid.b <= 255) &&
           (centroid.g >= 0 && centroid.g <= 40))
  {
    color_out = "🔵blue";
  }
  else if ((centroid.r >= 100 && centroid.r <= 220) &&
           (centroid.b >= 100 && centroid.b <= 220) &&
           (centroid.g >=   0 && centroid.g <= 40 ))
  {
    color_out = "🟣purple";
  }

  // 若仍为 none, 则视为失败
  if (color_out == "none")
  {
    ROS_WARN("[getColorClean] color => none => recognition fail");
    return false;
  }

  // 若成功, 赋值 pose_out
  pose_out.point.x = centroid.x;
  pose_out.point.y = centroid.y;
  pose_out.point.z = centroid.z;

  // 在 rviz 标记
  publishBasketMarker(pose_out, color_out);
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// t1_callback
///////////////////////////////////////////////////////////////////////////////
bool cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
                      cw1_world_spawner::Task1Service::Response &response)
{
  t1_service(request.object_loc, request.goal_loc);
  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// t2_callback
///////////////////////////////////////////////////////////////////////////////
bool cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
                      cw1_world_spawner::Task2Service::Response &response)
{
  // 订阅点云 (叠加识别)
  ros::Subscriber sub_cloud =
      nh_.subscribe("/r200/camera/depth_registered/points", 
                    1,
                    &cw1::cloudCallBackOne,
                    this);

  // 订阅彩色图像(可视化调试)
  color_sub = it.subscribe("/r200/camera/color/image_raw", 1, &cw1::imageCallback, this);

  geometry_msgs::Pose scan_pose;
  scan_pose.orientation.x = 0.9238795; // panda的水平姿态
  scan_pose.orientation.y = -0.3826834;
  scan_pose.orientation.z = 0.0;
  scan_pose.orientation.w = 0.0;

  // 针对 request.basket_locs 里的每个坐标都去扫描并识别颜色
  for (size_t i = 0; i < request.basket_locs.size(); i++)
  {
    // 每次扫描前, 清空点云
    g_add_cloud->clear();
    count_cloud = 1;

    // 移动到扫描位
    scan_pose.position.x = request.basket_locs[i].point.x;
    scan_pose.position.y = request.basket_locs[i].point.y;
    scan_pose.position.z = request.basket_locs[i].point.z + 0.45;
    moveArm(scan_pose);

    // 打开检测, 等待2s
    detect_pc = true;
    ros::Duration(2.0).sleep();

    // 调用安全识别
    std::string color_found;
    geometry_msgs::PointStamped pose_found;
    bool success = getColorClean(g_add_cloud, color_found, pose_found);

    detect_pc = false;

    // 把识别结果写到 baskets_color
    if (success)
    {
      baskets_color.push_back(color_found);
      baskets_locs_real.push_back(pose_found);
      ROS_INFO("[Task2] corner %zu => color: %s, (%.3f, %.3f, %.3f)",
               i, color_found.c_str(), pose_found.point.x, pose_found.point.y, pose_found.point.z);
    }
    else
    {
      baskets_color.push_back("none");
      geometry_msgs::PointStamped dummy;
      dummy.point.x = 0.0; dummy.point.y = 0.0; dummy.point.z = 0.0;
      baskets_locs_real.push_back(dummy);
      ROS_WARN("[Task2] corner %zu => no valid color => set none,(0,0,0)", i);
    }
  }

  // 填充服务响应
  response.basket_colours = baskets_color;
  for (size_t i = 0; i < response.basket_colours.size(); i++)
  {
    ROS_INFO("[Task 2 Response] Basket %zu Colour: %s", i, response.basket_colours[i].c_str());
  }

  // 清理
  baskets_color.clear();
  baskets_locs_real.clear();
  sub_cloud.shutdown();

  ROS_INFO("The coursework solving callback for task 2 has been triggered");
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// t3_callback
///////////////////////////////////////////////////////////////////////////////
bool cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
                      cw1_world_spawner::Task3Service::Response &response)
{
  ROS_INFO("[Task3 Debug] t3_callback has started.");

  // 1) 识别四个篮子
  ROS_INFO("[Task3 Debug] Step1 => getColorBasket()");
  getColorBasket();

  // 2) 再识别box
  ROS_INFO("[Task3 Debug] Step2 => getColorBox()");
  getColorBox();

  // 3) 订阅图像 & depth
  color_sub = it.subscribe("/r200/camera/color/image_raw", 1, &cw1::imageCallback, this);
  depth_sub = it.subscribe("/r200/camera/depth/image_raw", 1, &cw1::depthCallback, this);
  camera_info_sub = nh_.subscribe("/r200/camera/depth/camera_info", 1, &cw1::cameraInfoCb, this);

  // 4) pick_place
  ROS_INFO("[Task3 Debug] Step3 => pick_place()");
  pick_place();

  // 清理
  baskets_locs_real.clear();
  baskets_color.clear();

  ROS_INFO("[Task3 Debug] All tasks for t3_callback are completed.");

  // Debug输出
  ROS_INFO("[Task3 Debug] 最后一次刷新：box_pose = (%.3f, %.3f, %.3f), basket_pose = (%.3f, %.3f, %.3f)",
           box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z,
           basket_pose.pose.position.x, basket_pose.pose.position.y, basket_pose.pose.position.z);

  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// 移动手臂到目标位姿
///////////////////////////////////////////////////////////////////////////////
bool cw1::moveArm(geometry_msgs::Pose target_pose)
{
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  arm_group_.setMaxVelocityScalingFactor(0.7);
  arm_group_.setMaxAccelerationScalingFactor(0.7);
  arm_group_.setPlanningTime(5.0);

  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  if (success)
  {
    ROS_INFO("Executing movement...");
    arm_group_.move();
  }
  else
  {
    ROS_ERROR("Motion planning failed!");
  }
  return success;
}

///////////////////////////////////////////////////////////////////////////////
// 移动手爪到指定开合宽度
///////////////////////////////////////////////////////////////////////////////
bool cw1::moveGripper(float width)
{
  if (width > gripper_open_)
    width = gripper_open_;
  if (width < gripper_closed_)
    width = gripper_closed_;

  double eachJoint = width / 2.0;
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  hand_group_.setJointValueTarget(gripperJointTargets);
  hand_group_.setMaxVelocityScalingFactor(1.0);
  hand_group_.setMaxAccelerationScalingFactor(1.0);
  hand_group_.setPlanningTime(3.0);

  ROS_INFO("Attempting to plan the path for gripper");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising gripper plan %s", success ? "" : "FAILED");
  if (success)
  {
    ROS_INFO("Executing gripper movement...");
    hand_group_.move();
  }
  else
  {
    ROS_ERROR("Gripper motion planning failed!");
  }
  return success;
}

///////////////////////////////////////////////////////////////////////////////
// 笛卡尔路径
///////////////////////////////////////////////////////////////////////////////
double cw1::planCartesianPath(std::vector<geometry_msgs::Pose> waypoints)
{
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0; 
  const double eef_step = 0.01;
  double fraction = 0.0;
  int maxtries = 100;
  int attempts = 0;

  while (fraction < 1.0 && attempts < maxtries)
  {
    fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    attempts++;
    if (attempts % 10 == 0)
      ROS_INFO("Still trying cartesian path after %d attempts...", attempts);
  }

  if (fraction == 1.0)
  {
    ROS_INFO("Path computed successfully. Moving the arm.");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    arm_group_.execute(plan);
    ros::Duration(0.5).sleep();
  }
  else
  {
    ROS_WARN("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
  }
  return fraction;
}

///////////////////////////////////////////////////////////////////////////////
// Task1：Pick & Place
///////////////////////////////////////////////////////////////////////////////
void cw1::t1_service(geometry_msgs::PoseStamped object_loc,
                     geometry_msgs::PointStamped goal_loc)
{
  geometry_msgs::Pose pre_grasp_pose;
  geometry_msgs::Pose post_grasp_pose;
  geometry_msgs::Quaternion grasp_orientation;

  grasp_orientation.x = 0.9238795;
  grasp_orientation.y = -0.3826834;
  grasp_orientation.z = 0.0;
  grasp_orientation.w = 0.0;

  // 物体位置
  pre_grasp_pose.position = object_loc.pose.position;
  pre_grasp_pose.orientation = grasp_orientation;

  // 目标位置
  post_grasp_pose.position = goal_loc.point;
  post_grasp_pose.orientation = grasp_orientation;

  // 1) 移动到物体上方0.3
  pre_grasp_pose.position.z += 0.3;
  moveArm(pre_grasp_pose);
  // 打开爪子
  moveGripper(0.10);

  // 2) 下移到物体处抓取
  pre_grasp_pose.position.z -= 0.19;
  moveArm(pre_grasp_pose);
  moveGripper(0.028);

  // 3) 抬回0.3
  pre_grasp_pose.position.z += 0.3;
  moveArm(pre_grasp_pose);

  // 4) 先XY到篮子正上方
  geometry_msgs::Pose mid_pose = post_grasp_pose;
  mid_pose.position.z = pre_grasp_pose.position.z;
  moveArm(mid_pose);

  // 5) 再下降
  mid_pose.position.z = goal_loc.point.z + 0.3;
  moveArm(mid_pose);

  // 放爪
  moveGripper(0.10);

  // 抬起一点
  mid_pose.position.z += 0.2;
  moveArm(mid_pose);
}

///////////////////////////////////////////////////////////////////////////////
// 点云滤波(体素)
///////////////////////////////////////////////////////////////////////////////
void cw1::filter_point(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  g_vx.setInputCloud(in_cloud_ptr);
  g_vx.setLeafSize(g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter(*out_cloud_ptr);
  return;
}

// 直通滤波
void cw1::applyPT(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr,
                  float thrs_min, float thrs_max)
{
  g_pt.setInputCloud(in_cloud_ptr);
  g_pt.setFilterFieldName("z");
  g_pt.setFilterLimits(thrs_min, thrs_max);
  g_pt.filter(*out_cloud_ptr);
  return;
}

// 直通滤波 + 转换为 pcl::PointXYZRGB
void cw1::applyPTNew(PointCPtr &in_cloud_ptr,
                     pcl::PointCloud<pcl::PointXYZRGB> &out_cloud_ptr,
                     float thrs_min, float thrs_max)
{
  g_pt.setInputCloud(in_cloud_ptr);
  g_pt.setFilterFieldName("z");
  g_pt.setFilterLimits(thrs_min, thrs_max);
  g_pt.filter(out_cloud_ptr);
  return;
}

///////////////////////////////////////////////////////////////////////////////
// 点云回调
///////////////////////////////////////////////////////////////////////////////
void cw1::cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  if (detect_pc)
  {
    if (!cloud_input_msg->width || !cloud_input_msg->height)
    {
      ROS_WARN("[Debug cloudCallBackOne] empty cloud_msg from topic!");
      return;
    }

    pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
    pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

    if (g_cloud_ptr->empty())
    {
      ROS_WARN("[Debug cloudCallBackOne] g_cloud_ptr is empty after fromPCLPointCloud2()!");
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_camera(new PointC);

    // 等待 TF
    if (!g_listener2_.waitForTransform(base_frame_, g_cloud_ptr->header.frame_id,
                                       ros::Time(0), ros::Duration(3.0)))
    {
      ROS_ERROR("[Debug cloudCallBackOne] TF from %s to %s not available!",
                g_cloud_ptr->header.frame_id.c_str(), base_frame_.c_str());
      return;
    }

    // 转换点云到 base_frame
    pcl_ros::transformPointCloud(base_frame_, *g_cloud_ptr, *input_camera, g_listener2_);

    if (input_camera->empty())
    {
      ROS_WARN("[Debug cloudCallBackOne] input_camera empty after transform!");
      return;
    }

    // 体素
    filter_point(input_camera, g_filter_point);

    if (g_filter_point->empty())
    {
      ROS_WARN("[Debug cloudCallBackOne] g_filter_point is empty after voxel!");
    }

    // 直通
    applyPT(input_camera, g_cloud_filtered_ground, 0.024, 0.15);

    if (g_cloud_filtered_ground->empty())
    {
      ROS_WARN("[Debug cloudCallBackOne] g_cloud_filtered_ground empty after passThrough!");
    }

    // 累加
    if (count_cloud == 1)
    {
      *g_add_cloud = *g_cloud_filtered_ground;
      count_cloud++;
    }
    else
    {
      *g_add_cloud += *g_cloud_filtered_ground;
    }

    if (g_add_cloud->empty())
    {
      ROS_WARN("[Debug cloudCallBackOne] g_add_cloud empty after accumulation!");
    }

    // 发布以查看
    sensor_msgs::PointCloud2 filtered_ros_msg;
    pcl::toROSMsg(*g_add_cloud, filtered_ros_msg);
    g_pub_cloud.publish(filtered_ros_msg);
  }
  else
  {
    g_add_cloud->clear();
    g_cloud_filtered_ground->clear();
    count_cloud = 1;
  }
}

///////////////////////////////////////////////////////////////////////////////
// 旧的 getColor 逻辑 (可选保留，用于调试)
///////////////////////////////////////////////////////////////////////////////
void cw1::getColor(PointCPtr &in_cloud_ptr)
{
  ROS_INFO("[Debug getColor] Step1: 准备检测篮子颜色...");

  if (g_add_cloud->empty())
  {
    ROS_ERROR("[Debug getColor] ❗ g_add_cloud is empty => cannot do color recognition!");
    baskets_color.push_back("none");
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB> each_basket;
  // applyPTNew(g_add_cloud, each_basket, 0.0, 0.15);
  applyPTNew(g_add_cloud, each_basket, 0.024, 0.15);

  if (each_basket.empty())
  {
    ROS_WARN("[Debug getColor] ❗ each_basket is empty => no points in 0~0.15 range!");
    baskets_color.push_back("none");
    return;
  }

  ROS_INFO("[Debug getColor] Step2: each_basket size = %ld", each_basket.size());

  pcl::PointXYZRGB each_centroid;
  pcl::computeCentroid(each_basket, each_centroid);

  ROS_INFO("[Debug getColor] Step3: Basket centroid => (%.3f, %.3f, %.3f)",
           each_centroid.x, each_centroid.y, each_centroid.z);

  // 注意这里 cast to int
  ROS_INFO("[Debug getColor] Raw color => R:%d G:%d B:%d",
           (int)each_centroid.r, (int)each_centroid.g, (int)each_centroid.b);

  std::string recognized_color = "none";
  if ((each_centroid.r >= 100 && each_centroid.r <= 255) &&
      (each_centroid.g >=   0 && each_centroid.g <= 40 ) &&
      (each_centroid.b >=   0 && each_centroid.b <= 40 ))
  {
    recognized_color = "🔴red";
  }
  else if ((each_centroid.r >= 0 && each_centroid.r <= 40) &&
           (each_centroid.b >= 100 && each_centroid.b <= 255) &&
           (each_centroid.g >= 0 && each_centroid.g <= 40))
  {
    recognized_color = "🔵blue";
  }
  else if ((each_centroid.r >= 100 && each_centroid.r <= 220) &&
           (each_centroid.b >= 100 && each_centroid.b <= 220) &&
           (each_centroid.g >=   0 && each_centroid.g <= 40 ))
  {
    recognized_color = "🟣purple";
  }

  baskets_color.push_back(recognized_color);

  if (recognized_color != "none")
  {
    geometry_msgs::PointStamped basket_real_pose;
    basket_real_pose.point.x = each_centroid.x;
    basket_real_pose.point.y = each_centroid.y;
    basket_real_pose.point.z = each_centroid.z;

    publishBasketMarker(basket_real_pose, recognized_color);
  }
}

///////////////////////////////////////////////////////////////////////////////
// 在 RViz 中发布一个平行 Z 轴的彩色线段
///////////////////////////////////////////////////////////////////////////////
void cw1::publishBasketMarker(const geometry_msgs::PointStamped &basket_pose,
                              const std::string &color_string)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basket_namespace";
  marker.id = marker_id_++;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.01;
  marker.pose.orientation.w = 1.0;

  if (color_string.find("red") != std::string::npos)
  {
    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
  }
  else if (color_string.find("blue") != std::string::npos)
  {
    marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 1.0;
  }
  else if (color_string.find("purple") != std::string::npos)
  {
    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 1.0;
  }
  else
  {
    marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
  }

  geometry_msgs::Point p1, p2;
  p1.x = basket_pose.point.x;
  p1.y = basket_pose.point.y;
  p1.z = basket_pose.point.z;

  p2 = p1; 
  p2.z += 0.2; // 画一条竖线

  marker.points.push_back(p1);
  marker.points.push_back(p2);

  marker_pub_.publish(marker);
}

///////////////////////////////////////////////////////////////////////////////
// 扫描四个角落并识别
///////////////////////////////////////////////////////////////////////////////
// void cw1::moveAround(geometry_msgs::Pose target_pose)
// {
//   // std::vector<geometry_msgs::Pose> waypoint;
//   // waypoint.push_back(target_pose);

//   // target_pose.position.x += 0.05;
//   // waypoint.push_back(target_pose);

//   // target_pose.position.x -= 0.1;
//   // waypoint.push_back(target_pose);

//   // target_pose.position.x += 0.05;
//   // target_pose.position.y += 0.05;
//   // waypoint.push_back(target_pose);

//   // target_pose.position.y -= 0.1;
//   // waypoint.push_back(target_pose);

//   // planCartesianPath(waypoint);

// }

void cw1::moveAround(geometry_msgs::Pose target_pose)
{
  // 将要发送给 planCartesianPath 的一系列点
  std::vector<geometry_msgs::Pose> waypoints;
  
  // 0) 从当前“中心点”开始
  waypoints.push_back(target_pose);

  // 1) 向左移动 0.05 (到达矩形左边)
  geometry_msgs::Pose left_pose = target_pose;
  left_pose.position.x -= 0.05;
  waypoints.push_back(left_pose);

  // 2) 上移到矩形左上角 (x 不变, y + 0.05)
  geometry_msgs::Pose left_upper = left_pose;
  left_upper.position.y += 0.05;
  waypoints.push_back(left_upper);

  // 3) 向右移动到矩形右上角 (x + 0.1, y不变)
  geometry_msgs::Pose right_upper = left_upper;
  right_upper.position.x += 0.10;
  waypoints.push_back(right_upper);

  // 4) 向下到矩形右下角 (y - 0.1)
  geometry_msgs::Pose right_lower = right_upper;
  right_lower.position.y -= 0.10;
  waypoints.push_back(right_lower);

  // 5) 向左到矩形左下角 (x - 0.1)
  geometry_msgs::Pose left_lower = right_lower;
  left_lower.position.x -= 0.10;
  waypoints.push_back(left_lower);

  // 6) 回到中心点
  waypoints.push_back(target_pose);

  // 将以上轨迹发送给 planCartesianPath
  planCartesianPath(waypoints);
}


void cw1::getColorBasket()
{
  ROS_INFO("[Debug getColorBasket] 订阅点云...");
  ros::Subscriber sub_cloud =
      nh_.subscribe("/r200/camera/depth_registered/points",
                    1,
                    &cw1::cloudCallBackOne,
                    this);

  geometry_msgs::Pose scan_pose;
  scan_pose.orientation.x = 0.9238795;
  scan_pose.orientation.y = -0.3826834;
  scan_pose.orientation.z = 0.0;
  scan_pose.orientation.w = 0.0;
  scan_pose.position.z = 0.40;

  // 清空之前记录
  baskets_color.clear();
  baskets_locs_real.clear();

  for (int i = 0; i < (int)baskets_locs.size(); i++)
  {
    g_add_cloud->clear();
    count_cloud = 1;

    ROS_INFO("[Debug getColorBasket] 扫描篮子 %d => (x=%.3f, y=%.3f)",
             i, baskets_locs[i].point.x, baskets_locs[i].point.y);

    scan_pose.position.x = baskets_locs[i].point.x;
    scan_pose.position.y = baskets_locs[i].point.y;

    moveAround(scan_pose);

    detect_pc = true;
    ros::Duration(2.0).sleep();

    // 调用 getColorClean
    std::string color_found;
    geometry_msgs::PointStamped pose_found;
    bool success = getColorClean(g_add_cloud, color_found, pose_found);
    detect_pc = false;

    if (success)
    {
      baskets_color.push_back(color_found);
      baskets_locs_real.push_back(pose_found);
      ROS_INFO("Basket %d => color: %s, center(%.3f,%.3f,%.3f)",
               i, color_found.c_str(),
               pose_found.point.x, pose_found.point.y, pose_found.point.z);
    }
    else
    {
      baskets_color.push_back("none");
      geometry_msgs::PointStamped dummy;
      dummy.point.x = 0.0; 
      dummy.point.y = 0.0; 
      dummy.point.z = 0.0;
      baskets_locs_real.push_back(dummy);
      ROS_WARN("Basket %d => no valid color => (none,0,0,0)", i);
    }
  }

  ROS_INFO("=== [Task3 Debug] Finished scanning all 4 corners ===");
  for (int i = 0; i < (int)baskets_color.size(); i++)
  {
    ROS_INFO("Corner %d -> Color: %s, Center(%.3f, %.3f, %.3f)",
             i, baskets_color[i].c_str(),
             baskets_locs_real[i].point.x,
             baskets_locs_real[i].point.y,
             baskets_locs_real[i].point.z);
  }
  ROS_INFO("==========================================");

  sub_cloud.shutdown();
}

///////////////////////////////////////////////////////////////////////////////
// 图像回调 (检测 box)
///////////////////////////////////////////////////////////////////////////////
void cw1::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (!detect_box)  // 若不检测，则跳过
    return;

  colorImage = cv_ptr->image;
  cv::Mat hsvImage;
  cv::cvtColor(colorImage, hsvImage, cv::COLOR_BGR2HSV);

  for (size_t k = 0; k < color_names.size(); k++)
  {
    ROS_INFO("[Debug imageCallback] Checking color: %s", color_names[k].c_str());
    cv::Mat mask;
    cv::inRange(hsvImage, lower_bounds[k], upper_bounds[k], mask);
    int count = cv::countNonZero(mask);
    if (count > 0)
    {
      ROS_INFO("[Debug imageCallback] Detected %s color with %d pixels", color_names[k].c_str(), count);
    }

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (auto &contour : contours)
    {
      cv::Rect bounding_rect = cv::boundingRect(contour);
      ROS_INFO("[Debug imageCallback] boundingRect => x=%d, y=%d, w=%d, h=%d, area=%d",
               bounding_rect.x, bounding_rect.y, bounding_rect.width, bounding_rect.height,
               bounding_rect.area());
      if (bounding_rect.area() < 2000 && count > 0)
      {
        cv::rectangle(colorImage, bounding_rect, cv::Scalar(0, 255, 0), 2);
        cv::Point center = (bounding_rect.br() + bounding_rect.tl()) / 2;
        cv::circle(colorImage, center, 5, cv::Scalar(255, 0, 0), 2);

        box_color.push_back(color_names[k].c_str());
        // 与 baskets_color 对比
        for (int j = 0; j < (int)baskets_color.size(); j++)
        {
          if (color_names[k] == baskets_color[j])
          {
            if (depthImage.at<uint16_t>(center.y, center.x) == 0)
            {
              ROS_WARN("[Debug imageCallback] Depth=0 => no valid 3D data!");
              continue;
            }
            ROS_INFO("[Debug imageCallback] Valid depth at pixel (%d, %d): %u",
                     center.x, center.y, depthImage.at<uint16_t>(center.y, center.x));

            box_pose = getBoxPose(center.x, center.y);

            // 对应篮子的坐标
            basket_pose.pose.position.x = baskets_locs_real[j].point.x;
            basket_pose.pose.position.y = baskets_locs_real[j].point.y;
            basket_pose.pose.position.z = baskets_locs_real[j].point.z;

            detect_box = false;
            ROS_INFO("[Debug imageCallback] ✅Detect box => X: %.3f, Y: %.3f, Z: %.3f",
                     box_pose.pose.position.x,
                     box_pose.pose.position.y,
                     box_pose.pose.position.z);
            return;
          }
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// 深度图回调
///////////////////////////////////////////////////////////////////////////////
void cw1::depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  depthImage = cv_ptr->image;
}

///////////////////////////////////////////////////////////////////////////////
// 相机内参回调
///////////////////////////////////////////////////////////////////////////////
void cw1::cameraInfoCb(const sensor_msgs::CameraInfo &msg)
{
  camera_info = msg;
}

///////////////////////////////////////////////////////////////////////////////
// 像素坐标 -> 世界坐标Pose
///////////////////////////////////////////////////////////////////////////////
geometry_msgs::PoseStamped cw1::getBoxPose(int pixel_x, int pixel_y)
{
  float real_z = depthImage.at<uint16_t>(pixel_y, pixel_x) / 1000.0;
  float real_x = (pixel_x - camera_info.K.at(2)) / camera_info.K.at(0) * real_z;
  float real_y = (pixel_y - camera_info.K.at(5)) / camera_info.K.at(4) * real_z;

  ROS_INFO("[Debug getBoxPose] pixel (%d,%d)->(%.3f,%.3f,%.3f)",
           pixel_x, pixel_y, real_x, real_y, real_z);

  geometry_msgs::PoseStamped pose_before, pose_transformed;
  pose_before.header.frame_id = "color"; 
  pose_before.header.stamp = ros::Time();
  pose_before.pose.position.x = real_x;
  pose_before.pose.position.y = real_y;
  pose_before.pose.position.z = real_z;
  pose_before.pose.orientation.w = 1.0;

  try
  {
    pose_transformed = tf2_buffer_.transform(pose_before, "world");
    return pose_transformed;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("[Debug getBoxPose]: %s", ex.what());
    ros::Duration(1.0).sleep();
  }
  return pose_transformed;
}

///////////////////////////////////////////////////////////////////////////////
// 让机械臂移动到box上方以便识别
///////////////////////////////////////////////////////////////////////////////
void cw1::getColorBox()
{
  std::vector<double> joint_group_positions(7);
  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = -0.265;
  joint_group_positions[2] = -0.006;
  joint_group_positions[3] = -1.460;
  joint_group_positions[4] = -0.001;
  joint_group_positions[5] = 1.185;
  joint_group_positions[6] = 0.7802;

  arm_group_.setJointValueTarget(joint_group_positions);
  arm_group_.move();
  ROS_INFO("[Debug getColorBox] Moved to scanning pose for box detection.");
}

///////////////////////////////////////////////////////////////////////////////
// pick_place
///////////////////////////////////////////////////////////////////////////////
void cw1::pick_place()
{
  // 假设最多抓6个盒子
  for (int i = 0; i < 6; i++)
  {
    getColorBox();
    detect_box = true;
    ros::Duration(2.0).sleep();

    if (!detect_box)
    {
      ROS_INFO("[Task3 Debug pick_place] Found a matching color box.");
      ROS_INFO("[Task3 Debug pick_place] All recognized box colors so far:");
      for (size_t cc = 0; cc < box_color.size(); cc++)
      {
        ROS_INFO(" -> %s", box_color[cc].c_str());
      }
      if (!box_color.empty())
      {
        ROS_INFO("[Task3 Debug pick_place] Current pick box color: %s", box_color.back().c_str());
      }
      ROS_INFO("[Task3 Debug pick_place] Target basket => (%.3f, %.3f, %.3f)",
               basket_pose.pose.position.x,
               basket_pose.pose.position.y,
               basket_pose.pose.position.z);

      // 1) box上方 +0.15
      box_pose.pose.orientation.x = 0.9238795;
      box_pose.pose.orientation.y = -0.3826834;
      box_pose.pose.orientation.z = 0.0;
      box_pose.pose.orientation.w = 0.0;
      box_pose.pose.position.z += 0.15;

      std::vector<geometry_msgs::Pose> waypoint;
      waypoint.push_back(box_pose.pose);
      planCartesianPath(waypoint);

      // 打开爪
      moveGripper(0.10);

      // 2) 向下0.07
      box_pose.pose.position.z -= 0.07;
      waypoint.clear();
      waypoint.push_back(box_pose.pose);
      planCartesianPath(waypoint);

      // 合拢手爪
      moveGripper(0.025);

      // 3) 抬起 +0.30
      box_pose.pose.position.z += 0.30;
      waypoint.clear();
      waypoint.push_back(box_pose.pose);
      planCartesianPath(waypoint);

      // 4) 移到篮子上方
      geometry_msgs::Pose above_basket = basket_pose.pose;
      above_basket.orientation = box_pose.pose.orientation;
      above_basket.position.z = box_pose.pose.position.z;
      waypoint.clear();
      waypoint.push_back(above_basket);
      planCartesianPath(waypoint);

      // 5) 下探到篮子
      above_basket.position.z = basket_pose.pose.position.z + 0.3;
      waypoint.clear();
      waypoint.push_back(above_basket);
      planCartesianPath(waypoint);

      // 放开
      moveGripper(0.10);

      // 抬起
      above_basket.position.z += 0.2;
      waypoint.clear();
      waypoint.push_back(above_basket);
      planCartesianPath(waypoint);

      // 清空 box_color
      box_color.clear();
    }
    else
    {
      ROS_WARN("[Task3 Debug pick_place] detect_box still true => no new box found!");
    }
  }
}
