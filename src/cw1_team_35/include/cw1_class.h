#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>

// include services from the spawner package
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

// TF
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// visualization
#include <visualization_msgs/Marker.h> // Marker for RViz visualization

// 定义点云别名
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

// 用于 cv::imshow 的窗口名
static const std::string OPENCV_WINDOW = "Image window";

class cw1
{
public:
  /* -------------------------------------------------------------------------
   *                            构造函数
   * ------------------------------------------------------------------------*/
  explicit cw1(ros::NodeHandle &nh);

  /* -------------------------------------------------------------------------
   *                       服务回调函数 (Task1, Task2, Task3)
   * ------------------------------------------------------------------------*/
  bool t1_callback(cw1_world_spawner::Task1Service::Request  &request,
                   cw1_world_spawner::Task1Service::Response &response);

  bool t2_callback(cw1_world_spawner::Task2Service::Request  &request,
                   cw1_world_spawner::Task2Service::Response &response);

  bool t3_callback(cw1_world_spawner::Task3Service::Request  &request,
                   cw1_world_spawner::Task3Service::Response &response);

  /* -------------------------------------------------------------------------
   *                       任务逻辑和工具函数
   * ------------------------------------------------------------------------*/
  // Task1 具体执行逻辑
  void t1_service(geometry_msgs::PoseStamped object_loc,
                  geometry_msgs::PointStamped goal_loc);

  // 移动手臂到给定Pose
  bool moveArm(geometry_msgs::Pose target_pose);

  // 移动夹爪到给定宽度
  bool moveGripper(float width);

  // 以笛卡尔方式规划移动轨迹
  double planCartesianPath(std::vector<geometry_msgs::Pose> waypoints);

  // 点云滤波
  void filter_point(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);
  void applyPT(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr,
               float thrs_min, float thrs_max);
  void applyPTNew(PointCPtr &in_cloud_ptr,
                  pcl::PointCloud<pcl::PointXYZRGB> &out_cloud_ptr,
                  float thrs_min, float thrs_max);

  // 深度相机点云回调 (Task2 / Task3)
  void cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  // 识别篮子颜色
  void getColor(PointCPtr &in_cloud_ptr);

  // 移动到预设位置以识别场景中的方块 (Task3)
  void getColorBox();

  // 扫描四个角，识别篮子位置 (Task3)
  void getColorBasket();

  // 在扫描篮子时，移动机械臂的方式
  void moveAround(geometry_msgs::Pose target_pose);

  // RGB 图像回调 (Task3，用于识别盒子)
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  // 深度图像回调 (用于获取像素深度)
  void depthCallback(const sensor_msgs::ImageConstPtr &msg);

  // 相机内参回调
  void cameraInfoCb(const sensor_msgs::CameraInfo &msg);

  // pick&place：持续拾取并放置多个盒子 (Task3)
  void pick_place();

  // 像素坐标 -> 世界坐标
  geometry_msgs::PoseStamped getBoxPose(int pixel_x, int pixel_y);

  /**
   * @brief 在 RViz 中发布一个线状 Marker，显示识别到的篮子位置
   *
   * @param basket_pose 篮子的坐标（世界坐标系下）
   * @param color_string 识别到的颜色字符串 (e.g. "🔴red", "🔵blue", "🟣purple")
   */
  void publishBasketMarker(const geometry_msgs::PointStamped &basket_pose,
                          const std::string &color_string);

  bool getColorClean(PointCPtr &in_cloud_ptr,
                      std::string &color_out,
                      geometry_msgs::PointStamped &pose_out);
  /* -------------------------------------------------------------------------
   *                            类内成员变量
   * ------------------------------------------------------------------------*/
  // 常量 / 基础配置
  std::string base_frame_ = "panda_link0";  // 用于 transform
  double gripper_open_   = 80e-3;           // 夹爪最大张开
  double gripper_closed_ = 0.0;             // 夹爪闭合最小
  int g_point_cloud_switch = 0;
  int count_cloud = 1;
  ros::Publisher filtered_cloud_pub_;
  sensor_msgs::PointCloud2 filtered_cloud_msg_
  // ROS相关
  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  // MoveIt
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  // 图像&点云
  image_transport::ImageTransport it;
  image_transport::Subscriber color_sub;
  image_transport::Subscriber depth_sub;
  ros::Subscriber camera_info_sub;
  ros::Subscriber sub_cloud;
  ros::Publisher g_pub_cloud;

  // 在 RViz 中发布 Marker
  ros::Publisher marker_pub_;    // 发布器
  static int marker_id_;         // 静态ID计数器

  // PCL相关
  PointCPtr g_cloud_ptr;
  PointCPtr g_cloud_filtered;
  PointCPtr g_cloud_filtered_ground;
  PointCPtr g_filter_point;
  PointCPtr g_add_cloud;

  pcl::PCLPointCloud2 g_pcl_pc;
  pcl::VoxelGrid<PointT> g_vx;
  pcl::PassThrough<PointT> g_pt;
  double g_vg_leaf_sz;

  // TF相关
  tf::TransformListener g_listener_, g_listener2_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  // 识别过程中的标志
  bool detect_pc;   // 是否叠加点云(用于basket识别)
  bool detect_box;  // 是否识别box

  // 记录篮子 / 盒子的识别信息
  std::vector<std::string> baskets_color;
  std::vector<geometry_msgs::PointStamped> baskets_locs;
  std::vector<geometry_msgs::PointStamped> baskets_locs_real;

  std::vector<std::string> box_color;
  std::vector<geometry_msgs::PointStamped> box_locs;

  // 用于颜色阈值检测
  std::vector<cv::Scalar> lower_bounds;
  std::vector<cv::Scalar> upper_bounds;
  std::vector<std::string> color_names;

  // 相机信息
  sensor_msgs::CameraInfo camera_info;
  cv::Mat colorImage;
  cv::Mat depthImage;

  // 当前检测到的 box / basket 位姿
  geometry_msgs::PoseStamped box_pose;
  geometry_msgs::PoseStamped basket_pose;
};

#endif // CW1_CLASS_H_
