#ifndef CW2_CLASS_H_
#define CW2_CLASS_H_

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/package.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

// PCL / TF
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 特征估计 & 识别
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <Eigen/Core>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

// Service definitions
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

// PCL 类型简化别名
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<DescriptorType> DescriptorCloud;
typedef pcl::Normal NormalType;

// 用于存储 6D 位姿的简单结构
struct Pose6Dof
{
  Eigen::Matrix3f rotation;    
  Eigen::Vector3f translation; 
};

class cw2
{
public:
  cw2(ros::NodeHandle nh);

  // 三个任务的服务回调
  bool t1_callback(cw2_world_spawner::Task1Service::Request &request,
                   cw2_world_spawner::Task1Service::Response &response);

  bool t2_callback(cw2_world_spawner::Task2Service::Request &request,
                   cw2_world_spawner::Task2Service::Response &response);

  bool t3_callback(cw2_world_spawner::Task3Service::Request &request,
                   cw2_world_spawner::Task3Service::Response &response);

  // 控制函数
  bool move_arm(const geometry_msgs::PointStamped& target);
  bool move_gripper(float width);
  bool rotate_end_effector(double yaw);

  // 点云回调函数
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
  ros::NodeHandle nh_;

  // 服务
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  // 点云相关
  ros::Subscriber cloud_sub_;
  ros::Publisher filtered_cloud_pub_;
  PointCloudRGB::Ptr latest_cloud_;
  bool cloud_received_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // MoveIt groups
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface hand_group_;

  // 用于保存或加载的“物体模型”点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud_;

  // ========= 过滤相关函数 =========
  PointCloudRGB::Ptr filter_pass_through(const PointCloudRGB::Ptr& cloud_in);
  PointCloudRGB::Ptr filter_voxel_grid(const PointCloudRGB::Ptr& cloud_in);
  void publish_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                     const std::string& frame_id);


  double computeYawRad(const Eigen::Matrix3f &rotation);
  bool estimatePoseByMatching(const PointCloudRGB::Ptr& scene_cloud,
    const std::string& model_path,
    Eigen::Matrix4f& transform_out);
};

#endif // CW2_CLASS_H_
  
