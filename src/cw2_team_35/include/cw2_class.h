#ifndef CW2_CLASS_H_
#define CW2_CLASS_H_

// ========== C++ standard library==========
#include <algorithm>
#include <thread>
#include <chrono>
#include <unordered_set>

// ========== ROS ==========
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// ========== MoveIt ==========
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// ========== Octomap ==========
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// ========== PCL / TF ==========
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>

// ========== Services ==========
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

// ========== PCL Type Aliases ==========
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<DescriptorType> DescriptorCloud;
typedef pcl::Normal NormalType;

class cw2
{
public:
  cw2(ros::NodeHandle nh);

  // ===== Service Callbacks =====
  bool t1_callback(cw2_world_spawner::Task1Service::Request &req,
                   cw2_world_spawner::Task1Service::Response &res);
  bool t2_callback(cw2_world_spawner::Task2Service::Request &req,
                   cw2_world_spawner::Task2Service::Response &res);
  bool t3_callback(cw2_world_spawner::Task3Service::Request &req,
                   cw2_world_spawner::Task3Service::Response &res);

  struct KeyHash {
    size_t operator()(const octomap::OcTreeKey& k) const {
      return (static_cast<size_t>(k.k[0]) << 42) ^
             (static_cast<size_t>(k.k[1]) << 21) ^
             static_cast<size_t>(k.k[2]);
    }
  };

  // Detected object info
  struct DetectedObj {
    geometry_msgs::Point centroid;    // In world frame
    std::string category;             // obstacle / basket / object
    std::string shape;                // cross / nought / "N/A"
    std::unordered_set<octomap::OcTreeKey, KeyHash> voxel_keys; // Occupied keys
  };

  // ===== Core Controls =====
  bool move_to_pose(const geometry_msgs::PointStamped& target, double z_offset, bool reset_orientation);
  bool move_gripper(float width);
  bool rotate_end_effector(double yaw);
  bool rotate_end_effector_roll_offset(double delta_roll);
  bool rotate_end_effector_pitch_offset(double delta_pitch);
  bool rotate_base_joint(double delta_angle_rad);
  bool rotate_joint(const std::string& joint_name, double delta_angle_rad);

  // ===== Cloud Processing =====
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterOutGreenPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterTopLayer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterByOctomapVoxels(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                                                const std::unordered_set<octomap::OcTreeKey, KeyHash>& keys,
                                                                const octomap::OcTree& tree);

  // ===== Grasping & Planning =====
  bool cartesian_grasp_and_place(const geometry_msgs::PointStamped& grasp_point,
                                 const geometry_msgs::PointStamped& place_point,
                                 double rotate_angle);

  geometry_msgs::PointStamped computeGraspPoint(const geometry_msgs::Point& centroid,
                                                const std::vector<geometry_msgs::PointStamped>& corners,
                                                const std::string& shape_type);
  geometry_msgs::PointStamped computeGraspPoint(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull,
                                                const std::vector<geometry_msgs::PointStamped>& corners,
                                                const std::string& shape_type);

  geometry_msgs::Point computeCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull);
  geometry_msgs::Vector3 computeOffsetVector(const geometry_msgs::PointStamped& grasp_point,
                                             const geometry_msgs::Point& centroid);
  geometry_msgs::PointStamped computeAdjustedPlacementPoint(const geometry_msgs::PointStamped& original_place_point,
                                                             const geometry_msgs::Vector3& offset);
  double compute_orientation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull,
                             const geometry_msgs::PointStamped& grasp_point);
  double compute_yaw_offset(const std::string& shape_type);

  // ===== Visualization =====
  void publishCloud(const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& frame_id);
  void publishBasketMarker(const geometry_msgs::PointStamped& basket_point, int id);
  void publishCentroidPointMarker(const geometry_msgs::Point& centroid, ros::Publisher& marker_pub, int id);
  void publishGraspPointMarker(const geometry_msgs::PointStamped& grasp_point, ros::Publisher& marker_pub, int id);
  void publishOrientationArrow(const geometry_msgs::PointStamped& origin, double yaw, ros::Publisher& marker_pub, int id);
  void publishCentroidCircleMarker(const geometry_msgs::Point& centroid, double radius, ros::Publisher& marker_pub, int id);
  void publishConvexHullMarker(const std::vector<geometry_msgs::PointStamped>& corners, ros::Publisher& marker_pub, int id);

  // ===== Utility / Tools =====
  bool savePointCloudToFile(const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filename);
  std::string classifyShapeByCentroidRegion(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                            const geometry_msgs::Point& centroid, double radius, int threshold);
  geometry_msgs::PointStamped make_point(double x, double y, double z);

  bool scan_sub_area(const std::vector<geometry_msgs::PointStamped>& corners);
  bool record_initial_joint_and_pose();
  bool save_initial_joint_and_pose();
  bool go_to_initial_state();

  std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<pcl::Vertices>>
  computeConvexHull(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);

  std::vector<geometry_msgs::PointStamped> extract_corner_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                                                 const std::vector<pcl::Vertices>& polygons,
                                                                 size_t top_k,
                                                                 const std::string& frame_id);

  bool run_fpfh_alignment(const std::string& object_path, const std::string& scene_path);
  bool extract_objects(const octomap::OcTree& tree, bool neighbor26, std::vector<DetectedObj>& out);
  void build_octomap_from_accumulated_clouds();
  void publishAccumulatedCloud();
  void clusterAccumulatedPointCloud();

  // ===== Template Filters =====
  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr filterPassThrough(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                                                          const std::string& field_name = "z",
                                                          float limit_min = 0.0,
                                                          float limit_max = 0.4);

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr filterVoxelGrid(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                                                        float leaf_size = 0.001f);

  template <typename PointT>
  void publishCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                    const std::string& frame_id);

  template <typename PointT>
  bool savePointCloudToFile(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                            const std::string& filename);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_, t2_service_, t3_service_;
  ros::Subscriber cloud_sub_;
  ros::Publisher filtered_cloud_pub_, marker_pub_, grasp_point_pub_, grasp_arrow_pub_, centroid_pub_;
  ros::Publisher octomap_pub_, accumulated_cloud_pub_, cluster_marker_pub_, basket_pub_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
  PointCloudRGB::Ptr latest_cloud_rgb;
  PointCloudXYZ::Ptr latest_cloud_xyz;
  PointCloudXYZ::Ptr model_cloud;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<octomap::OcTree> latest_octree_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  moveit::planning_interface::MoveGroupInterface arm_group_, hand_group_;

  bool cloud_received_;
  bool is_scanning_ = false;

  std::vector<double> initial_joint_values_;
  geometry_msgs::Pose initial_ee_pose_;
};

#include "cw2_class_impl.hpp"
#endif // CW2_CLASS_H_
