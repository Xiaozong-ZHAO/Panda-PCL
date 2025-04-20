#ifndef CW2_CLASS_H_
#define CW2_CLASS_H_
#include <algorithm>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <unordered_set>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <octomap_msgs/conversions.h>        // fullMapToMsg

// PCL / TF
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/fpfh_omp.h>  // ✅ FPFHEstimationOMP
#include <pcl/registration/sample_consensus_prerejective.h> // ✅ SampleConsensusPrerejective
#include <pcl/visualization/pcl_visualizer.h> // ✅ PCLVisualizer + ColorHandler
#include <pcl/common/time.h>  // ✅ ScopeTime
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/MarkerArray.h>


#include <thread>
#include <chrono>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

// 特征估计 & 识别
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>

// Service definitions
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

// PCL 类型简化别名
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<DescriptorType> DescriptorCloud;
typedef pcl::Normal NormalType;

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

  struct KeyHash {
  size_t operator()(const octomap::OcTreeKey& k) const {
    return (static_cast<size_t>(k.k[0]) << 42) ^
            (static_cast<size_t>(k.k[1]) << 21) ^
            static_cast<size_t>(k.k[2]);
  }
};
  struct DetectedObj
  {
    geometry_msgs::Point centroid;   // 世界系
    std::string category;            // obstacle / basket / object
    std::string shape;               // cross / nought / "N/A"
    
    // ✅ 新增字段，用于存储物体的体素 key 集合
    std::unordered_set<octomap::OcTreeKey, KeyHash> voxel_keys;
  };

  moveit::planning_interface::PlanningSceneInterface psi_;

  // 控制函数
  bool move_to_pose(const geometry_msgs::PointStamped& target, double z_offset, bool reset_orientation);
  bool move_gripper(float width);
  bool rotate_end_effector(double yaw);

  // 点云回调
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  // ============== 模板版过滤函数 ==============
  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr
  filterPassThrough(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                    const std::string& field_name = "z",
                    float limit_min = 0.0, 
                    float limit_max = 0.4);

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr
  filterVoxelGrid(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                  float leaf_size = 0.001f);

  // ============== 模板版发布函数 ==============
  template <typename PointT>
  void publishCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                    const std::string& frame_id);
  void publishBasketMarker(
    const geometry_msgs::PointStamped& basket_point,
    int id
  );

  // ============== 模板版位姿估计函数(示例) ==============
  template <typename PointT>
  bool estimatePoseByMatchingGeneric(
      const typename pcl::PointCloud<PointT>::Ptr& scene_cloud,
      const std::string& model_path,
      Eigen::Matrix4f& transform_out);
  // 生成凸包

  void publishConvexHullMarker(
    const std::vector<geometry_msgs::PointStamped>& corners,
    ros::Publisher& marker_pub,
    int id /*= 0*/);

    geometry_msgs::PointStamped computeGraspPoint(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull,
      const std::vector<geometry_msgs::PointStamped>& corners,
      const std::string& shape_type);
    void publishGraspPointMarker(
      const geometry_msgs::PointStamped& grasp_point,
      ros::Publisher& marker_pub,
      int id);

  std::pair<
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
  std::vector<pcl::Vertices>
  >
  computeConvexHull(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);

  std::vector<geometry_msgs::PointStamped> extract_corner_points(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::vector<pcl::Vertices>& polygons,
    size_t top_k,
    const std::string& frame_id);

  double compute_orientation(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull,
    const geometry_msgs::PointStamped& grasp_point);

  
  double compute_yaw_offset(const std::string& shape_type);

  void publishOrientationArrow(
    const geometry_msgs::PointStamped& origin,
    double yaw,
    ros::Publisher& marker_pub,
    int id);

  bool cartesian_grasp_and_place(
    const geometry_msgs::PointStamped& grasp_point,
    const geometry_msgs::PointStamped& place_point,
    double rotate_angle);

  geometry_msgs::PointStamped computeGraspPoint(
    const geometry_msgs::Point& centroid,
    const std::vector<geometry_msgs::PointStamped>& corners,
    const std::string& shape_type);

  geometry_msgs::Point computeCentroid(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull);

  geometry_msgs::Vector3 computeOffsetVector(
    const geometry_msgs::PointStamped& grasp_point,
    const geometry_msgs::Point& centroid);

  geometry_msgs::PointStamped computeAdjustedPlacementPoint(
    const geometry_msgs::PointStamped& original_place_point,
    const geometry_msgs::Vector3& offset);

  bool run_fpfh_alignment(
    const std::string& object_path, 
    const std::string& scene_path);

  bool rotate_end_effector_roll_offset(double delta_roll);
  bool rotate_end_effector_pitch_offset(double delta_pitch);


  template <typename PointT>
  bool savePointCloudToFile(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                            const std::string& filename);
                            
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterOutGreenPoints(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterTopLayer(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);
  
  void publishCentroidCircleMarker(
    const geometry_msgs::Point& centroid,
    double radius,
    ros::Publisher& marker_pub,
    int id
    );
  std::string classifyShapeByCentroidRegion(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const geometry_msgs::Point& centroid,
    double radius,
    int threshold);
  bool scan_sub_area(const std::vector<geometry_msgs::PointStamped>& corners);
  bool record_initial_joint_and_pose();
  bool save_initial_joint_and_pose();
  bool go_to_initial_state();
  bool rotate_base_joint(double delta_angle_rad);
  geometry_msgs::PointStamped make_point(double x, double y, double z);
  void build_octomap_from_accumulated_clouds();
  bool rotate_joint(const std::string& joint_name, double delta_angle_rad);
  bool extract_objects(const octomap::OcTree& tree,
    bool neighbor26,
    std::vector<DetectedObj>& out);   // <── 新增一个输出参数
  
  void publishAccumulatedCloud();
  void clusterAccumulatedPointCloud();

private:
  ros::NodeHandle nh_;
  // 服务
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  // 点云相关
  ros::Subscriber cloud_sub_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher grasp_point_pub_;
  ros::Publisher grasp_arrow_pub_;
  ros::Publisher centroid_pub_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
  ros::Publisher octomap_pub_;
  ros::Publisher accumulated_cloud_pub_;
  ros::Publisher cluster_marker_pub_;
  ros::Publisher basket_pub_;

  std::vector<double> initial_joint_values_;
  geometry_msgs::Pose initial_ee_pose_;

  // 分别保存 RGB 和 XYZ 的最新点云
  PointCloudRGB::Ptr latest_cloud_rgb;
  PointCloudXYZ::Ptr latest_cloud_xyz;
  PointCloudXYZ::Ptr model_cloud;

  bool cloud_received_;
  bool is_scanning_ = false;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<octomap::OcTree> latest_octree_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterByOctomapVoxels(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    const std::unordered_set<octomap::OcTreeKey, KeyHash>& keys,
    const octomap::OcTree& tree);


  // MoveIt groups
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface hand_group_;
};

#include "cw2_class_impl.hpp" // 建议把模板实现放到 .hpp 或 .ipp 里

#endif // CW2_CLASS_H_
