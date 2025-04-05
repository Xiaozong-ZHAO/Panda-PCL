/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW3_CLASS_H_
#define CW3_CLASS_H_

// system includes
#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string> // std::string
#include <vector>

// ROS includes
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/segmentation/region_growing_rgb.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Moveit specific includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// include services from the spawner package - we will be responding to these
#include "cw3_world_spawner/Task1Service.h"
#include "cw3_world_spawner/Task2Service.h"
#include "cw3_world_spawner/Task3Service.h"

// type definition
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

// // include any services created in this package
// #include "cw3_team_x/example.h"

class cw3
{
public:

  /* ----- class member functions ----- */

  /// @brief Remove all collision from motion planning.
  void
  remove_all_collisions ();

  /// @brief Moveit function: move arm to a given pose
  /// @param target_pose (geometry_msgs::Pose) Target pose to reach
  /// @return true if reach that pose.
  bool 
  moveArm(geometry_msgs::Pose target_pose);

  /// @brief Preprocessing of raw input Point Cloud data.
  /// @param cloud_input_msg (sensor_msgs::PointCloud2ConstPtr) pointer pointing to the input point cloud.
  void
  cloudCallBack (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);

  /// @brief Apply Voxel Grid filtering.
  /// @param in_cloud_ptr (PointCPtr) the input PointCloud2 pointer
  /// @param out_cloud_ptr (PointCPtr) the output PointCloud2 pointer
  void
  applyVX (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

  /// @brief Normal estimation.
  /// @param in_cloud_ptr (PointCPtr) the input PointCloud2 pointer.
  void
  findNormals (PointCPtr &in_cloud_ptr);

  /// @brief Segment Plane from point cloud.
  /// @param in_cloud_ptr (PointCPtr) the input PointCloud2 pointer.
  void
  segPlane (PointCPtr &in_cloud_ptr);

  /// @brief Function to publish the filtered point cloud in order to visualise in rviz.
  /// @param pc_pub (ros::Publisher) publisher to advertise filtered point cloud.
  /// @param pc (PointC) filtered point cloud.
  void
  pubFilteredPCMsg (ros::Publisher &pc_pub, PointC &pc);

  /// @brief Function to calculate the orientation of the object.
  /// @param cloud_input the input point cloud.
  tf2::Quaternion 
  getOrientation(PointCPtr &cloud_input);

  /// @brief Object projection.
  /// @param in_cloud_ptr (PointCPtr) the input PointCloud2 pointer.
  void
  projection (PointCPtr &in_cloud_ptr);

  /// @brief Find the Pose of Cylinder.
  /// @param in_cloud_ptr (PointCPtr) the input PointCloud2 pointer.
  geometry_msgs::Point
  findCylPose (PointCPtr &in_cloud_ptr);

  /// @brief transform the point from link_8 frame to camera frame
  /// @param in_point point in link_8 frame.
  geometry_msgs::Point
  link2Cam (geometry_msgs::Point in_point);

  /// @brief transform the point from camera frame to link_8 frame
  /// @param in_point point in camera frame.
  geometry_msgs::Point
  cam2Link (geometry_msgs::Point in_point);

  /// @brief apply pass through to the point cloud on x direction
  /// @param in_cloud_ptr ptr of the input PT.
  void 
  applyPT_x (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr, double &threshold);

  /// @brief apply pass through to the point cloud on y direction
  /// @param in_cloud_ptr ptr of the input PT.
  void 
  applyPT_y (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr, double &threshold);

  /// @brief Moveit function: add an object to the motion planning and collision avoidance.
  /// @param object_name (std::string) name of the object
  /// @param centre (geometry_msgs::Point) centre point of the object
  /// @param dimensions (geometry_msgs::Vector3) xyz dimensions of the object
  /// @param orientation (geometry_msgs::Quaternion) orientation of the object
  void
  addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation);

  /// @brief add the plane on the RViz
  void add_plane();

  /// @brief Function to detect the object type (nought or cross).
  /// @param cloud_input the input point cloud.
  std::string 
  getObjectType(PointCPtr &cloud_input);

  /// @brief Clustering based on the Euclidean distance.
  /// @param in_cloud_ptr (PointCPtr) the input PointCloud2 pointer.
  void
  Cluster (PointCPtr &in_cloud_ptr);

  /// @brief Moveit function: oppen/close gripper to certain width.
  /// @param width (float) the width between two fingers of gripper.
  /// @return true is reach certain width.
  bool 
  moveGripper(float width);

  /// @brief Movelt function to add object into motion planning for collision avoidence. 
  /// @param centre geometry_msgs::Pose centre position of the object.
  /// @param dimensions double dimensions (in meter) of the object.
  /// @param name string name of the object.
  void
  addCollision(std::string name ,geometry_msgs::Pose centre, double dimensions);

  /// @brief Movelt function to remove object in motion planning.
  /// @param object_name std::string name of object to be removed.
  void
  removeCollisionObject(std::string object_name);










  /* ----- class member variables ----- */

  /// @brief MoveIt interface to interact with the moveit planning scene (eg collision objects). 
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /// @brief MoveIt interface to move groups to seperate the arm and the gripper these are defined in urdf.
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};

  /// @brief The input point cloud frame id.
  std::string g_input_pc_frame_id_;

  /// @brief Point Cloud (input).
  pcl::PCLPointCloud2 g_pcl_pc;

  /// @brief Point Cloud (input) pointer.
  PointCPtr g_cloud_ptr;

  /// @brief Pointer Cloud (Pass Through filtered & voxel grid) pointer
  PointCPtr g_cloud_filtered_pt, g_cloud_filtered_vx;

  /// @brief Voxel Grid filter.
  pcl::VoxelGrid<PointT> g_vx;

  /// @brief Voxel Grid filter's leaf size.
  double g_vg_leaf_sz;

  /// @brief The trigger for task 1 to segment the plane
  bool task_1_trigger = false;

  /// @brief The trigger for task 2 to segment the plane
  bool task_2_trigger = false;

  /// @brief The trigger for task 3 to segment the plane
  bool task_3_trigger = false;

  /// @brief The flag indicate if the recognization in task 2 is finished
  bool recog_task_2 = false;

  /// @brief The flag indicate if the clustering in task 3 is finished
  bool cluster_task_3 = false;

  // /// @brief The flag indicate if the movement is successful
  // bool success = false;

  /// @brief Normal estimation.
  pcl::NormalEstimation<PointT, pcl::Normal> g_ne;

  /// @brief KDTree for nearest neighborhood search
  pcl::search::KdTree<PointT>::Ptr g_tree_ptr;

  /// @brief Nearest neighborhooh size for normal estimation.
  double g_k_nn;

  /// @brief Cloud of normals.
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals_filtered_plane;

  /// @brief Point cloud to hold plane points.
  PointCPtr g_cloud_plane;

  /// @brief Point Cloud to hold the rest after removing plane.
  PointCPtr g_cloud_filtered_plane;

  /// @brief SAC segmentation.
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg;

  /// @brief Point indices for plane.
  pcl::PointIndices::Ptr g_inliers_plane;

  /// @brief Model coefficients for the plane segmentation.
  pcl::ModelCoefficients::Ptr g_coeff_plane;

  /// @brief ROS publishers.
  ros::Publisher g_pub_rm_plane;

  /// @brief ROS publishers.
  ros::Publisher g_pub_rm_plane_2;

  /// @brief Point Cloud (filtered) sensros_msg for publ.
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;

  /// @brief Extract point cloud indices.
  pcl::ExtractIndices<PointT> g_extract_pc;

  /// @brief Extract point cloud normal indices.
  pcl::ExtractIndices<pcl::Normal> g_extract_normals;

  /// @brief Flag showing if the segmentation of plane finished.
  bool segment_done = false;

  /// @brief Model coefficients for the plane projection.
  pcl::ModelCoefficients::Ptr g_coeff_plane_project;

  /// @brief Point Cloud of plane projected.
  PointCPtr g_cloud_projected_plane;

  /// @brief Projection.
  pcl::ProjectInliers<PointT> proj;

  /// @brief ROS geometry message point.
  geometry_msgs::PointStamped g_cyl_pt_msg;

  /// @brief TF listener definition.
  tf::TransformListener g_listener_;

  /// @brief the pose of detection
  geometry_msgs::Pose detect_pose;

  /// @brief Transformation from camera frame to link 8 frame.
  geometry_msgs::TransformStamped TranCamera2Link8, TranLink82Cam;

  /// @brief An buffer to store 10s of the r eceiving tf2 transformations.
  tf2_ros::Buffer tfBuffer;

  /// @brief A TransformListener object which reveiving tf2 transformations.
  tf2_ros::TransformListener tfListener{tfBuffer};
  
  /// @brief variable to store the object type for task_1
  std::string type_task_1;

  /// @brief quaternion of the object in task_1
  tf2::Quaternion q_object_task_1;

  /// @brief Pass Through filter.
  pcl::PassThrough<PointT> g_pt;

  /// @brief frame id of the base frame (i.e., world frame)
  const std::string base_frame_ = "panda_link0";

  /// @brief variable to store the object type for task_2
  std::string task_2_objectType;
  
  // /// @brief define the cluster indices for the task 3
  // std::vector<pcl::PointIndices> cluster_indices_task_3;

  /// @brief define the vector stores the poses of clusters
  std::vector<geometry_msgs::Pose> cluster_poses;

  /// @brief the rotation angle in task 1
  double degree, rad;

  /// @brief parameters for the gripper
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;

  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
    * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  









  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;


  // constructor
  cw3(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw3_world_spawner::Task1Service::Request &request,
    cw3_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw3_world_spawner::Task2Service::Request &request,
    cw3_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw3_world_spawner::Task3Service::Request &request,
    cw3_world_spawner::Task3Service::Response &response);
};

#endif // end of include guard for cw3_CLASS_H_
