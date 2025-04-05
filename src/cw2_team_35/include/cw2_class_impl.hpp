#ifndef CW2_CLASS_IMPL_HPP_
#define CW2_CLASS_IMPL_HPP_

#include "cw2_class.h"
#include <pcl/io/pcd_io.h>

// ============== 模板版 PassThrough 过滤 ==============
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
cw2::filterPassThrough(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                       const std::string& field_name,
                       float limit_min, 
                       float limit_max)
{
  typename pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName(field_name);
  pass.setFilterLimits(limit_min, limit_max);
  pass.filter(*cloud_out);
  return cloud_out;
}

// ============== 模板版 VoxelGrid 过滤 ==============
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
cw2::filterVoxelGrid(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                     float leaf_size)
{
  typename pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(cloud_in);
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.filter(*cloud_out);
  return cloud_out;
}

// ============== 模板版发布点云 ==============
template <typename PointT>
void cw2::publishCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                       const std::string& frame_id)
{
  sensor_msgs::PointCloud2 ros_cloud_msg;
  pcl::toROSMsg(*cloud, ros_cloud_msg);
  ros_cloud_msg.header.frame_id = frame_id;
  ros_cloud_msg.header.stamp = ros::Time::now();
  filtered_cloud_pub_.publish(ros_cloud_msg);
}

// ============== 模板版位姿估计示例 ==============
template <typename PointT>
bool cw2::estimatePoseByMatchingGeneric(
    const typename pcl::PointCloud<PointT>::Ptr& scene_cloud,
    const std::string& model_path,
    Eigen::Matrix4f& transform_out)
{
  // 1. 加载模型点云
  typename pcl::PointCloud<PointT>::Ptr model_cloud(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT>(model_path, *model_cloud) < 0)
  {
    ROS_ERROR("❌ Failed to load model point cloud from %s", model_path.c_str());
    return false;
  }

  // 2. 估计法线
  pcl::NormalEstimationOMP<PointT, NormalType> ne;
  ne.setKSearch(10);

  auto model_normals = boost::make_shared<pcl::PointCloud<NormalType>>();
  auto scene_normals = boost::make_shared<pcl::PointCloud<NormalType>>();

  ne.setInputCloud(model_cloud);
  ne.compute(*model_normals);

  ne.setInputCloud(scene_cloud);
  ne.compute(*scene_normals);

  // 3. 提取关键点（UniformSampling）
  pcl::UniformSampling<PointT> us;
  us.setRadiusSearch(0.01f);

  typename pcl::PointCloud<PointT>::Ptr model_keypoints(new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr scene_keypoints(new pcl::PointCloud<PointT>);

  us.setInputCloud(model_cloud);
  us.filter(*model_keypoints);

  us.setInputCloud(scene_cloud);
  us.filter(*scene_keypoints);

  // 4. 计算 SHOT 描述子
  pcl::SHOTEstimationOMP<PointT, NormalType, DescriptorType> shot;
  shot.setRadiusSearch(0.02f);

  DescriptorCloud::Ptr model_desc(new DescriptorCloud);
  DescriptorCloud::Ptr scene_desc(new DescriptorCloud);

  shot.setInputNormals(model_normals);
  shot.setSearchSurface(model_cloud);
  shot.setInputCloud(model_keypoints);
  shot.compute(*model_desc);

  shot.setInputNormals(scene_normals);
  shot.setSearchSurface(scene_cloud);
  shot.setInputCloud(scene_keypoints);
  shot.compute(*scene_desc);

  // 5. 建立对应关系 (KNN)
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud(model_desc);

  for (std::size_t i = 0; i < scene_desc->size(); ++i)
  {
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);

    if (!std::isfinite(scene_desc->at(i).descriptor[0]))
      continue;

    if (match_search.nearestKSearch(scene_desc->at(i), 1,
                                    neigh_indices, neigh_sqr_dists) > 0 &&
        neigh_sqr_dists[0] < 0.25f)
    {
      correspondences->emplace_back(neigh_indices[0],
                                    static_cast<int>(i),
                                    neigh_sqr_dists[0]);
    }
  }

  // 6. Hough3D 分组
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame>);
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame>);

  pcl::BOARDLocalReferenceFrameEstimation<PointT, NormalType, pcl::ReferenceFrame> rf_est;
  rf_est.setRadiusSearch(0.015f);

  rf_est.setInputCloud(model_keypoints);
  rf_est.setInputNormals(model_normals);
  rf_est.setSearchSurface(model_cloud);
  rf_est.compute(*model_rf);

  rf_est.setInputCloud(scene_keypoints);
  rf_est.setInputNormals(scene_normals);
  rf_est.setSearchSurface(scene_cloud);
  rf_est.compute(*scene_rf);

  pcl::Hough3DGrouping<PointT, PointT, pcl::ReferenceFrame, pcl::ReferenceFrame> hough;
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

  // 拿到第一个解即可
  transform_out = transforms[0];
  return true;
}

#endif // CW2_CLASS_IMPL_HPP_
