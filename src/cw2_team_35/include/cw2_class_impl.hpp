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

template <typename PointT>
bool cw2::savePointCloudToFile(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                               const std::string& filename)
{
  if (!cloud || cloud->empty()) {
    ROS_WARN("Trying to save an empty point cloud to %s", filename.c_str());
    return false;
  }

  if (pcl::io::savePCDFileBinary(filename, *cloud) == 0) {
    ROS_INFO("Saved point cloud to %s", filename.c_str());
    return true;
  } else {
    ROS_ERROR("Failed to save point cloud to %s", filename.c_str());
    return false;
  }
}

#endif // CW2_CLASS_IMPL_HPP_
