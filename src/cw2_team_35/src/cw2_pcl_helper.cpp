#include <cw2_class.h>

geometry_msgs::Vector3 cw2::computeOffsetVector(
    const geometry_msgs::PointStamped& grasp_point,
    const geometry_msgs::Point& centroid)
  {
    geometry_msgs::Vector3 offset;
    offset.x =  grasp_point.point.x - centroid.x;
    offset.y =  grasp_point.point.y - centroid.y;
    offset.z =  grasp_point.point.z - centroid.z;
    return offset;
  }
  
  geometry_msgs::PointStamped cw2::computeAdjustedPlacementPoint(
    const geometry_msgs::PointStamped& original_place_point,
    const geometry_msgs::Vector3& offset)
  {
    geometry_msgs::PointStamped adjusted;
    adjusted.header = original_place_point.header;
    adjusted.point.x = original_place_point.point.x + offset.x;
    adjusted.point.y = original_place_point.point.y + offset.y;
    adjusted.point.z = original_place_point.point.z + offset.z;
    return adjusted;
  }
  
  geometry_msgs::Point cw2::computeCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull)
  {
    geometry_msgs::Point centroid_point;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*hull, centroid);
  
    centroid_point.x = centroid[0];
    centroid_point.y = centroid[1];
    centroid_point.z = centroid[2];
  
    return centroid_point;
  }
  
  geometry_msgs::PointStamped cw2::computeGraspPoint(
    const geometry_msgs::Point& centroid,
    const std::vector<geometry_msgs::PointStamped>& corners,
    const std::string& shape_type)
  {
    geometry_msgs::PointStamped grasp_point;
    grasp_point.header.frame_id = "world";
    grasp_point.header.stamp = ros::Time::now();
  
    if (shape_type == "nought") {
      geometry_msgs::Point y_max_pt;
      float max_y = -std::numeric_limits<float>::max();
      for (const auto& pt : corners) {
        if (pt.point.y > max_y) {
          max_y = pt.point.y;
          y_max_pt = pt.point;
        }
      }
  
      Eigen::Vector2f vec(y_max_pt.x - centroid.x,
                          y_max_pt.y - centroid.y);
  
      float theta = M_PI / 4.0f;
      Eigen::Matrix2f R;
      R << std::cos(theta), -std::sin(theta),
           std::sin(theta),  std::cos(theta);
  
      Eigen::Vector2f rotated = R * vec * (std::sqrt(2.0f) / 2.0f);
  
      grasp_point.point.x = centroid.x + 0.95 * rotated[0];
      grasp_point.point.y = centroid.y + 0.95 * rotated[1];
      grasp_point.point.z = centroid.z + 0.015;
  
    } else if (shape_type == "cross") {
      geometry_msgs::Point y_max_pt;
      float max_y = -std::numeric_limits<float>::max();
      for (const auto& pt : corners) {
        if (pt.point.y > max_y) {
          max_y = pt.point.y;
          y_max_pt = pt.point;
        }
      }
  
      float alpha = 0.70f;  // 越接近1 → 越靠近 y_max_pt
      grasp_point.point.x = (1 - alpha) * centroid.x + alpha * y_max_pt.x;
      grasp_point.point.y = (1 - alpha) * centroid.y + alpha * y_max_pt.y;
      grasp_point.point.z = centroid.z + 0.015;
    }
  
    return grasp_point;
  }
  
  
  double cw2::compute_orientation(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull,
    const geometry_msgs::PointStamped& grasp_point)
  {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*hull, centroid);
  
    double dx = grasp_point.point.x - centroid[0];
    double dy = grasp_point.point.y - centroid[1];
  
    double yaw = std::atan2(dy, dx);  // 原始朝向
  
    // 限制在 ±166° （约 2.9 弧度）
    const double max_yaw_rad = 166.0 * M_PI / 180.0;
  
    if (std::abs(yaw) > max_yaw_rad) {
      yaw += (yaw > 0) ? -M_PI : M_PI;  // 旋转 180°，方向反过来
      // 限制在 [-π, π]，避免超出范围
      if (yaw > M_PI) yaw -= 2 * M_PI;
      if (yaw < -M_PI) yaw += 2 * M_PI;
    }
  
    return yaw;
  }
  
  
  double cw2::compute_yaw_offset(const std::string& shape_type)
  {
    if (shape_type == "nought") {
      return M_PI / 4.0;
    } else if (shape_type == "cross") {
      return -M_PI / 4.0;
    } else {
      return 0.0;  // 默认无偏移
    }
  }

  std::vector<geometry_msgs::PointStamped> cw2::extract_corner_points(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::vector<pcl::Vertices>& polygons,
    size_t top_k,
    const std::string& frame_id)  // 新增参数：frame
  {
    std::vector<geometry_msgs::PointStamped> corners;
    if (polygons.empty()) return corners;
  
    const std::vector<uint32_t>& indices = polygons[0].vertices;
    size_t n = indices.size();
    if (n < 3) return corners;
  
    std::vector<std::pair<float, geometry_msgs::PointStamped>> candidates;
  
    for (size_t i = 0; i < n; ++i)
    {
      const pcl::PointXYZRGB& A = cloud->points[indices[(i + n - 1) % n]];
      const pcl::PointXYZRGB& B = cloud->points[indices[i]];
      const pcl::PointXYZRGB& C = cloud->points[indices[(i + 1) % n]];
  
      float ABx = B.x - A.x;
      float ABy = B.y - A.y;
      float BCx = C.x - B.x;
      float BCy = C.y - B.y;
  
      float cross = ABx * BCy - ABy * BCx;
      float abs_cross = std::abs(cross);
  
      geometry_msgs::PointStamped pt;
      pt.header.frame_id = frame_id;
      pt.header.stamp = ros::Time::now();  // 可选：设置当前时间
      pt.point.x = B.x;
      pt.point.y = B.y;
      pt.point.z = B.z;
  
      candidates.emplace_back(abs_cross, pt);
  
    }
  
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) {
                return a.first > b.first;
              });
  
    for (size_t i = 0; i < std::min(top_k, candidates.size()); ++i) {
      corners.push_back(candidates[i].second);
    }
  
    return corners;
  }
  
  std::pair<
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
    std::vector<pcl::Vertices>
  >
  cw2::computeConvexHull(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
  {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  std::vector<pcl::Vertices> polygons;
  
  chull.setInputCloud(input_cloud);
  chull.setDimension(2);  // 通常我们假设物体在同一个平面上
  chull.reconstruct(*hull, polygons);
  
  return {hull, polygons};
  }

  std::string cw2::classifyShapeByCentroidRegion(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const geometry_msgs::Point& centroid,
    double radius,
    int threshold)
  {
  int count = 0;
  double r2 = radius * radius;
  
  for (const auto& pt : cloud->points) {
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
  
    double dx = pt.x - centroid.x;
    double dy = pt.y - centroid.y;
    double dz = pt.z - centroid.z;
  
    // 可以只考虑XY平面，也可以考虑3D球形区域
    double dist2 = dx * dx + dy * dy;  // 或 dx*dx + dy*dy + dz*dz;
  
    if (dist2 <= r2) {
      count++;
    }
  }
  
  if (count < threshold) {
    return "nought";
  } else {
    return "cross";
  }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::filterTopLayer(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud)
  {
    float z_max = -std::numeric_limits<float>::max();
  
    // 第一步：找最高点
    for (const auto& pt : input_cloud->points) {
        if (!std::isnan(pt.z)) {
            if (pt.z > z_max) {
                z_max = pt.z;
            }
        }
    }
  
    float z_min = z_max - 0.035f;
  
    // 第二步：保留 z 在 [z_min, z_max] 之间的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    output_cloud->header = input_cloud->header;
    output_cloud->is_dense = false;
    output_cloud->height = 1;
  
    for (const auto& pt : input_cloud->points) {
        if (!std::isnan(pt.z) && pt.z >= z_min && pt.z <= z_max) {
            output_cloud->points.push_back(pt);
        }
    }
  
    output_cloud->width = static_cast<uint32_t>(output_cloud->points.size());
  
    return output_cloud;
  }

  void cw2::publishCentroidPointMarker(
    const geometry_msgs::Point& centroid,
    ros::Publisher& marker_pub,
    int id
  )
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // 默认假设是 world frame
    marker.header.stamp = ros::Time::now();
    marker.ns = "centroid_point";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
  
    marker.pose.position = centroid;
    marker.pose.orientation.w = 1.0;
  
    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;
  
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;  // 黄色质心
    marker.color.a = 1.0;
  
    marker.lifetime = ros::Duration(0.0);  // 永久显示
  
    marker_pub.publish(marker);
  }
  
  
  void cw2::publishBasketMarker(
    const geometry_msgs::PointStamped& basket_point,
    int id
  ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // 如果你的basket_point是相对于world的，就写world
    marker.header.stamp = ros::Time::now();
    marker.ns = "basket_marker";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
  
    marker.pose.position = basket_point.point;
    marker.pose.orientation.w = 1.0;
  
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
  
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
  
    marker.lifetime = ros::Duration(0.0);
  
    basket_pub_.publish(marker);
  }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::filterByOctomapVoxels(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    const std::unordered_set<octomap::OcTreeKey, cw2::KeyHash>& keys,
    const octomap::OcTree& tree)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    output->header = input_cloud->header;
    output->is_dense = input_cloud->is_dense;
    output->height = 1;
  
    for (const auto& pt : input_cloud->points) {
      if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
  
      octomap::OcTreeKey key;
      if (tree.coordToKeyChecked(pt.x, pt.y, pt.z, key)) {
        if (keys.count(key)) {
          output->points.push_back(pt);
        }
      }
    }
  
    output->width = static_cast<uint32_t>(output->points.size());
    return output;
  }
  