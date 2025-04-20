#include "cw2_class.h"

/**
 * @brief Computes the vector offset from the centroid to the grasp point.
 *
 * @param grasp_point The stamped grasp point.
 * @param centroid The centroid point in the same frame.
 * @return geometry_msgs::Vector3 The offset vector (grasp_point – centroid).
 */
geometry_msgs::Vector3 cw2::computeOffsetVector(
    const geometry_msgs::PointStamped& grasp_point,
    const geometry_msgs::Point& centroid)
{
  geometry_msgs::Vector3 offset;
  // Compute component-wise difference
  offset.x = grasp_point.point.x - centroid.x;
  offset.y = grasp_point.point.y - centroid.y;
  offset.z = grasp_point.point.z - centroid.z;
  return offset;
}

/**
 * @brief Adjusts a placement point by applying an offset vector.
 *
 * @param original_place_point The original stamped placement point.
 * @param offset The offset vector to apply.
 * @return geometry_msgs::PointStamped The new stamped placement point.
 */
geometry_msgs::PointStamped cw2::computeAdjustedPlacementPoint(
    const geometry_msgs::PointStamped& original_place_point,
    const geometry_msgs::Vector3& offset)
{
  geometry_msgs::PointStamped adjusted;
  // Preserve header
  adjusted.header = original_place_point.header;
  // Apply offset
  adjusted.point.x = original_place_point.point.x + offset.x;
  adjusted.point.y = original_place_point.point.y + offset.y;
  adjusted.point.z = original_place_point.point.z + offset.z;
  return adjusted;
}

/**
 * @brief Computes the centroid of a PCL point cloud.
 *
 * @param hull Pointer to the input point cloud.
 * @return geometry_msgs::Point The computed centroid coordinates.
 */
geometry_msgs::Point cw2::computeCentroid(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull)
{
  // Use PCL's compute3DCentroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*hull, centroid);

  geometry_msgs::Point pt;
  pt.x = centroid[0];
  pt.y = centroid[1];
  pt.z = centroid[2];
  return pt;
}

/**
 * @brief Computes a grasp point based on shape and hull corners.
 *
 * @param centroid The object centroid.
 * @param corners Hull corner points.
 * @param shape_type "nought" or "cross".
 * @return geometry_msgs::PointStamped The stamped grasp point.
 */
geometry_msgs::PointStamped cw2::computeGraspPoint(
    const geometry_msgs::Point& centroid,
    const std::vector<geometry_msgs::PointStamped>& corners,
    const std::string& shape_type)
{
  geometry_msgs::PointStamped grasp_point;
  // Use world frame and current time
  grasp_point.header.frame_id = "world";
  grasp_point.header.stamp    = ros::Time::now();

  if (shape_type == "nought") {
    // Find corner with maximum Y
    geometry_msgs::Point max_pt; float max_y = -std::numeric_limits<float>::max();
    for (const auto& p : corners) {
      if (p.point.y > max_y) {
        max_y = p.point.y;
        max_pt = p.point;
      }
    }
    // Compute direction vector in XY plane
    Eigen::Vector2f vec(max_pt.x - centroid.x, max_pt.y - centroid.y);
    // Rotate by 45 degrees
    float theta = M_PI / 4.0f;
    Eigen::Matrix2f R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);
    Eigen::Vector2f rotated = R * vec * (std::sqrt(2.0f) / 2.0f);

    // Scale and offset
    grasp_point.point.x = centroid.x + 0.95f * rotated[0];
    grasp_point.point.y = centroid.y + 0.95f * rotated[1];
    grasp_point.point.z = centroid.z + 0.015;
  }
  else if (shape_type == "cross") {
    // Find corner with maximum Y
    geometry_msgs::Point max_pt; float max_y = -std::numeric_limits<float>::max();
    for (const auto& p : corners) {
      if (p.point.y > max_y) {
        max_y = p.point.y;
        max_pt = p.point;
      }
    }
    // Interpolate between centroid and max_pt
    float alpha = 0.70f;
    grasp_point.point.x = (1 - alpha) * centroid.x + alpha * max_pt.x;
    grasp_point.point.y = (1 - alpha) * centroid.y + alpha * max_pt.y;
    grasp_point.point.z = centroid.z + 0.015;
  }

  return grasp_point;
}

/**
 * @brief Computes the yaw orientation from centroid to grasp point,
 *        with limits to avoid near-vertical swings.
 *
 * @param hull Pointer to the convex hull cloud.
 * @param grasp_point The stamped grasp point.
 * @return double The computed yaw in radians.
 */
double cw2::compute_orientation(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull,
    const geometry_msgs::PointStamped& grasp_point)
{
  // Compute true centroid again
  Eigen::Vector4f cent;
  pcl::compute3DCentroid(*hull, cent);
  double dx = grasp_point.point.x - cent[0];
  double dy = grasp_point.point.y - cent[1];
  double yaw = std::atan2(dy, dx);

  // Limit to ±166°
  const double max_yaw = 166.0 * M_PI / 180.0;
  if (std::abs(yaw) > max_yaw) {
    yaw += (yaw > 0 ? -M_PI : M_PI);
    // Wrap into [-π, π]
    if (yaw > M_PI)  yaw -= 2 * M_PI;
    if (yaw < -M_PI) yaw += 2 * M_PI;
  }
  return yaw;
}

/**
 * @brief Provides a yaw offset based on shape type.
 *
 * @param shape_type "nought" or "cross".
 * @return double Yaw offset in radians.
 */
double cw2::compute_yaw_offset(const std::string& shape_type)
{
  if (shape_type == "nought") {
    return M_PI / 4.0;
  } else if (shape_type == "cross") {
    return -M_PI / 4.0;
  } else {
    return 0.0;
  }
}

/**
 * @brief Extracts top-K corner points from a convex hull polygon.
 *
 * @param cloud The hull point cloud.
 * @param polygons Polygon indices from convex hull.
 * @param top_k Number of corners to select.
 * @param frame_id Frame to stamp output points.
 * @return std::vector<geometry_msgs::PointStamped> Selected corner points.
 */
std::vector<geometry_msgs::PointStamped> cw2::extract_corner_points(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::vector<pcl::Vertices>& polygons,
    size_t top_k,
    const std::string& frame_id)
{
  std::vector<geometry_msgs::PointStamped> corners;
  if (polygons.empty()) return corners;

  const auto& inds = polygons[0].vertices;
  size_t n = inds.size();
  if (n < 3) return corners;

  // Collect cross-product magnitudes
  std::vector<std::pair<float, geometry_msgs::PointStamped>> cand;
  for (size_t i = 0; i < n; ++i) {
    const auto& A = cloud->points[inds[(i+n-1)%n]];
    const auto& B = cloud->points[inds[i]];
    const auto& C = cloud->points[inds[(i+1)%n]];
    // Cross product in 2D
    float cross = (B.x - A.x)*(C.y - B.y) - (B.y - A.y)*(C.x - B.x);
    float mag = std::abs(cross);

    geometry_msgs::PointStamped pt;
    pt.header.frame_id = frame_id;
    pt.header.stamp    = ros::Time::now();
    pt.point.x = B.x; pt.point.y = B.y; pt.point.z = B.z;
    cand.emplace_back(mag, pt);
  }

  // Sort descending by magnitude
  std::sort(cand.begin(), cand.end(),
            [](auto& a, auto& b){ return a.first > b.first; });

  // Take top_k
  for (size_t i = 0; i < std::min(top_k, cand.size()); ++i) {
    corners.push_back(cand[i].second);
  }
  return corners;
}

/**
 * @brief Computes the 2D convex hull of an input cloud.
 *
 * @param input_cloud The input point cloud.
 * @return A pair of hull cloud pointer and polygon indices.
 */
std::pair<
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
  std::vector<pcl::Vertices>
> cw2::computeConvexHull(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<pcl::Vertices> polys;
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud(input_cloud);
  chull.setDimension(2);  // planar hull
  chull.reconstruct(*hull, polys);
  return {hull, polys};
}

/**
 * @brief Classifies shape by counting points within a radius around the centroid.
 *
 * @param cloud Input point cloud.
 * @param centroid Center point.
 * @param radius Radius in meters.
 * @param threshold Minimum count for "cross".
 * @return std::string "nought" or "cross".
 */
std::string cw2::classifyShapeByCentroidRegion(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const geometry_msgs::Point& centroid,
    double radius,
    int threshold)
{
  int count = 0;
  double r2 = radius * radius;
  for (const auto& pt : cloud->points) {
    if (std::isnan(pt.x) || std::isnan(pt.y)) continue;
    double dx = pt.x - centroid.x;
    double dy = pt.y - centroid.y;
    if ((dx*dx + dy*dy) <= r2) {
      ++count;
    }
  }
  return (count < threshold ? "nought" : "cross");
}

/**
 * @brief Filters to the top layer of points within ~3.5 cm of max Z.
 *
 * @param input_cloud The input point cloud.
 * @return Filtered point cloud pointer.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::filterTopLayer(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
  // Find maximum Z
  float z_max = -std::numeric_limits<float>::max();
  for (auto& pt : input_cloud->points) {
    if (!std::isnan(pt.z) && pt.z > z_max) {
      z_max = pt.z;
    }
  }
  float z_min = z_max - 0.035f;

  // Retain points within [z_min, z_max]
  auto output = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  output->header = input_cloud->header;
  for (auto& pt : input_cloud->points) {
    if (!std::isnan(pt.z) && pt.z >= z_min && pt.z <= z_max) {
      output->points.push_back(pt);
    }
  }
  output->width = output->points.size();
  output->height = 1;
  output->is_dense = false;
  return output;
}

/**
 * @brief Publish a centroid as a small sphere marker.
 *
 * @param centroid The point to display.
 * @param marker_pub ROS publisher.
 * @param id Marker ID.
 */
void cw2::publishCentroidPointMarker(
    const geometry_msgs::Point& centroid,
    ros::Publisher& marker_pub,
    int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns   = "centroid_point";
  marker.id   = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position = centroid;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.025;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;

  marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.0);

  marker_pub.publish(marker);
}

/**
 * @brief Publish a basket position as a larger sphere marker.
 *
 * @param basket_point The stamped basket center.
 * @param id Marker ID.
 */
void cw2::publishBasketMarker(
    const geometry_msgs::PointStamped& basket_point,
    int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = basket_point.header.frame_id;
  marker.header.stamp    = ros::Time::now();
  marker.ns   = "basket_marker";
  marker.id   = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position = basket_point.point;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.0);

  basket_pub_.publish(marker);
}

/**
 * @brief Filters input point cloud by retaining only points whose octomap key is in the given set.
 *
 * @param input_cloud The input PCL cloud.
 * @param keys Set of occupied octomap keys.
 * @param tree The octomap tree for coordinate-key conversion.
 * @return Filtered point cloud pointer.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::filterByOctomapVoxels(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    const std::unordered_set<octomap::OcTreeKey, KeyHash>& keys,
    const octomap::OcTree& tree)
{
  auto output = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  output->header = input_cloud->header;
  for (auto& pt : input_cloud->points) {
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
    octomap::OcTreeKey key;
    if (tree.coordToKeyChecked(pt.x, pt.y, pt.z, key) && keys.count(key)) {
      output->points.push_back(pt);
    }
  }
  output->width = output->points.size();
  output->height = 1;
  output->is_dense = input_cloud->is_dense;
  return output;
}
