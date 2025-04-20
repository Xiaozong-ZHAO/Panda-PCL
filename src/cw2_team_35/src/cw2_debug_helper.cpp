#include "cw2_class.h"

/**
 * @brief Publish a spherical marker at the grasp point.
 * 
 * @param grasp_point The stamped point where the sphere is placed.
 * @param marker_pub ROS publisher for visualization_msgs::Marker.
 * @param id Unique identifier for this marker.
 */
void cw2::publishGraspPointMarker(
    const geometry_msgs::PointStamped& grasp_point,
    ros::Publisher& marker_pub,
    int id)
{
  visualization_msgs::Marker marker;
  // Header
  marker.header.frame_id = grasp_point.header.frame_id;
  marker.header.stamp    = ros::Time::now();
  // Namespace and identifier
  marker.ns   = "grasp_point";
  marker.id   = id;
  // Marker type and action
  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  // Position and default orientation
  marker.pose.position = grasp_point.point;
  marker.pose.orientation.w = 1.0;

  // Scale of the sphere
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;

  // White color, fully opaque
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  // Zero lifetime means the marker persists until overwritten
  marker.lifetime = ros::Duration(0);

  // Publish the marker
  marker_pub.publish(marker);
}

/**
 * @brief Publish a set of points representing convex hull vertices.
 * 
 * @param corners Vector of stamped points defining the hull.
 * @param marker_pub ROS publisher for visualization_msgs::Marker.
 * @param id Unique identifier for this marker.
 */
void cw2::publishConvexHullMarker(
    const std::vector<geometry_msgs::PointStamped>& corners,
    ros::Publisher& marker_pub,
    int id /*= 0*/)
{
  if (corners.empty()) {
    ROS_WARN("No corner points to publish!");
    return;
  }

  visualization_msgs::Marker marker;
  // Use first corner's frame for header
  marker.header.frame_id = corners[0].header.frame_id;
  marker.header.stamp    = ros::Time::now();
  marker.ns   = "convex_hull_vertices";
  marker.id   = id;
  marker.type   = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;

  // Scale controls point width/height
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;

  // White color, fully opaque
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  // No pose offset, locked to frame
  marker.pose.orientation.w = 1.0;
  marker.frame_locked = true;
  marker.lifetime = ros::Duration(0);

  // Add each corner point
  for (const auto& stamped_point : corners) {
    marker.points.push_back(stamped_point.point);
  }

  marker_pub.publish(marker);
}

/**
 * @brief Publish a circular line strip marker around a centroid.
 * 
 * @param centroid Center of the circle.
 * @param radius Radius of the circle in meters.
 * @param marker_pub ROS publisher for visualization_msgs::Marker.
 * @param id Unique identifier for this marker.
 */
void cw2::publishCentroidCircleMarker(
    const geometry_msgs::Point& centroid,
    double radius,
    ros::Publisher& marker_pub,
    int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns     = "centroid_circle";
  marker.id     = id;
  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  // Line thickness
  marker.scale.x = 0.002;
  // Yellow color
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = ros::Duration(0);

  // Generate points around the circle in the XY plane
  const int SEGMENTS = 36;
  for (int i = 0; i <= SEGMENTS; ++i) {
    double angle = 2.0 * M_PI * i / SEGMENTS;
    geometry_msgs::Point p;
    p.x = centroid.x + radius * std::cos(angle);
    p.y = centroid.y + radius * std::sin(angle);
    p.z = centroid.z;  // constant height
    marker.points.push_back(p);
  }

  marker_pub.publish(marker);
}

/**
 * @brief Publish the global point cloud as a ROS PointCloud2.
 */
void cw2::publishAccumulatedCloud()
{
  if (!accumulated_cloud_ || accumulated_cloud_->empty()) {
    ROS_WARN("No accumulated cloud to publish.");
    return;
  }

  sensor_msgs::PointCloud2 cloud_msg;
  // Convert PCL cloud to ROS message
  pcl::toROSMsg(*accumulated_cloud_, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp    = ros::Time::now();

  // Publish and log count
  accumulated_cloud_pub_.publish(cloud_msg);
  ROS_INFO("Published accumulated cloud with %zu points.", accumulated_cloud_->size());
}

/**
 * @brief Publish an arrow marker indicating end-effector orientation (yaw).
 * 
 * @param origin Start point of the arrow.
 * @param yaw Desired yaw angle in radians.
 * @param marker_pub ROS publisher for visualization_msgs::Marker.
 * @param id Unique identifier for this marker.
 */
void cw2::publishOrientationArrow(
    const geometry_msgs::PointStamped& origin,
    double yaw,
    ros::Publisher& marker_pub,
    int id)
{
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = origin.header.frame_id;
  arrow.header.stamp    = ros::Time::now();
  arrow.ns     = "grasp_orientation";
  arrow.id     = id;
  arrow.type   = visualization_msgs::Marker::ARROW;
  arrow.action = visualization_msgs::Marker::ADD;

  // Define start and end points in the XY plane
  geometry_msgs::Point start = origin.point;
  double len = 0.1;  // arrow length
  geometry_msgs::Point end;
  end.x = start.x + len * std::cos(yaw);
  end.y = start.y + len * std::sin(yaw);
  end.z = start.z;

  arrow.points.push_back(start);
  arrow.points.push_back(end);

  // Scale.x = shaft diameter, scale.y = head diameter
  arrow.scale.x = 0.01;
  arrow.scale.y = 0.02;
  arrow.pose.orientation.w = 1.0;

  // Orange color
  arrow.color.r = 1.0;
  arrow.color.g = 0.5;
  arrow.color.b = 0.0;
  arrow.color.a = 1.0;

  arrow.lifetime = ros::Duration(0);
  marker_pub.publish(arrow);
}
