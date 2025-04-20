#include "cw2_class.h"

bool cw2::extract_objects(const octomap::OcTree& tree,
    bool                   neighbor26,
    std::vector<DetectedObj>& out)
  {
  using Key = octomap::OcTreeKey;
  
  const double res = tree.getResolution();
  const int    min_voxel_threshold = 200;
  const int deltas[26][3] = {            // 26‑邻接表
  {-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1},
  {-1,-1,0},{-1,1,0},{1,-1,0},{1,1,0},
  {-1,0,-1},{-1,0,1},{1,0,-1},{1,0,1},
  {0,-1,-1},{0,-1,1},{0,1,-1},{0,1,1},
  {-1,-1,-1},{-1,-1,1},{-1,1,-1},{-1,1,1},
  {1,-1,-1},{1,-1,1},{1,1,-1},{1,1,1} };
  
  /* ---------- 1. 收集所有占据体素 ---------- */
  std::unordered_set<Key, cw2::KeyHash> occ;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
  if (tree.isNodeOccupied(*it)) occ.insert(it.getKey());
  
  if (occ.empty()) { ROS_WARN("OctoMap is empty."); return false; }
  
  /* ---------- 2. flood‑fill ---------- */
  std::unordered_set<Key, cw2::KeyHash> vis;
  std::vector<Key> stack; stack.reserve(2048);
  
  int obj_idx = 1;
  for (const Key& seed : occ)
  {
  if (vis.count(seed)) continue;
  
  /* -- 聚类基本属性 -- */
  double min_x=1e9,max_x=-1e9,min_y=1e9,max_y=-1e9,min_z=1e9,max_z=-1e9;
  size_t voxel_cnt = 0;
  std::unordered_set<Key, cw2::KeyHash> cluster;
  stack.clear();   stack.push_back(seed);   vis.insert(seed);
  
  while (!stack.empty()) {
  Key cur = stack.back(); stack.pop_back();
  voxel_cnt++; cluster.insert(cur);
  octomap::point3d p = tree.keyToCoord(cur);
  min_x = std::min(min_x, (double)p.x()); max_x = std::max(max_x, (double)p.x());
  min_y = std::min(min_y, (double)p.y()); max_y = std::max(max_y, (double)p.y());
  min_z = std::min(min_z, (double)p.z()); max_z = std::max(max_z, (double)p.z());
  
  int nb = neighbor26 ? 26 : 6;
  for (int i = 0; i < nb; ++i) {
  Key nbk(cur[0]+deltas[i][0], cur[1]+deltas[i][1], cur[2]+deltas[i][2]);
  if (occ.count(nbk) && !vis.count(nbk)) { vis.insert(nbk); stack.push_back(nbk); }
  }
  }
  if (voxel_cnt < min_voxel_threshold) continue;
  
  /* ---------- 3. 分类 ---------- */
  double height = max_z - min_z + res;
  std::string category;
  if      (height > 0.05) category = "obstacle";
  else if (height >= 0.03) category = "basket";
  else                     category = "object";
  
  std::string shape = "N/A";
  geometry_msgs::Point centroid_pt{};   // 默认 (0,0,0)
  
  /* ---------- 4. 质心计算 ---------- */
  if (category == "object")      // 原有逻辑：只用顶部表面平均
  {
  octomap::OcTreeKey topKey = tree.coordToKey(0,0,max_z);
  std::vector<octomap::point3d> surf;
  for (const Key& k : cluster)
  if (k[2] == topKey[2]) surf.emplace_back(tree.keyToCoord(k));
  
  if (!surf.empty()) {
  for (const auto& p : surf) {
  centroid_pt.x += p.x(); centroid_pt.y += p.y(); centroid_pt.z += p.z();
  }
  centroid_pt.x /= surf.size(); centroid_pt.y /= surf.size(); centroid_pt.z /= surf.size();
  }
  
  /* --- 十字 / 圆环判别 --- */
  Key center_key = tree.coordToKey(centroid_pt.x, centroid_pt.y, max_z);
  bool center_occ = false;
  for (int dx=-1; dx<=1 && !center_occ; ++dx)
  for (int dy=-1; dy<=1 && !center_occ; ++dy) {
  Key ck(center_key[0]+dx, center_key[1]+dy, center_key[2]);
  if (cluster.count(ck)) { center_occ = true; break; }
  }
  shape = center_occ ? "cross" : "nought";
  }
  else if (category == "basket")   // 👈 新增：篮子质心 = 包围盒中心
  {
  centroid_pt.x = 0.5 * (min_x + max_x);
  centroid_pt.y = 0.5 * (min_y + max_y);
  centroid_pt.z = 0.5 * (min_z + max_z);
  }
  /* obstacle 无需质心 */
  
  /* ---------- 5. 保存结果 ---------- */
  DetectedObj d;
  d.centroid    = centroid_pt;
  d.category    = category;
  d.shape       = shape;
  d.voxel_keys  = cluster;
  out.push_back(d);
  
  ROS_INFO("Obj%02d vox=%zu h=%.3f  cat=%s  shape=%s",
  obj_idx++, voxel_cnt, height, category.c_str(), shape.c_str());
  }
  return true;
  }
  
  
  
  
  void cw2::build_octomap_from_accumulated_clouds()
  {
    ROS_INFO("Building OctoMap from RGB accumulated cloud...");
  
    double resolution = 0.005;
  
    if (!latest_octree_)
        latest_octree_ = std::make_shared<octomap::OcTree>(resolution);
    else
        latest_octree_->clear();
  
    for (const auto& pt : accumulated_cloud_->points)
    {
      if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
      {
        latest_octree_->updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
      }
    }
  
    latest_octree_->updateInnerOccupancy();
  
    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id = "world";
    map_msg.header.stamp = ros::Time::now();
    if (octomap_msgs::fullMapToMsg(*latest_octree_, map_msg))
    {
      octomap_pub_.publish(map_msg);
      ROS_INFO("Published octomap with %zu nodes.", latest_octree_->size());
    }
    else
    {
      ROS_ERROR("Failed to convert octomap to ROS message.");
    }
  }