#include <Eigen/Core> // 引入Eigen核心部分的头文件，用于进行矩阵等数学运算
#include <pcl/point_types.h> // 引入PCL库中点类型的定义
#include <pcl/point_cloud.h> // 引入PCL库中点云类型的定义
#include <pcl/common/time.h> // 引入PCL库中时间处理的功能
#include <pcl/console/print.h> // 引入PCL库的控制台打印功能，用于输出信息
#include <pcl/features/normal_3d_omp.h> // 引入PCL库中用于估计点云法线的功能
#include <pcl/features/fpfh_omp.h> // 引入PCL库中用于计算FPFH特征的功能
#include <pcl/filters/filter.h> // 引入PCL库中的滤波器功能
#include <pcl/filters/voxel_grid.h> // 引入PCL库中的体素网格滤波器，用于降采样
#include <pcl/io/pcd_io.h> // 引入PCL库中的点云数据输入输出功能
#include <pcl/registration/sample_consensus_prerejective.h> // 引入PCL库中进行样本一致性预拒绝配准的功能
#include <pcl/visualization/pcl_visualizer.h> // 引入PCL库中的点云可视化功能
 
 
// 类型定义 我们首先定义便利类型，以免代码混乱。
typedef pcl::PointNormal PointNT; // 定义包含法线信息的点类型
typedef pcl::PointCloud<PointNT> PointCloudT; // 定义包含法线的点云类型
typedef pcl::FPFHSignature33 FeatureT; // 定义用于FPFH特征的数据结构
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT; // 定义FPFH特征估计器类型
typedef pcl::PointCloud<FeatureT> FeatureCloudT; // 定义存放FPFH特征的点云类型
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT; // 定义用于点云颜色处理的类型
 

// 与场景中的杂乱和遮挡进行刚体对齐
int main (int argc, char **argv) 
{
  // 点云定义
  //然后我们实例化必要的数据容器，检查输入参数并加载对象和场景点云。
  //尽管我们已经定义了包含法线的基本点类型，但我们只为对象预先定义了法线
  //（通常是这种情况）。我们将估计下面场景的法线信息。
  PointCloudT::Ptr object (new PointCloudT); // 定义待对齐的对象点云
  PointCloudT::Ptr object_aligned (new PointCloudT); // 定义对齐后的对象点云
  PointCloudT::Ptr scene_before_downsampling (new PointCloudT); // 定义降采样前的场景点云
  PointCloudT::Ptr scene (new PointCloudT); // 定义场景点云
  FeatureCloudT::Ptr object_features (new FeatureCloudT); // 定义对象点云的特征
  FeatureCloudT::Ptr scene_features (new FeatureCloudT); // 定义场景点云的特征
  
  // 获取输入对象和场景
  if (argc != 3)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return (1);
  }
  std::cout << argv[1] << endl;
  std::cout << argv[2] << endl;
  // 加载对象和场景
  pcl::console::print_highlight ("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
      pcl::io::loadPCDFile<PointNT> (argv[2], *scene_before_downsampling) < 0)
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
    return (1);
  }
  
  // 降采样 为了加快处理速度，我们使用 PCL 的 VoxelGrid 类
  // 将对象和场景点云下采样至 5 毫米的分辨率。
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid; // 创建体素网格滤波器
  const float leaf = 0.005f; // 定义体素大小
  grid.setLeafSize (leaf, leaf, leaf); // 设置体素大小
  grid.setInputCloud (object); // 设置输入点云为对象点云
  grid.filter (*object); // 执行滤波
  grid.setInputCloud (scene_before_downsampling); // 设置输入点云为场景点云
  grid.filter (*scene); // 执行滤波
  
  // 为场景估计法线
  // 现在使用 PCL 的 NormalEstimationOMP 来估计场景中缺失的表面法线。
  // 计算下面用于匹配的特征需要表面法线。
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest; // 创建法线估计器
  nest.setRadiusSearch (0.005); // 设置搜索半径
  nest.setInputCloud (scene); // 设置输入点云
  nest.setSearchSurface (scene_before_downsampling); // 设置搜索表面
  nest.compute (*scene); // 执行法线估计
  
  // 估计特征
  //对于下采样点云中的每个点，我们现在使用 PCL 的 FPFHEstimationOMP 类
  // 来计算用于在对齐过程中进行匹配的快速点特征直方图 (FPFH) 描述符。
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest; // 创建FPFH特征估计器
  fest.setRadiusSearch (0.025); // 设置搜索半径
  //object同时包含每个点的位置和法线数据
  fest.setInputCloud (object); // 设置输入点云为对象点云
  fest.setInputNormals (object); // 设置输入法线 object同时包含每个点的位置和法线数据
  fest.compute (*object_features); // 计算对象点云的特征
  fest.setInputCloud (scene); // 设置输入点云为场景点云
  fest.setInputNormals (scene); // 设置输入法线为场景点云法线
  fest.compute (*scene_features); // 计算场景点云的特征
 
 // 执行对齐 我们现在准备好设置对齐过程。我们使用 
 // SampleConsensusPrerejective 类，它实现了高效的 RANSAC 位姿估计
 // 循环。这是通过使用 CorrespondenceRejectorPoly 类提前消除不良姿势
 // 假设来实现的。
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align; // 创建对齐器
  align.setInputSource (object); // 设置源点云
  align.setSourceFeatures (object_features); // 设置源点云特征
  align.setInputTarget (scene); // 设置目标点云
  align.setTargetFeatures (scene_features); // 设置目标点云特征
  align.setMaximumIterations (50000); // 设置RANSAC迭代次数
  align.setNumberOfSamples (3); // 设置采样点数
  align.setCorrespondenceRandomness (5); // 设置最近特征使用数
  align.setSimilarityThreshold (0.95f); // 设置边长相似度阈值
  align.setMaxCorrespondenceDistance (2.5f * leaf); // 设置内点阈值
  align.setInlierFraction (0.25f); // 设置接受一个姿态假设所需的内点比例
  {// 最后，我们准备好执行对齐过程。
    pcl::ScopeTime t("Alignment"); // 记录对齐时间
    align.align (*object_aligned); // 执行对齐
  }// 对齐的对象存储在点云object_aligned中。
  // 如果找到具有足够内点的姿势（超过对象点总数的 25%），
  // 则称算法收敛，我们可以打印和可视化结果。
  
  if (align.hasConverged ())//在最后一次对齐运行之后返回收敛状态
  {
    // 打印结果
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation (); // 获取最终变换矩阵
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
    
    // 显示对齐结果
    pcl::visualization::PCLVisualizer visu("Alignment"); // 创建可视化器
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene"); // 添加场景点云
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned"); // 添加对齐后的对象点云
    visu.spin (); // 开始交互式可视化
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n"); // 打印对齐失败信息
    return (1);
  }
  
  return (0);
}