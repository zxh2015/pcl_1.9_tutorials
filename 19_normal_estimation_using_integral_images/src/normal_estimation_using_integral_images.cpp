#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

//积分图进行法线估计

int main()
{
  //加载点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("table_scene_mug_stereo_textured.pcd", *cloud);
  //估计法线
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//  设置估计方法
  ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
//  最大深度变化系数
  ne.setMaxDepthChangeFactor(0.02f);
//  优化法线方向时的考虑的邻域大小
  ne.setNormalSmoothingSize(10.0f);
//  输入点云，这里必须是有序点云
  ne.setInputCloud(cloud);
//  执行法线估计
  ne.compute(*normals);

  //法线可视化
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
  return 0;
}
