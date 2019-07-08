#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{
  srand(time(NULL));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //点云生成
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
  }

//  输入点云数据
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

//  需要搜索的点云
  pcl::PointXYZ searchPoint;
  searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

  int K = 10;
//  k近邻相邻点的结果索引
  std::vector<int> k_indices(K);
//  相邻点的合成平方距离
  std::vector<float> k_sqr_distances(K);
  std::cout << "K nearest neighbor search at (" << searchPoint.x
            << " " << searchPoint.y
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;
//  K近邻搜索
  if(kdtree.nearestKSearch(searchPoint, K, k_indices, k_sqr_distances) >0){
      for (size_t i = 0; i < k_indices.size(); ++i){
          std::cout << "    " << cloud->points[k_indices[i]].x
                    << " " << cloud->points[k_indices[i]].y
                    << " " << cloud->points[k_indices[i]].z
                    << " (squared distance: " << k_sqr_distances[i] << ")" << std::endl;
      }
  }


// 半径内搜索近邻
    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;
    if(    kdtree.radiusSearch(searchPoint,radius, k_indices,k_sqr_distances )>0){
        for (size_t i = 0; i < k_indices.size(); ++i)
          std::cout << "    " << cloud->points[k_indices[i]].x
                    << " " << cloud->points[k_indices[i]].y
                    << " " << cloud->points[k_indices[i]].z
                    << " (squared distance: " << k_sqr_distances[i] << ")" << std::endl;

    }
  return 0;
}
