#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main(int argc,char**argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // 创建点云
    cloud.width=5;
    cloud.height=1;
    cloud.is_dense=false;
    cloud.points.resize(cloud.width*cloud.height);
    //随机写入点云数据
    for(size_t i=0;i<cloud.points.size();++i)
    {
        cloud.points[i].x=1024*rand()/(RAND_MAX+1.0f);
        cloud.points[i].y=1024*rand()/(RAND_MAX+1.0f);
        cloud.points[i].z=1024*rand()/(RAND_MAX+1.0f);
    }
    //    保存点云
    pcl::io::savePCDFileASCII("test_pcd.pcd",cloud);

    //    打印信息
    std::cerr<<"Saved "<<cloud.points.size()<<" data points to test_pcd.pcd."<<std::endl;
    for(size_t i=0;i<cloud.points.size();++i)
        std::cerr<<"    "<<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z<<std::endl;
    return(0);
}
