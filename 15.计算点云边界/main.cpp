
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/features/boundary.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include "open3d/Open3D.h"
#include <iostream>


using namespace std;

int estimateBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float re,float reforn)
{

  pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>);
  normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud));
  normEst.setRadiusSearch(0.001); //设置法线估计的半径
  normEst.compute(*normals); //将法线估计结果保存至normals
  //输出法线的个数
  std:cout<<"reforn: "<<reforn<<std::endl;
  std::cerr << "normals: " << normals->size() << std::endl;

  boundEst.setInputCloud(cloud); //设置输入的点云
  boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
  boundEst.setRadiusSearch(0.001); //设置边界估计所需要的半径
  boundEst.setAngleThreshold(M_PI/2); //边界估计时的角度阈值
  boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
  boundEst.compute(boundaries); //将边界估计结果保存在boundaries

  //输出边界点的个数
  std::cerr << "boundaries: " <<boundaries.points.size() << std::endl;

  //存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
  for(int i = 0; i < cloud->points.size(); i++)
  {

    if(boundaries[i].boundary_point > 0)
    {
      cloud_boundary->push_back(cloud->points[i]);
    }
  }
  std::cout << "3333333333333333: "<<  cloud_boundary->points.size()<<std::endl;

  auto edge = open3d::geometry::PointCloud();
  edge.points_.clear();

  for(int i=0; i< cloud_boundary->points.size(); i++)
  {
    Eigen::Vector3d pt(cloud_boundary->points[i].x, cloud_boundary->points[i].y, cloud_boundary->points[i].z);
    edge.points_.push_back(pt);
  }
  std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(edge);

  cloudPtr1->PaintUniformColor({0, 1, 0});
  open3d::visualization::DrawGeometries({cloudPtr1});

  return 0;
}






int main()
{
  std::string filename = "/home/rvbust/Documents/2.ply";
  auto open3d_cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();

  if (open3d::io::ReadPointCloudFromPLY(filename, *open3d_cloud_ptr,open3d::io::ReadPointCloudOption()))
  {
    open3d::utility::LogInfo("Successfully read");
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_ptr->resize(open3d_cloud_ptr->points_.size());

  for (int i=0; i< open3d_cloud_ptr->points_.size(); i++) {
    cloud_ptr->points[i].x = open3d_cloud_ptr->points_[i].x();
    cloud_ptr->points[i].y = open3d_cloud_ptr->points_[i].y();
    cloud_ptr->points[i].z = open3d_cloud_ptr->points_[i].z();
  }

  std::cout << " 2222222222222222222" << std::endl;


  // 创建滤波器对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_ptr);
  sor.setMeanK (100);//寻找每个点的50个最近邻点
  sor.setStddevMulThresh (1.0);//一个点的最近邻距离超过全局平均距离的一个标准差以上，就会舍弃
  sor.filter (*cloud_filtered);
  std::cout<<"cloud_src: "<<cloud_ptr->points.size()<<std::endl;
  std::cout<<"cloud_filtered: "<<cloud_filtered->points.size()<<std::endl;

  float re=0.002,reforn=0.002;
  estimateBorders(cloud_filtered,re,reforn);


}


