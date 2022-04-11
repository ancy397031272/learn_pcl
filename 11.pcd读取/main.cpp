#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string pcd_path = "test_pcd.pcd";

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read file \n");
    return -1;
  }
  std::cout << "Loaded " << cloud->size()
            << "data points from test_pcd.pcd with the following fields: "
            << std::endl;

  for (auto &pt : cloud->points) {
    std::cout << "   " << pt.x << " " << pt.y << " " << pt.z << std::endl;
  }

  pcl::PCDReader  reader;
  pcl::PCLPointCloud2 pcd2;
  reader.read(pcd_path, pcd2);
  pcl::fromPCLPointCloud2 (pcd2, *cloud);
  for (auto &pt : cloud->points) {
    std::cout << "   " << pt.x << " " << pt.y << " " << pt.z << std::endl;
  }

  return 0;
}