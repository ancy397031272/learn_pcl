#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

int main(int, char **) {
  using CloudType = pcl::PointCloud<pcl::PointXYZ>;
  CloudType::Ptr cloud;
  cloud =CloudType::Ptr(new CloudType);
  pcl::io::loadPCDFile<pcl::PointXYZ>("your_pcd_file.pcd", *cloud);
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud, minPt, maxPt);
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;
  return (0);
}