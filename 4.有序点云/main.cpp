// STL
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main() {
  // Setup the cloud
  using PointType = pcl::PointXYZ;
  using CloudType = pcl::PointCloud<PointType>;
  CloudType::Ptr cloud(new CloudType);

  // Make the cloud a 10x10 grid
  cloud->height = 10;
  cloud->width = 10;
  cloud->is_dense = true;
  cloud->resize(cloud->height * cloud->width);

  // Output the (0,0) point
  std::cout << (*cloud)(0, 0) << std::endl;

  // Set the (0,0) point
  PointType p;
  p.x = 1;
  p.y = 2;
  p.z = 3;
  (*cloud)(0, 0) = p;

  // Confirm that the point was set
  std::cout << (*cloud)(0, 0) << std::endl;

  return (0);
}