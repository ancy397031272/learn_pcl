#include <iostream>

#include <pcl/filters/filter.h>

int main() {
  using CloudType = pcl::PointCloud<pcl::PointXYZ>;
  CloudType::Ptr cloud (new CloudType);
  cloud->is_dense = false;
  CloudType::Ptr output_cloud (new CloudType);

  CloudType::PointType p_nan;
  p_nan.x = std::numeric_limits<float>::quiet_NaN();
  p_nan.y = std::numeric_limits<float>::quiet_NaN();
  p_nan.z = std::numeric_limits<float>::quiet_NaN();
  cloud->push_back(p_nan);

  CloudType::PointType p_valid;
  p_valid.x = 1.0f;
  cloud->push_back(p_valid);

  std::cout << "size: " << cloud->size () << std::endl;

  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*cloud, *output_cloud, indices);
  std::cout << "size: " << output_cloud->size () << std::endl;

  return 0;
}