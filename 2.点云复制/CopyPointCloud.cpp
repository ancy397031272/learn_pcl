#include <iostream>
#include <pcl/common/io.h>

static void sameType();
static void differenceType();

int main() {
  sameType();
  differenceType();
}

void sameType() {
  using CloudType = pcl::PointCloud<pcl::PointXYZ>;
  CloudType::Ptr cloud(new CloudType);
  CloudType::PointType p;
  p.x = 1;
  p.y = 2;
  p.z = 3;
  cloud->push_back(p);
  std::cout << p.x << " " << p.y << " " << p.z << std::endl;
  CloudType::Ptr cloud2(new CloudType);
  copyPointCloud(*cloud, *cloud2);

  CloudType::PointType p_retrieved = (*cloud2)[0];
  std::cout << p_retrieved.x << " " << p_retrieved.y << " " << p_retrieved.z
            << std::endl;
}

void differenceType() {
  using CloudType = pcl::PointCloud<pcl::PointXYZ>;
  CloudType::Ptr cloud(new CloudType);

  CloudType::PointType p;
  p.x = 1;
  p.y = 2;
  p.z = 3;
  cloud->push_back(p);
  std::cout << p.x << " " << p.y << " " << p.z << std::endl;

  using CloudType2 = pcl::PointCloud<pcl::PointNormal>;
  CloudType2::Ptr cloud2(new CloudType2);
  copyPointCloud(*cloud, *cloud2);

  CloudType2::PointType p_retrieved = (*cloud2)[0];
  std::cout << p_retrieved.x << " " << p_retrieved.y << " " << p_retrieved.z
            << std::endl;
}