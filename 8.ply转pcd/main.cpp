
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace pcl::io;

int main (int argc, char** argv)
{
  pcl::PLYReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read<pcl::PointXYZ>("input.ply", *cloud);
  pcl::io::savePCDFile("pointcloud.pcd", *cloud );

  return 0;
}