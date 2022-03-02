
#include <iostream>
#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/point_types.h>

using namespace std;

int main() {
  pcl::PointXYZ p_valid;
  p_valid.x = 0;
  p_valid.y = 0;
  p_valid.z = 0;

  cout << "IS p_valid valid? " << pcl::isFinite(p_valid) << endl;

  // If any component is NaN, the point is not finite.
  pcl::PointXYZ p_invalid;
  p_invalid.x = std::numeric_limits<float>::quiet_NaN();
  p_invalid.y = 0;
  p_invalid.z = 0;
  std::cout << "Is p_invalid valid? " << pcl::isFinite(p_invalid) << std::endl;

  return 0;
}