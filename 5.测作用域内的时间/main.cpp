#include <iostream>

#include <pcl/common/time.h>

int main() {
  pcl::ScopeTime scope_time("Test loop");
  {
    float total = 0.0f;
    for (std::size_t i = 0; i < 1e4; ++i) {
      total += static_cast<float>(i);
    }
  }
  std::cout << "Done." << std::endl;

  return (0);
}