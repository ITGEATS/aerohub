#include <iomanip>
#include <iostream>
#include "uav_follow_path_execution/timeline.hpp"

int main() {
  namespace execution = uav_follow_path_execution;
  execution::Path path;
  path.fields = 7;
  std::size_t count;
  std::cin >> path.start >> count;
  path.points.resize(count);
  for (auto & point : path.points) {
    std::cin >> point.t;
    for (auto & value : point.p) {std::cin >> value;}
    for (auto & value : point.v) {std::cin >> value;}
    for (auto & value : point.a) {std::cin >> value;}
    std::cin >> point.yaw;
  }
  double epoch;
  std::cout << std::setprecision(17);
  while (std::cin >> epoch) {
    const auto point = execution::sample(path, epoch);
    for (auto value : point.p) {std::cout << value << ' ';}
    for (auto value : point.v) {std::cout << value << ' ';}
    for (auto value : point.a) {std::cout << value << ' ';}
    std::cout << point.yaw << '\n';
  }
}
