//
// Created by aska on 19-2-1.
//

#include "Utils.h"

void Utils::SavePointsAsPly(std::string save_path, const std::vector<Eigen::Vector3d> &points) {
  std::ofstream my_file;
  my_file.open(save_path.c_str());
  my_file << "ply\nformat ascii 1.0\n";
  my_file << "element vertex " << points.size() << "\n";
  my_file << "property float32 x\nproperty float32 y\nproperty float32 z\n";
  my_file << "end_header\n";
  for (const auto &pt : points) {
    my_file << pt(0) << " " << pt(1) << " " << pt(2) << "\n";
  }
  my_file.close();
}