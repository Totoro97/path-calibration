#include <iostream>
#include <string>
#include <vector>

#include <Eigen>
#include <opencv2/opencv.hpp>

#include "Algorithms.h"
#include "Thinning.h"

void Run(std::string img_0_path, std::string img_1_path) {
  // Load Image
  auto img_0 = cv::imread(img_0_path);
  auto img_1 = cv::imread(img_1_path);

  // Thinning
  cv::Mat img_0_gray, img_1_gray;
  cv::cvtColor(img_0, img_0_gray, cv::COLOR_BGR2GRAY);
  cv::cvtColor(img_1, img_1_gray, cv::COLOR_BGR2GRAY);
  cv::threshold(img_0_gray, img_0_gray, 10, 255, cv::THRESH_BINARY);
  cv::threshold(img_1_gray, img_1_gray, 10, 255, cv::THRESH_BINARY);

  thinning(img_0_gray, img_0_gray);

  // Get path points (2D)
  std::vector<Eigen::Vector2i> path_2d;
  cv::imwrite("out.png", img_0_gray);
  Algo::GetPathPoints(img_0_gray, path_2d);

  // Main algorithm
  // Calibrater.Run(path_2d, img_1_gray);
}

int main() {
  Run("0.png", "1.png");
  return 0;
}
