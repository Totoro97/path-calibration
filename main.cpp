#include <iostream>
#include <Eigen>
#include <string>
#include <opencv2/opencv.hpp>

void Run(std::string img_0_path, std::string img_1_path) {
  // Load Image
  auto img_0 = cv::imread(img_0_path);
  auto img_1 = cv::imread(img_1_path);

  // Thinning
  cv::Mat img_0_gray, img_1_gray;
  cv::cvtColor(img_0, img_0_gray, cv::COLOR_BGR2GRAY);
  cv::cvtColor(img_1, img_1_gray, cv::COLOR_BGR2GRAY);

  // Get path points (2D)

  // Main algorithm
  // Calibrater.Run()

}

int main() {
  Run("1.png", "2.png");
  return 0;
}
