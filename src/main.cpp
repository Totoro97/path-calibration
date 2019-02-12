#include <iostream>
#include <string>
#include <vector>

#include <Eigen>
#include <opencv2/opencv.hpp>

#include "Algorithms.h"
#include "Calibrator.h"
#include "Thinning.h"

void Run(std::string img_0_path, std::string img_1_path) {
  // Load Image
  auto img_0 = cv::imread(img_0_path);
  auto img_1 = cv::imread(img_1_path);

  // Thinning
  /*
  cv::Mat img_0_gray, img_1_gray;
  cv::cvtColor(img_0, img_0_gray, cv::COLOR_BGR2GRAY);
  cv::cvtColor(img_1, img_1_gray, cv::COLOR_BGR2GRAY);
  cv::threshold(img_0_gray, img_0_gray, 10, 255, cv::THRESH_BINARY);
  cv::threshold(img_1_gray, img_1_gray, 10, 255, cv::THRESH_BINARY);
  */
  cv::Mat img_0_gray(img_0.rows, img_0.cols, CV_8UC1);
  for (int i = 0; i < img_0.rows * img_0.cols; i++) {
    if (img_0.data[i * 3] == img_0.data[0] &&
        img_0.data[i * 3 + 1] == img_0.data[1] &&
        img_0.data[i * 3 + 2] == img_0.data[2]) {
      img_0_gray.data[i] = 0;
    }
    else
      img_0_gray.data[i] = (uchar) 255;
  }
  cv::Mat img_1_gray(img_1.rows, img_1.cols, CV_8UC1);
  for (int i = 0; i < img_1.rows * img_1.cols; i++) {
    if (img_1.data[i * 3] == img_1.data[0] &&
        img_1.data[i * 3 + 1] == img_1.data[1] &&
        img_1.data[i * 3 + 2] == img_1.data[2]) {
      img_1_gray.data[i] = 0;
    }
    else
      img_1_gray.data[i] = (uchar) 255;
  }
  thinning(img_0_gray, img_0_gray);
  thinning(img_1_gray, img_1_gray);

  // Get path points (2D)
  std::vector<Eigen::Vector2i> path_2d;
  cv::imwrite("out.png", img_0_gray);
  Algo::GetPathPoints(img_0_gray, path_2d);

  // Test path.
  /*
  cv::Mat test_img(img_0.rows, img_0.cols, CV_8UC3);
  for (const auto &pt : path_2d) {
    if (pt(0) < 0) {
      continue;
    }
    cv::circle(test_img, cv::Point2d(pt(1), pt(0)), 1, cv::Scalar(255, 255, 255), 1);
    cv::imshow("path", test_img);
    cv::waitKey(10);
  }
  */

  // Main algorithm
  Calibrator calibrator(path_2d, img_1_gray);
  calibrator.Run();
}

int main() {
  // Run("data/5_0.png", "data/5_1.png");
  // Run("backup/0.png", "backup/1.png");
  // Run("/home/aska/Data/8.png", "/home/aska/Data/9.png");
  return 0;
}
