//
// Created by aska on 19-1-29.
//

#include <Eigen>
#include <opencv2/opencv.hpp>
#include <vector>

#include "DistMap.h"

class Calibrator {
public:
  Calibrator(const std::vector<Eigen::Vector2i> &path_2d, const cv::Mat &img_gray): path_2d_(path_2d) {
    dist_map_ = new DistMap(img_gray, true);
  }

  void Run();

  ~Calibrator() {
    delete(dist_map_);
  }
  std::vector<Eigen::Vector2i> path_2d_;
  std::vector<int> past_sampled_, next_sampled_;
  std::vector<int> sampled_;
  std::vector<double> depth_;

  DistMap *dist_map_ = nullptr;
  double cam_paras_[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
};
