//
// Created by aska on 19-1-29.
//
#pragma once

#include <Eigen>
#include <opencv2/opencv.hpp>
#include <vector>

#include "DistMap.h"
#include "Utils.h"

class Calibrator {
public:
  Calibrator(const std::vector<Eigen::Vector2i> &path_2d, const cv::Mat &img_gray):
    path_2d_(path_2d), img_gray_(img_gray) {
    dist_map_ = new DistMap(img_gray, true);
    dist_map_->ShowDistMap();
  }
  ~Calibrator() {
    delete(dist_map_);
  }

  void AddDeltaP(const Eigen::VectorXd &delta_p);
  double CalcCurrentError();
  void Run();
  void ShowSampledPoints();
  void ShowCurrentSituation();
  void SaveCurrentPoints();
  double GetDepth(int idx);

  cv::Mat img_gray_;
  Eigen::Vector2d Warp(int i, int j, double depth);
  std::vector<Eigen::Vector2i> path_2d_;
  std::vector<int> past_sampled_, next_sampled_;
  std::vector<int> sampled_;
  std::vector<double> depth_;
  int sit_counter_ = 0;

  DistMap *dist_map_ = nullptr;
  double cam_paras_[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0 };
};
