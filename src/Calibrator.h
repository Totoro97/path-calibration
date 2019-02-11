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
  Calibrator(const std::vector<Eigen::Vector2i> &path_2d, const cv::Mat &img_gray, Calibrator *past_calibrator = nullptr);

  ~Calibrator() {
    delete(dist_map_);
  }

  void InitializeParameters(Calibrator *past_calibrator);
  void AddDeltaP(const Eigen::VectorXd &delta_p);
  double CalcCurrentError();
  void Run();
  void ShowSampledPoints();
  void ShowCurrentSituation();
  void SaveCurrentPoints();
  double GetDepth(int idx);
  double WarpDepth(int i, int j, double depth);
  Eigen::Vector2d Warp(int i, int j, double depth);

  cv::Mat img_gray_;
  std::vector<Eigen::Vector2i> path_2d_;
  std::vector<int> past_sampled_, next_sampled_;
  std::vector<int> sampled_;
  std::vector<double> depth_;
  int sit_counter_ = 0;

  DistMap *dist_map_ = nullptr;
  double cam_paras_[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -6.95 };
};
