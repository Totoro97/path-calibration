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
  Calibrator(const cv::Mat &img_gray);

  ~Calibrator() {
    delete(dist_map_);
  }

  void InitialSample();
  void InitializeParameters(Calibrator *past_calibrator);
  void AddDeltaP(const Eigen::VectorXd &delta_p);
  double CalcCurrentError();
  void Run(int max_iter_num = (1 << 30));
  void ShowSampledPoints();
  void ShowCurrentSituation();
  void SaveCurrentPoints();
  double GetDepth(int idx);
  double FocalLength();
  Eigen::Vector3d Pix2World(int i, int j, double depth);
  Eigen::Vector3d World2Cam(Eigen::Vector3d pt_world);
  Eigen::Vector2d Warp(int i, int j, double depth);

  cv::Mat img_gray_;
  std::vector<Eigen::Vector2i> path_2d_;
  std::vector<int> past_sampled_, next_sampled_;
  std::vector<int> sampled_;
  std::vector<double> depth_;
  int sit_counter_ = 0;
  int height_, width_;
  int num_ex_paras_;
  bool has_iterated_ = false;
  Calibrator *another_calibrator_;
  DistMap *dist_map_ = nullptr;
  // TODO: Hard code here.
  double cam_paras_[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -6.95 };
  // double biased_cam_paras_[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -6.95 };
};
