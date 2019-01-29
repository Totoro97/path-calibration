//
// Created by aska on 19-1-29.
//

#include "Calibrator.h"

void Calibrator::Run() {
  // Initialize Parameters.
  double r_a = 0.0, r_b = 0.0, r_c = 0.0;
  double t_x = 0.0, t_y = 0.0, t_z = 0.0;

  int num_ex_paras_ = 0;
  past_sampled_.resize(path_2d_.size(), -1);
  next_sampled_.resize(path_2d_.size(), -1);
  depth_.resize(path_2d_.size(), 1.0);

  for (int i = 0; i < path_2d_.size(); i++) {
    if (i == 0 || i == path_2d_.size() - 1 || path_2d_[i - 1](0) == -1 || path_2d_[i + 1](0) == -1) {
      past_sampled_[i] = next_sampled_[i] = i;
      num_ex_paras_++;
    }
  }
  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) != -1 && past_sampled_[i] == -1) {
      past_sampled_[i] = past_sampled_[i - 1];
    }
  }
  for (int i = (int) path_2d_.size() - 1; i >= 0; i--) {
    if (path_2d_[i](0) != -1 && next_sampled_[i] == -1) {
      next_sampled_[i] = next_sampled_[i + 1];
    }
  }

  int num_valid_funcs = 0;
  for (const auto &pt : path_2d_) {
    if (pt(0) != -1) {
      num_valid_funcs++;
    }
  }

  auto GetDepth = [this](int idx) {
    
  };
  bool iter_ok = false;
  while (!iter_ok) {
    Eigen::MatrixXd A(num_valid_funcs, 6 + num_ex_paras_);
    int idx = 0;
    for (int i = 0; i < path_2d_.size(); i++) {
      if (path_2d_[i](0) == -1) {
        continue;
      }
      auto warped = Warp(path_2d_[i](0), path_2d_[i](1), );
    }
  }
}