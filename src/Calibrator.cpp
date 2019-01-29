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
      sampled_.push_back(i);
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

  // ShowSampledPoints();
  ShowCurrentSituation();
  bool iter_ok = false;
  while (!iter_ok) {
    Eigen::MatrixXd A(num_valid_funcs, 6 + num_ex_paras_);
    Eigen::VectorXd B(num_valid_funcs);
    int idx = -1;
    for (int i = 0; i < path_2d_.size(); i++) {
      if (path_2d_[i](0) == -1) {
        continue;
      }
      idx++;
      auto warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
      B(idx) = dist_map_->Distance(warped(0), warped(1));
      const double step_len = 1.0 / (double) (1 << 10);
      Eigen::Vector2d grad_i;
      grad_i(0) =
        (dist_map_->Distance(warped(0) + step_len, warped(1)) - B(idx)) / step_len;
      grad_i(1) =
        (dist_map_->Distance(warped(0), warped(1) + step_len) - B(idx)) / step_len;
      int p_idx = -1;

      // Depth Paras.
      for (int p : sampled_) {
        p_idx++;
        if (next_sampled_[i] != p && past_sampled_[i] != p) {
          A(idx, p_idx) = 0.0;
          continue;
        }
        double new_depth_ = depth_[p] + step_len;
        std::swap(new_depth_, depth_[p]);
        auto new_warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
        std::swap(new_depth_, depth_[p]);
        A(idx, p_idx) = grad_i.dot((new_warped - warped) / step_len);
      }

      // Camera Paras.
      for (int t = 0; t < 6; t++) {
        p_idx++;
        cam_paras_[t] += step_len;
        auto new_warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
        cam_paras_[t] -= step_len;
        A(idx, p_idx) = grad_i.dot((new_warped - warped) / step_len);
      }
    }

    // Solve least square.
    Eigen::VectorXd delta_p = A.colPivHouseholderQr().solve(B);
    int p_idx = -1;
    for (int p : sampled_) {
      depth_[p] += delta_p(++p_idx);
    }
    for (int t = 0; t < 6; t++) {
      cam_paras_[t] += delta_p(++p_idx);
    }
    ShowCurrentSituation();
  }
}

void Calibrator::ShowSampledPoints() {
  cv::Mat img(dist_map_->height_, dist_map_->width_, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < path_2d_.size(); i++) {
    if (next_sampled_[i] == i) {
      cv::circle(img, cv::Point(path_2d_[i](1), path_2d_[i](0)), 0, cv::Scalar(0, 255, 0), 1);
    }
  }
  cv::imshow("Sampled", img);
  cv::waitKey(-1);
}

void Calibrator::ShowCurrentSituation() {
  int idx = -1;
  double current_error = 0.0;
  cv::Mat img(dist_map_->height_, dist_map_->width_, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) == -1) {
      continue;
    }
    idx++;
    auto warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
    cv::circle(img, cv::Point(warped(1), warped(0)), 0, cv::Scalar(255, 0, 0), 1);
    current_error += dist_map_->Distance(warped(0), warped(1));
  }
  current_error /= (double) idx;
  std::cout << "current_error: " << current_error << std::endl;
  cv::imshow("Current", img);
  cv::waitKey(-1);
}

double Calibrator::GetDepth(int idx) {
  if (next_sampled_[idx] == idx) {
    return depth_[idx];
  }
  else {
    int a = past_sampled_[idx];
    int b = next_sampled_[idx];
    double bias = ((double) (idx - a)) / ((double) (b - a));
    return depth_[a] * (1.0 - bias) + depth_[b] * bias;
  }
}

Eigen::Vector2d Calibrator::Warp(int i, int j, double depth) {
  double x = (j - dist_map_->width_  / 2.0) * depth;
  double y = (i - dist_map_->height_ / 2.0) * depth;
  double z = depth;
  // 0, 1, 2: translation.
  double t_x = x + cam_paras_[0];
  double t_y = y + cam_paras_[1];
  double t_z = z + cam_paras_[2];
  // 3, 4, 5: rotation.
  Eigen::Vector3d w(cam_paras_[3], cam_paras_[4], cam_paras_[5]);
  Eigen::Matrix3d t;
  t = Eigen::AngleAxisd(w.norm(), w);
  auto r_coord = t * Eigen::Vector3d(t_x, t_y, t_z);

  return Eigen::Vector2d(
    r_coord(1) / r_coord(2) + (double) dist_map_->height_ / 2,
    r_coord(0) / r_coord(2) + (double) dist_map_->width_ / 2
    );
}