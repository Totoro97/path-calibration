//
// Created by aska on 19-1-29.
//

#include "Algorithms.h"
#include "Calibrator.h"


Calibrator(const cv::Mat &img_gray) : img_gray_(img_gray) {
  height_ = img_gray.rows;
  width_ = img_gray.cols;
  Algo::GetPathPoints(path_2d_, img_gray_);
  dist_map_ = new DistMap(img_gray, true);
  InitializeParameters(nullptr);
}

// ----------------- deprecated -----------------
/*
Calibrator::Calibrator(const std::vector<Eigen::Vector2i> &path_2d,
                       const cv::Mat &img_gray,
                       Calibrator *another_calibrator):
  path_2d_(path_2d), img_gray_(img_gray) {
  dist_map_ = new DistMap(img_gray, true);
  dist_map_->ShowDistMap();
  InitializeParameters(another_calibrator);
}
*/

void Calibrator::InitializeParameters(Calibrator *another_calibrator) {
  // Too many hard codes......
  another_calibrator_ = another_calibrator;
  if (another_calibrator == nullptr) {
    // TODO: Hard code here.
    depth_.resize(path_2d_.size(), 2.3);
    std::memset(cam_paras_, 0, sizeof(double) * 7);
  }
  else if (!has_iterated_) {
    // TODO: Hard code here.
    depth_.resize(path_2d_.size(), 2.3);
    std::memcpy(cam_paras_, another_calibrator->cam_paras_, sizeof(double) * 7);
  }
  else {
    std::vector<double> tmp_sum;
    std::vector<double> new_depth;

    // depth
    // TODO: Hard code here.
    new_depth.resize(path_2d_.size(), 2.3);
    tmp_sum.resize(path_2d_.size(), 0.0);
    for (int i = 0; i < another_calibrator->path_2d_.size(); i++) {
      auto pix_2d = another_calibrator->path_2d_[i];
      if (pix_2d(0) == -1) {
        continue;
      }
      Eigen::Vector3d pt_world = another_calibrator->Pix2World(pix_2d(0), pix_2d(1), another_calibrator->GetDepth(i));
      Eigen::Vector3d pt_cam = World2Cam(pt_world);
      double new_depth = pt_cam(2); // z
      double focal_length = another_calibrator->FocalLength();
      Eigen::Vector2d new_pix_2d(
        pt_cam(1) / (new_depth * focal_length) + height_ / 2,
        pt_cam(0) / (new_depth * focal_length) + width_ / 2);

      for (int j = 0; j < path_2d_.size(); j++) {
        if (path_2d_[j](0) == -1) {
          continue;
        }
        double distance = (Eigen::Vector2d(path_2d_[j](0), path_2d_[j](1)) - new_pix_2d).norm();
        // TODO: Hard code here.
        if (distance > 5.0) {
          continue;
        }
        double c = std::exp(-distance);
        tmp_sum[j] += c;
        new_depth[j] += c * new_depth;
      }
    }

    // TODO: Hard code here.
    double change_ratio = 1.0;
    for (int j = 0; j < path_2d_.size(); j++) {
      if (tmp_sum[j] > 1e-9) {
        depth_[j] = depth_[j] * (1.0 - change_ratio) + new_depth[j] * change_ratio;
      }
    }
  }
}

void Calibrator::InitialSample() {
  num_ex_paras_ = 0;
  past_sampled_.resize(path_2d_.size(), -1);
  next_sampled_.resize(path_2d_.size(), -1);

  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) == -1) {
      continue;
    }
    if (i == 0 || i == path_2d_.size() - 1 || path_2d_[i - 1](0) == -1 || path_2d_[i + 1](0) == -1) {
      past_sampled_[i] = next_sampled_[i] = i;
      sampled_.push_back(i);
      num_ex_paras_++;
    }
    else if (i % 40 == 0) {
      past_sampled_[i] = next_sampled_[i] = i;
      sampled_.push_back(i);
      num_ex_paras_++;
    }
  }
  std::cout << "num_ex_paras = " << num_ex_paras_ << std::endl;

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
}

void Calibrator::Run(int max_iter_num) {
  int num_valid_funcs = 0;

  for (const auto &pt : path_2d_) {
    if (pt(0) != -1) {
      num_valid_funcs++;
    }
  }

  // ShowSampledPoints();
  SaveCurrentPoints();
  ShowCurrentSituation();
  int iter_counter = 0;
  while (iter_counter++ < max_iter_num) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_valid_funcs, 7 + num_ex_paras_);
    Eigen::VectorXd B = Eigen::VectorXd::Zero(num_valid_funcs);
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
      // Depth Paras.
      int p_idx = -1;
      for (int p : sampled_) {
        p_idx++;
        // fix first depth
        if (p_idx == 0 || (iter_counter & 3) != 0) {
          continue;
        }
        // TODO: Hard code here.
        if (iter_counter < 12)
          continue;
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
      for (int t = 0; t < 7; t++) {
        p_idx++;
        if (t < 3 && (iter_counter & 3) != 2) {
          continue;
        }
        if (t >= 3 && t < 6 && (iter_counter & 3) != 1) {
          continue;
        }
        if (t >= 6 && (iter_counter & 3) != 3) {
          continue;
        }
        cam_paras_[t] += step_len;
        auto new_warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
        cam_paras_[t] -= step_len;
        A(idx, p_idx) = grad_i.dot((new_warped - warped) / step_len);
        //if (t == 6) {
        //  std::cout << "focus length gradient: " << A(idx, p_idx) << std::endl;
        //}
      }
    }

    // Drop out.
    // TODO: Hard code here.
    double drop_out_ratio = 0.01;
    double all_sum = 0.0;
    std::vector<std::pair<double, int> > rank_list;
    for (int j = 0; j < 7 + num_ex_paras_; j++) {
      double tmp_sum = 0.0;
      for (int i = 0; i < num_valid_funcs; i++) {
        tmp_sum += std::abs(A(i, j));
      }
      rank_list.emplace_back(tmp_sum, j);
      all_sum += tmp_sum;
    }
    std::sort(rank_list.begin(), rank_list.end());
    double res = all_sum * drop_out_ratio;
    for (auto iter = rank_list.begin(); iter != rank_list.end() && res > 0.0; iter++) {
      res -= iter->first;
      if (res > 1e-9) {
        std::cout << "bababababbabababa: " << iter->first << std::endl;
        int idx_p = iter->second;
        for (int idx_i = 0; idx_i < num_valid_funcs; idx_i++) {
          A(idx_i, idx_p) = 0.0;
        }
      }
    }
    // Solve least square.
    // Eigen::VectorXd delta_p = A.colPivHouseholderQr().solve(-B);
    // ---------------------
    Eigen::VectorXd delta_p = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-B);
    // ---------------------
    // Or: Grad?
    // Eigen::VectorXd delta_p = Eigen::VectorXd::Zero(7 + num_ex_paras_);
    // for (int i = 0; i < num_valid_funcs; i++) {
    //   delta_p += A.block(i, 0, 1, 7 + num_ex_paras_).transpose();
    // }
    // delta_p *= -1e-3 / num_valid_funcs;

    res = all_sum * drop_out_ratio;
    for (auto iter = rank_list.begin(); iter != rank_list.end() && res > 0.0; iter++) {
      res -= iter->first;
      if (res > 1e-9) {
        int idx_p = iter->second;
        delta_p(idx_p) = 0.0;
      }
    }

    double current_error = CalcCurrentError();
    while (true) {
      std::cout << delta_p << std::endl;
      AddDeltaP(delta_p);
      double new_error = CalcCurrentError();
      SaveCurrentPoints();
      ShowCurrentSituation();
      /*if (new_error > current_error * 1.1 || delta_p.norm() > 1.0 ) {
        AddDeltaP(-delta_p);
        delta_p *= 0.5;
      }
      else {*/
        break;
      //}
    }
    /*if (new_error > current_error) {
      AddDeltaP(-delta_p);
      break;
    };*/
  }
  has_iterated_ = true;
}

void Calibrator::AddDeltaP(const Eigen::VectorXd &delta_p) {
  int p_idx = -1;
  for (int p : sampled_) {
    depth_[p] += delta_p(++p_idx);
  }
  for (int t = 0; t < 7; t++) {
    cam_paras_[t] += delta_p(++p_idx);
  }
}

void Calibrator::ShowSampledPoints() {
  cv::Mat img(dist_map_->height_, dist_map_->width_, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < path_2d_.size(); i++) {
    if (next_sampled_[i] == i) {
      cv::circle(img, cv::Point(path_2d_[i](1), path_2d_[i](0)), 0, cv::Scalar(0, 255, 255), 1);
    }
  }
  cv::imshow("Sampled", img);
  cv::waitKey(-1);
}

double Calibrator::CalcCurrentError() {
  int idx = -1;
  double current_error = 0;
  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) == -1) {
      continue;
    }
    idx++;
    auto warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
    current_error += another_calibrator_->dist_map_->Distance(warped(0), warped(1));
  }
  current_error /= (double) idx;
  return current_error;
}

void Calibrator::ShowCurrentSituation() {
  int idx = -1;
  cv::Mat img;
  cv::cvtColor(another_calibrator_->img_gray_, img, cv::COLOR_GRAY2BGR);
  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) == -1) {
      continue;
    }
    idx++;
    auto warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
    cv::circle(img, cv::Point(warped(1), warped(0)), 0, cv::Scalar(0, 255, 255), 1);
  }
  std::cout << "current_error: " << CalcCurrentError() << std::endl;
  cv::imwrite(std::string("current_") + std::to_string(sit_counter_++) + std::string(".png"), img);
  cv::imshow("Current", img);
  cv::waitKey(-1);
}

void Calibrator::SaveCurrentPoints() {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) == -1) {
      continue;
    }
    double focal_length = FocalLength();
    double z = GetDepth(i);
    double x = (path_2d_[i](1) - width_ / 2.0) * focal_length * z;
    double y = (path_2d_[i](0) - height_ / 2.0) * focal_length * z;
    points.emplace_back(z, x, y);
  }
  Utils::SavePointsAsPly("current.ply", points);
}

double Calibrator::GetDepth(int idx) {
  double depth;
  if (next_sampled_[idx] == idx) {
    depth = depth_[idx];
  }
  else {
    int a = past_sampled_[idx];
    int b = next_sampled_[idx];
    double bias = ((double) (idx - a)) / ((double) (b - a));
    depth = depth_[a] * (1.0 - bias) + depth_[b] * bias;
  }
  // return std::exp(depth);
  return depth;
}

/*------------------- TODO: Fast Warp -------------------------------

Eigen::Vector2d Calibrator::Warp(int i, int j, double depth) {
  double focal_length = FocalLength();
  // double focal_length = cam_paras_[6];
  // double focal_length = 1.0;
  double *cam_paras = biased_cam_paras_;
  double x = (j - width_  / 2.0) * focal_length * depth;
  double y = (i - height_ / 2.0) * focal_length * depth;
  double z = depth;

  // 3, 4, 5: rotation.
  Eigen::Vector3d w(cam_paras[3], cam_paras[4], cam_paras[5]);
  Eigen::Matrix3d t;
  if (w.norm() < 1e-6) {
    t.setIdentity();
  }
  else {
    t = Eigen::AngleAxisd(w.norm(), w / w.norm());
  }
  auto r_coord = t * Eigen::Vector3d(x, y, z);

  // 0, 1, 2: translation.
  double t_x = r_coord(0) + cam_paras[0];
  double t_y = r_coord(1) + cam_paras[1];
  double t_z = r_coord(2) + cam_paras[2];

  // std::cout << "z = " << r_coord(2) << std::endl;
  return Eigen::Vector2d(
    r_coord(1) / (r_coord(2) * focal_length) + (double) dist_map_->height_ / 2,
    r_coord(0) / (r_coord(2) * focal_length) + (double) dist_map_->width_ / 2
    );
}
*/

Eigen::Vector2d Calibrator::Warp(int i, int j, double depth) {
  double focal_length = FocalLength();
  Eigen::Vector3d pt_world = Pix2World(i, j, depth);
  Eigen::Vector3d pt_cam = another_calibrator_->World2Cam(pt_world);
  return Eigen::Vector2d(
    pt_cam(1) / (pt_cam(2) * focal_length) + (double) height_ / 2,
    pt_cam(0) / (pt_cam(2) * focal_length) + (double) width_ / 2);
}

Eigen::Vector3d Calibrator::Pix2World(int i, int j, double depth) {
  double focal_length = FocalLength();
  double x = (j - width_  / 2.0) * focal_length * depth;
  double y = (i - height_ / 2.0) * focal_length * depth;
  double z = depth;

  // 3, 4, 5: rotation.
  Eigen::Vector3d w(cam_paras_[3], cam_paras_[4], cam_paras_[5]);
  Eigen::Matrix3d t;
  if (w.norm() < 1e-6) {
    t.setIdentity();
  }
  else {
    t = Eigen::AngleAxisd(w.norm(), w / w.norm());
  }
  auto r_coord = t * Eigen::Vector3d(x, y, z);

  // 0, 1, 2: translation.
  double t_x = r_coord(0) + cam_paras_[0];
  double t_y = r_coord(1) + cam_paras_[1];
  double t_z = r_coord(2) + cam_paras_[2];

  return Eigen::Vector3d(t_x, t_y, t_z);
}


Eigen::Vector3d Calibrator::World2Cam(Eigen::Vector3d pt_world) {
  Eigen::Vector3d pt_cam(pt_world);
  pt_cam -= Eigen::Vector3d(cam_paras_[0], cam_paras_[1], cam_paras_[2]);

  Eigen::Vector3d w(-cam_paras_[3], -cam_paras_[4], -cam_paras_[5]);
  Eigen::Matrix3d t;
  if (w.norm() < 1e-6) {
    t.setIdentity();
  }
  else {
    t = Eigen::AngleAxisd(w.norm(), w / w.norm());
  }
  return t * pt_cam;
}

double Calibrator::FocalLength() {
  return std::exp(cam_paras_[6]);
}