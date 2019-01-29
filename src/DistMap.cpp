//
// Created by aska on 19-1-29.
//

#include "DistMap.h"
#include <algorithm>
#include <fstream>
#include <cstring>
#include <iostream>

DistMap::DistMap(const cv::Mat &img, bool calc_dist_map, std::string map_path) {
  std::cout << "DistriMap Reconstruction: Begin" << std::endl;
  height_ = img.rows;
  width_ = img.cols;
  dist_map_ = new double[height_ * width_];
  if (calc_dist_map) {
    CalcDistMap(img, map_path);
  }
  else {
    LoadDistMap(map_path);
  }
}

DistMap::~DistMap() {
  delete(dist_map_);
}

void DistMap::CalcDistMap(const cv::Mat &img, std::string map_path) {
  std::cout << "CalcMap: Begin" << std::endl;
  auto dist_map = dist_map_;
  std::fill_n(dist_map, width_ * height_, 1e9);
  auto dis_cmp = [dist_map, this](std::pair<int, int> a, std::pair<int, int> b) {
    double dist_a = dist_map[a.first * this->width_ + a.second];
    double dist_b = dist_map[b.first * this->width_ + b.second];
    if (dist_a < dist_b - 1e-8)
      return true;
    if (dist_a > dist_b + 1e-8)
      return false;
    return (a.first * this->width_ + a.second < b.first * this->width_ + b.second);
  };

  std::set<std::pair<int, int>, decltype(dis_cmp)> que(dis_cmp);
  que.clear();
  // Calc
  for (int i = 0; i < height_; i++) {
    for (int j = 0; j < width_; j++) {
      if (!IsInside(img, i, j)) {
        continue;
      }
      dist_map[i * width_ + j] = 0;
      que.insert(std::make_pair(i, j));
    }
  }

  std::vector<std::pair<int, int> > biases;
  const int lim = 5;
  std::function<int(int, int)> gcd = [&gcd](int a, int b) {
    return b ? gcd(b, a % b) : a;
  };
  for (int i = -lim; i <= lim; i++)
    for (int j = -lim; j <= lim; j++) {
      if (gcd(std::abs(i), std::abs(j)) == 1) {
        biases.push_back(std::make_pair(i, j));
      }
    }
  std::cout << "biased size = " << biases.size() << std::endl;
  while (!que.empty()) {
    auto pix = *que.begin();
    que.erase(que.begin());
    // if (dist_map[pix.first * width_ + pix.second] > 50) {
    //  break;
    // }
    double current_dis = dist_map[pix.first * width_ + pix.second];
    for (const auto &bias : biases) {
      int r = pix.first + bias.first;
      int c = pix.second + bias.second;
      if (r < 0 || r >= height_ || c < 0 || c >= width_) {
        continue;
      }
      double new_dis = current_dis + std::sqrt(bias.first * bias.first + bias.second * bias.second);
      if (dist_map[r * width_ + c] < new_dis) {
        continue;
      }
      que.erase(std::make_pair(r, c));
      dist_map[r * width_ + c] = new_dis;
      que.insert(std::make_pair(r, c));
    }
  }

  // Save bin
  std::ofstream tmp_file;
  tmp_file.open(map_path.c_str(), std::ios::binary | std::ios::ate);
  tmp_file.write((const char *) dist_map_, width_ * height_ * sizeof(double));
  tmp_file.close();

  std::cout << "CalcDistMap: End" << std::endl;
}

void DistMap::LoadDistMap(std::string map_path) {
  std::ifstream map_stream(map_path.c_str(), std::ios::binary | std::ios::ate);
  map_stream.seekg(0, std::ios::beg);
  map_stream.read((char *)(dist_map_), width_ * height_ * sizeof(double));
  map_stream.close();
}

bool DistMap::IsInside(const cv::Mat &img, int i, int j) {
  auto *img_ptr = (uint8_t*) img.data;
  return (static_cast<int>(img_ptr[(i * width_ + j)]) == 255);
}

double DistMap::Distance(double a, double b) {
  const double eps = 1e-7;
  if (a > eps && a < height_ - 1 - eps && b > eps && b < width_ - 1 - eps) {
    int a_i = static_cast<int>(a);
    int b_i = static_cast<int>(b);
    double bias_a = a - a_i;
    double bias_b = b - b_i;
    int idx = a_i * width_ + b_i;
    double dist_1 = dist_map_[idx]     * (1.0 - bias_a) + dist_map_[idx + width_    ] * bias_a;
    double dist_2 = dist_map_[idx + 1] * (1.0 - bias_a) + dist_map_[idx + width_ + 1] * bias_a;
    return dist_1 * (1.0 - bias_b) + dist_2 * bias_b;
  }
  else {
    int a_i = (int) std::min(std::max(1e-7, a), height_ - 1.0);
    int b_i = (int) std::min(std::max(1e-7, b), width_ - 1.0);
    return dist_map_[a_i * width_ + b_i] + std::abs(a_i - a) + std::abs(b_i - b);
  }
}

void DistMap::ShowDistMap() {
  cv::Mat img(height_, width_, CV_8UC1);
  for (int i = 0; i < height_ * width_; i++) {
    img.data[i] = (uchar) std::min(255.0, dist_map_[i]);
  }
  cv::imshow("dist map", img);
  cv::waitKey(-1);
  cv::destroyAllWindows();
}