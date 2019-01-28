//
// Created by aska on 19-1-28.
//

#include "Algorithms.h"
// std
#include <algorithm>
#include <functional>
#include <queue>
#include <set>

void Algo::GetPathPoints(cv::Mat &mat, std::vector<Eigen::Vector2i> &path_2d) {
  int height = mat.rows;
  int width = mat.cols;
  if (mat.channels() != 1) {
    std::cout << "Error: channels not eq 1" << std::endl;
    return;
  }

  uint8_t *data_ptr = mat.data;
  auto IsNeighbor = [height, width, data_ptr](int x, int y, int bias_x, int bias_y) {
    int a = x + bias_x;
    int b = y + bias_y;
    if (a < 0 || a >= height || b < 0 || b >= width) {
      return false;
    }
    if (data_ptr[a * width + b] != 255) {
      return false;
    }
    if (std::abs(bias_x) + std::abs(bias_y) <= 1)
      return true;
    a = x + bias_x;
    b = y;
    if (a >= 0 && a < height && b >= 0 && b < width && data_ptr[a * width + b] == 255) {
      return false;
    }
    a = x;
    b = y + bias_y;
    if (a >= 0 && a < height && b >= 0 && b < width && data_ptr[a * width + b] == 255) {
      return false;
    }
    return true;
  };

  int num_ends = 0;
  int x_path_begin, y_path_begin;
  std::set<std::pair<int, int> > odd_points;
  auto res = new int[height * width * 9];

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      if (data_ptr[i * width + j] != 255) {
        continue;
      }
      int du = 0;
      for (int a = -1; a <= 1; a++) {
        for (int b = -1; b <= 1; b++) {
          if ((a || b) && IsNeighbor(i, j, a, b)) {
            du++;
            res[(i * width + j) * 9 + (a + 1) * 3 + b + 1]++;
          }
        }
      }

      // Debug
      if (du < 1) {
        std::cout << "Error: Isolated Points." << std::endl;
        return;
      }
      if (du == 1) {
        num_ends++;
        x_path_begin = i;
        y_path_begin = j;
      }
      else if ((du & 1) == 1) {
        odd_points.emplace(i, j);
      }
    }
  }

  if (num_ends != 2) {
    std::cout << "Error: Not a Path: " << num_ends << "." << std::endl;
    return;
  }

  // Add more edges to guarantee Euler Path.
  auto vis = new int[height * width];
  int tick_counter = 0;

  if ((odd_points.size() & 1) == 1) {
    std::cout << "Error: Odd Odd." << std::endl;
    return;
  }
  while (!odd_points.empty()) {
    tick_counter++;
    auto pr = *odd_points.begin();
    odd_points.erase(odd_points.begin());
    int x_begin = pr.first;
    int y_begin = pr.second;
    std::vector<std::pair<int, int> > que;
    std::vector<int> pas;
    que.emplace_back(x_begin, y_begin);
    pas.push_back(-1);
    bool found = false;

    for (int idx = 0; !found && idx < que.size(); idx++) {
      int x = que[idx].first, y = que[idx].second;
      for (int bias_x = -1; bias_x <= 1 && !found; bias_x++) {
        for (int bias_y = -1; bias_y <= 1 && !found; bias_y++) {
          if ((!bias_x && !bias_y) || IsNeighbor(x, y, bias_x, bias_y)) {
            continue;
          }
          int a = x + bias_x, b = y + bias_y;
          if (vis[a * width + b] == tick_counter)
            continue;
          vis[a * width + b] = tick_counter;
          que.emplace_back(a, b);
          pas.push_back(idx);

          if (odd_points.find(std::make_pair(a, b)) != odd_points.end()) {
            odd_points.erase(std::make_pair(a, b));
            found = true;
            for (int i = int(que.size()) - 1; i > 0; i = pas[i]) {
              int bias_a = que[pas[i]].first - a;
              int bias_b = que[pas[i]].second - b;
              res[(a * width + b) * 9 + (bias_a + 1) * 3 + bias_b + 1]++;
              res[((a + bias_a) * width + b + bias_b) * 9 + (-bias_a + 1) * 3 - bias_b + 1]++;
              a += bias_a;
              b += bias_b;
            }
          }
        }
      }
    };
    if (!found) {
      std::cout << "Error: Another odd point not found." << std::endl;
    }
  }

  delete[](vis);

  std::function<void(int, int)> FindEulerPath =
    [&FindEulerPath, height, width, data_ptr, &path_2d, &IsNeighbor, res](int x, int y) {
    for (int bias_x = -1; bias_x <= 1; bias_x++) {
      for (int bias_y = -1; bias_y <= 1; bias_y++) {
        if (!bias_x && !bias_y)
          continue;
        if (!IsNeighbor(x, y, bias_x, bias_y))
          continue;
        if (res[(x * width + y) * 9 + (bias_x + 1) * 3 + bias_y + 1] <= 0)
          continue;
        res[(x * width + y) * 9 + (bias_x + 1) * 3 + bias_y + 1]--;
        res[((x + bias_x) * width + y + bias_y) * 9 + (-bias_x + 1) * 3 - bias_y + 1]--;
        FindEulerPath(x + bias_x, y + bias_y);
      }
    }
    path_2d.emplace_back(x, y);
  };

  FindEulerPath(x_path_begin, y_path_begin);
  delete[](res);
}
