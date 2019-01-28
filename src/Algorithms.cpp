//
// Created by aska on 19-1-28.
//

#include "Algorithms.h"
// std
#include <algorithm>
#include <functional>

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
  int x_begin, y_begin;
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
        x_begin = i;
        y_begin = j;
      }
    }
  }

  if (num_ends != 2) {
    std::cout << "Error: Not a Path: " << num_ends << "." << std::endl;
    return;
  }

  auto vis = new bool[height * width * 9];
  std::function<void(int, int)> FindEulerPath =
    [&FindEulerPath, height, width, data_ptr, &path_2d, &IsNeighbor, vis](int x, int y) {
    for (int bias_x = -1; bias_x <= 1; bias_x++) {
      for (int bias_y = -1; bias_y <= 1; bias_y++) {
        if (!bias_x && !bias_y)
          continue;
        if (!IsNeighbor(x, y, bias_x, bias_y))
          continue;
        if (vis[(x * width + y) * 9 + (bias_x + 1) * 3 + bias_y + 1])
          continue;
        vis[(x * width + y) * 9 + (bias_x + 1) * 3 + bias_y + 1] = true;
        vis[((x + bias_x) * width + y + bias_y) * 9 + (-bias_x + 1) * 3 - bias_y + 1] = true;
        FindEulerPath(x + bias_x, y + bias_y);
      }
      path_2d.emplace_back(x, y);
      return;
    }
  };

  FindEulerPath(x_begin, y_begin);
  delete[](vis);
}
