#include "GlobalCalibrator.h"
#include "Thinning.h"

GlobalCalibrator::GlobalCalibrator(std::string dir_name, int num_frame): num_frame_(num_frame) {
  for (int i = 0; i < num_frame; i++) {
    auto img = cv::imread(dir_name + "/" + std::to_string(i) + ".png");
    cv::Mat img_gray(img.rows, img.cols, CV_8UC1);
    for (int i = 0; i < img.rows * img.cols; i++) {
      if (img.data[i * 3] == img.data[0] &&
          img.data[i * 3 + 1] == img.data[1] &&
          img.data[i * 3 + 2] == img.data[2]) {
        img_gray.data[i] = 0;
      }
      else
        img_gray.data[i] = (uchar) 255;
    }

    thinning(img_gray, img_gray);
    calibrators_.push_back(new Calibrator(img_gray, i));
  }
}

GlobalCalibrator::~GlobalCalibrator() {
  for (auto calibrator : calibrators_) {
    delete(calibrator);
  }
}

void GlobalCalibrator::Run() {
  // One way
  /*for (int l = 1; l < num_frame_; l++) {
    for (int i = 0; i + l < num_frame_; i++) {
      int j = i + l;
      auto cali_0 = calibrators_[i];
      auto cali_1 = calibrators_[j];
      // TODO: Hard code here.
      for (int cnt = 0; cnt < 3; cnt++) {
        cali_1->InitializeParameters(cali_0);
        cali_1->Run(20);
        cali_0->InitializeParameters(cali_1);
        cali_0->Run(20);
      }
    }
  }*/
  // Another way
  for (int i = 0; i < num_frame_; i++) {
    for (int T = 5; T >= 0; T--) {
      for (int k = i; k >= 0; k--) {
        auto cali_1 = calibrators_[i];
        for (int j = k - 1; j >= 0; j--) {
          auto cali_0 = calibrators_[j];
          for (int cnt = 0; cnt < 1; cnt++) {
            cali_1->InitializeParameters(cali_0);
            cali_1->Run(20);
            cali_0->InitializeParameters(cali_1);
            cali_0->Run(20);
          }
        }
      }
    }
  }
}