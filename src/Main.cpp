//
// Created by aska on 19-2-12.
//

#include "GlobalCalibrator.h"

int main() {
  auto global_calibrator = new GlobalCalibrator("/home/aska/Data", 4);
  global_calibrator->Run();
  delete(global_calibrator);
  return 0;
}