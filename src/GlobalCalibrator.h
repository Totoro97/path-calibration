#include <Eigen>
#include <vector>

#include "Calibrator.h"

class GlobalCalibrator {

public:
  std::vector<Calibrator *> calibrators_;

};