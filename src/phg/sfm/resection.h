#pragma once

#include <phg/core/calibration.h>

#include <opencv2/core.hpp>

namespace phg {

cv::Matx34d findCameraMatrix(const Calibration &calib, const std::vector<cv::Vec3d> &X, const std::vector<cv::Vec2d> &x);

}
