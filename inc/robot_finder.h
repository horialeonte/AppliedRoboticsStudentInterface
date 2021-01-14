#pragma once
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>

#include <opencv2/calib3d.hpp>

#include <iostream>

bool robot_finder(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta);

