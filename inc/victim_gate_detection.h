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
cv::Mat positioning(cv::Mat src, double angle);
bool victim_gate_detection(const cv::Mat& img_in, const double scale,  std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate);
