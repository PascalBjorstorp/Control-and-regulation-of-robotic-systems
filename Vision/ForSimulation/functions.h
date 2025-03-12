#pragma once

#include <iostream>
#include <vector>
#include <cmath>

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <X11/Xlib.h>

using namespace cv;

Point calc_ctrl_point(const Point start_line, const Point end_line, float offset_fac);

void bezier_curve(Mat& output, std::vector<Vec4i> lines, int res = 10, int range = 100);