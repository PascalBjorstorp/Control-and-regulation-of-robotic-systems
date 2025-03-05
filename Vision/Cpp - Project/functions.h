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

Point get_scrn_param();

int calc_line_len(Vec4i line);

void eliminate_small_lines(std::vector<Vec4i>& line_Vec, int min_len);

int find_start(const std::vector<Vec4i> line_Vec);

/*std::vector<Vec4i> sort_lines(std::vector<Vec4i> line_Vec);*/

Point calc_ctrl_point(const Point start_line, const Point end_line, float offset_fac);

void bezier_curve(Mat& output, std::vector<Vec4i> lines, int res = 10, int range = 100);

int calc_angle(const Vec4i line1, const Vec4i line2);

Point line_inter(const Vec4i line1, const Vec4i line2);

std::vector<Point> find_all_inters(const std::vector<Vec4i> lines);