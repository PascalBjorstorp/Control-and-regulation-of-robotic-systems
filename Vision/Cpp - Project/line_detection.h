#pragma once

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <X11/Xlib.h>

using namespace cv;

Point get_scrn_param();

int calc_dist(Point p1, Point p2);

int find_start_line(std::vector<Vec4i> lines, std::vector<Vec3f> SS_points);

std::vector<Vec4i> sort_lines(const std::vector<Vec4i> lines, const int start_id);

int calc_line_len(const Vec4i line);

void eliminate_small_lines(std::vector<Vec4i>& line_Vec, int min_len);

std::vector<Vec4i> eliminate_overlap(const std::vector<Vec4i> line_Vec);

std::vector<Vec4i> detect_lines(const Mat img);

std::vector<Vec4i> sort(const Mat img, std::vector<Vec4i> lines, std::vector<Vec3f> SS_points);

int calc_angle(const Vec4i line1, const Vec4i line2);

bool corner_between(const Vec4i line1, const Vec4i line2, const int angle_limit);

Vec4i extend_line(const Vec4i line, const float scale);

Point calc_inter(const Vec4i line1, const Vec4i line2);

std::vector<Point> find_inters(const std::vector<Vec4i> lines);

std::vector<Vec4i> handle_inters(const std::vector<Vec4i> lines, const std::vector<Point> inters);