#pragma once

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

Mat isolate_red(const Mat img);

Mat isolate_blue(const Mat img);

void detect_SS(const Mat img, std::vector<Vec3f>& SS_points);

void rmv_SS(Mat& img, const std::vector<Vec3f> SS_points);