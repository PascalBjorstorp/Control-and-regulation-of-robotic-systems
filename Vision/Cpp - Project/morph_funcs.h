#pragma once

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

Mat perform_skeletonization(const Mat img);

void perform_dilate(Mat& img);