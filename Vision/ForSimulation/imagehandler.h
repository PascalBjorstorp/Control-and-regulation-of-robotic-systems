#ifndef IMAGEHANDLER_H
#define IMAGEHANDLER_H

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

class imageHandler
{
    cv::Mat _img, _red_img, _blue_img, _green_img, output;
    std::vector<cv::Vec3f> _SS_points;

public:
    explicit imageHandler(cv::Mat img);

    void isolate_red();
    void isolate_blue();
    void isolate_green();
    void detect_SS(cv::Mat *img);
    void rmv_SS();
    void perform_skeletonization();
    void perform_dilate();

    const cv::Mat get_img() { return _img; }
    const cv::Mat get_red_img() { return _red_img; }
    const cv::Mat get_blue_img() { return _blue_img; }
    const std::vector<cv::Vec3f> get_SS_points() { return _SS_points; }
};

#endif // IMAGEHANDLER_H
