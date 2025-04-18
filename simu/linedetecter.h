#ifndef LINEDETECTER_H
#define LINEDETECTER_H

#include "imagehandler.h"
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class lineDetecter : public imageHandler
{
    std::vector<cv::Vec4i> _lines;
    std::vector<cv::Vec3f> _SS_points;
    cv::Mat _img, output;
    std::vector<cv::Point> _inters;
    std::vector<cv::Vec4i> _comp_path;

public:
    explicit lineDetecter();

    int calc_dist(cv::Point p1, cv::Point p2);

    int find_start_line();

    void sort_lines(const int start_id);

    int calc_line_len(const cv::Vec4i line);

    void eliminate_small_lines(int min_len);

    bool detect_overlap(const cv::Vec4i line1, const cv::Vec4i line2);

    void eliminate_overlap();

    void detect_lines();

    void sort();

    int calc_angle(const cv::Vec4i line1, const cv::Vec4i line2);

    bool corner_between(const cv::Vec4i line1, const cv::Vec4i line2, const int angle_limit);

    cv::Vec4i extend_line(const cv::Vec4i line, const float scale);

    cv::Point calc_inter(const cv::Vec4i line1, const cv::Vec4i line2);

    void find_inters();

    void handle_inters();

    std::vector<cv::Point> get_inters();
};

#endif // LINEDETECTER_H
