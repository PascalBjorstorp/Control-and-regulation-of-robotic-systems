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
    cv::Mat _img, output;
    std::vector<cv::Point> _inters;
    std::vector<cv::Vec4i> _comp_path;
    std::vector<cv::Vec4i> _perps;
    std::vector<int> _perp_and_line_id;
    int _start_id = 0;
    int _min_length = 25;
    int _perp_length = 55;
    int _angle_limit = 5;

public:
    explicit lineDetecter(cv::Mat img);

    int calc_dist(cv::Point p1, cv::Point p2);

    int find_start_line();

    void sort_lines(const int start_id);

    int calc_line_len(const cv::Vec4i line);

    void eliminate_small_lines(int min_len);

    bool detect_overlap(const cv::Vec4i line1, const cv::Vec4i line2);

    void eliminate_overlap();

    void detect_lines();

    cv::Vec4i find_closest_pair(const cv::Vec4i line1, const cv::Vec4i line2);

    void conn_lines();

    void flip();

    void sort();

    int calc_angle(const cv::Vec4i line1, const cv::Vec4i line2);

    bool corner_between(const cv::Vec4i line1, const cv::Vec4i line2, const int angle_limit);

    cv::Vec4i extend_line(const cv::Vec4i line, const float scale);

    cv::Point calc_inter(const cv::Vec4i line1, const cv::Vec4i line2);

    void find_inters();

    void handle_inters();

    void perp_line();

    void sort_perps();

    void eliminate_long_lines(int max_len);

    static void onMouse(int event, int x, int y, int, void*);

    std::vector<cv::Point> get_inters() { return _inters; }
};

#endif // LINEDETECTER_H
