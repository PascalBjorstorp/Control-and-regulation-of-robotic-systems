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
    int _min_length = 110;
    int _perp_length = 30;
    int _angle_limit = 10;

public:
    explicit lineDetecter(cv::Mat img);

    int calc_dist(cv::Point p1, cv::Point p2);

    int find_start_line();

    cv::Vec4i find_end_line(std::vector<cv::Vec4i> lines, std::vector<cv::Vec3f> SS_points);

    cv::Point calc_inter(const cv::Vec4i line1, const cv::Vec4i line2);

    double calc_angle(const cv::Vec4i line);

    std::vector<cv::Vec4i> find_corner_lines(const cv::Vec4i current_line, const std::vector<cv::Vec4i> lines, double angle_threshold = 30.0);

    std::vector<cv::Vec4i> find_intersecting_lines(const cv::Vec4i cur_line, const std::vector<cv::Vec4i> grouped_lines);

    int return_id(cv::Vec4i line, std::vector<cv::Vec4i> lines);

    std::pair<double, int> corner_check(const cv::Vec4i trgt_line, const std::vector<cv::Vec4i> lines);

    cv::Vec4i orient(cv::Vec4i line, cv::Vec4i next_line);

    bool check_end(cv::Vec4i line, cv::Vec4i end_line);

    std::vector<cv::Vec4i> find_lines_with_similar_angle(const cv::Vec4i trgt_line, const std::vector<cv::Vec4i> lines, double angle_tolerance);

    std::vector<cv::Vec4i> within_prox(cv::Vec4i line, std::vector<cv::Vec4i> lines, int prox_val);

    cv::Vec4i extend_line(const cv::Vec4i line, const float scale);

    std::pair<cv::Vec4i, double> ex_closest_line(cv::Vec4i cur_line, std::vector<cv::Vec4i> lines, double scale_val);

    double closest_dist(cv::Vec4i line, std::vector<cv::Vec4i> lines);

    cv::Vec4i closest_line(cv::Vec4i line, std::vector<cv::Vec4i> lines);

    bool is_line_between(const cv::Vec4i line1, const cv::Vec4i line2, const cv::Vec4i candidate_line);

    cv::Vec4i find_line_between(const cv::Vec4i line1, const cv::Vec4i line2, const std::vector<cv::Vec4i> lines);

    int calc_line_len(const cv::Vec4i line);

    void eliminate_small_lines(int min_len);

    bool detect_overlap(const cv::Vec4i line1, const cv::Vec4i line2);

    void eliminate_overlap();

    void detect_lines();

    cv::Vec4i find_closest_pair(const cv::Vec4i line1, const cv::Vec4i line2);

    void conn_lines();

    void flip();

    void sort();

    bool corner_between(const cv::Vec4i line1, const cv::Vec4i line2, const int angle_limit);

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

    int calc_dist(cv::Point p1, cv::Point p2);
};

#endif // LINEDETECTER_H
