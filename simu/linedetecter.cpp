#include "linedetecter.h"

lineDetecter::lineDetecter():imageHandler(imread("/home/aksel/Documents/GitHub/Control-and-regulation-of-robotic-systems/Vision/Cpp - Project/test_img_ended.jpg", cv::IMREAD_COLOR)){

    _img = get_img();
    output = imread("/home/aksel/Documents/GitHub/Control-and-regulation-of-robotic-systems/Vision/Cpp - Project/test_img_ended.jpg", cv::IMREAD_COLOR);
    _SS_points = get_SS_points();

    detect_lines();
    sort();

    // Draw the found lines
    for(size_t i = 0; i < _lines.size(); i++){
        line(output, cv::Point(_lines[i][0], _lines[i][1]), cv::Point(_lines[i][2], _lines[i][3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }

    namedWindow("first", cv::WINDOW_NORMAL);
    imshow("first", output);

    find_inters();

    for(size_t i = 0; i < _inters.size(); i++){
        if(_inters[i] == cv::Point(-1,-1)){
            continue;
        }
        else{
            cv::circle(output, _inters[i], 30, cv::Scalar(255,0,0), -1);
        }
    }

    handle_inters();

    for(size_t i = 0; i < _comp_path.size(); i++){
        line(output, cv::Point(_comp_path[i][0], _comp_path[i][1]), cv::Point(_comp_path[i][2], _comp_path[i][3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }

    namedWindow("first", cv::WINDOW_NORMAL);
    imshow("first", output);

    cv::waitKey(0);
    cv::destroyAllWindows();
}

int lineDetecter::calc_dist(cv::Point p1, cv::Point p2){

    /*
        Description:
        Calculates the distance between two points, and returns the calculated value.
    */

    int x = p2.x - p1.x;
    int y = p2.y - p1.y;

    int dist = sqrt((x*x)+(y*y));

    return dist;
}

int lineDetecter::find_start_line(){

    /*
        Description:
        Calculates and returns the element id of the first line segment in vector lines.
        The first line segment is defined as the line segment closest to the start identifier,,,,,,,,,,,,,,,
    */

    int temp_val;

    cv::Point start_pnt(_SS_points[0][0], _SS_points[0][1]);

    int temp_id, temp_dist, min_dist = INT_MAX;

    for(size_t i = 0; i < _lines.size(); i++){
        temp_dist = calc_dist(start_pnt, cv::Point(_lines[i][0], _lines[i][1]));

        if(temp_dist < min_dist){
            min_dist = temp_dist;
            temp_id = i;
        }
    }

    return temp_val;
}

void lineDetecter::sort_lines(const int start_id){
    std::vector<cv::Vec4i> sorted_lines, temp_lines = _lines;
    cv::Point temp_pnt, temp_target_pnt;
    int temp_dist, temp_id, target_id = start_id;

    for(int i = 0; i < _lines.size(); i++){
        int min_dist = INT_MAX;
        temp_target_pnt = cv::Point(_lines[target_id][2], _lines[target_id][3]);

        for(size_t j = 0; j < temp_lines.size(); j++){
            temp_pnt = cv::Point(temp_lines[j][0], temp_lines[j][1]);
            temp_dist = calc_dist(temp_target_pnt, temp_pnt);

            if((min_dist > temp_dist)){
                min_dist = temp_dist;
                temp_id = j;
            }
        }

        for(size_t j = 0; j < _lines.size(); j++){
            if(_lines[j] == temp_lines[temp_id]){
                target_id = j;
                sorted_lines.push_back(_lines[j]);
                temp_lines.erase(temp_lines.begin() + temp_id);
                break;
            }
        }
    }

    _lines = sorted_lines;
}

int lineDetecter::calc_line_len(cv::Vec4i line){

    /*
        Description:
        Calculates the length of given line, and returns calculated value.
    */

    int x = line[2] - line[0];
    int y = line[3] - line[1];

    int len = sqrt((x*x)+(y*y));

    return len;
}

void lineDetecter::eliminate_small_lines(int min_len){

    /*
        Description:
        Erases lines with length shorter than min_len from the vector of lines/path.
    */

    int len;

    for(size_t i = 0; i < _lines.size(); i++){
        len = calc_line_len(_lines[i]);

        if(len <= min_len){
            _lines.erase(_lines.begin() + i);
            i--;
        }
    }
}

bool lineDetecter::detect_overlap(const cv::Vec4i line1, const cv::Vec4i line2) {
    // Check if the bounding boxes of the lines overlap
    int x1_min = std::min(line1[0], line1[2]);
    int x1_max = std::max(line1[0], line1[2]);
    int y1_min = std::min(line1[1], line1[3]);
    int y1_max = std::max(line1[1], line1[3]);

    int x2_min = std::min(line2[0], line2[2]);
    int x2_max = std::max(line2[0], line2[2]);
    int y2_min = std::min(line2[1], line2[3]);
    int y2_max = std::max(line2[1], line2[3]);

    return !(x1_max < x2_min || x2_max < x1_min || y1_max < y2_min || y2_max < y1_min);
}

void lineDetecter::eliminate_overlap() {
    std::vector<cv::Vec4i> result;
    std::vector<bool> to_remove(_lines.size(), false);

    for (size_t i = 0; i < _lines.size(); i++){
        if(to_remove[i]){
            continue;
        }

        for(size_t j = i + 1; j < _lines.size(); j++){
            if(to_remove[j]){
                continue;
            }

            if(detect_overlap(_lines[i], _lines[j])){
                int len1 = calc_line_len(_lines[i]);
                int len2 = calc_line_len(_lines[j]);

                if(len1 >= len2) {
                    to_remove[j] = true;
                }else {
                    to_remove[i] = true;
                    break;
                }
            }
        }
    }

    for(size_t i = 0; i < _lines.size(); i++){
        if(!to_remove[i]){
            result.push_back(_lines[i]);
        }
    }

    _lines = result;
}

void lineDetecter::detect_lines(){
    HoughLinesP(_img, _lines, 1, CV_PI/180, 50, 50, 10); // runs the actual detection

    eliminate_small_lines(180);

    eliminate_overlap();
}

void lineDetecter::sort(){

    int start_id = find_start_line();

    sort_lines(start_id);
}

int lineDetecter::calc_angle(const cv::Vec4i line1, const cv::Vec4i line2){

    cv::Point line1_start(line1[0], line1[1]);
    cv::Point line2_start(line2[0], line2[1]);

    cv::Point line1_end(line1[2], line1[3]);
    cv::Point line2_end(line2[2], line2[3]);

    cv::Point dir_line1 = line1_end - line1_start;
    cv::Point dir_line2 = line2_end - line2_start;

    float line1_len = calc_line_len(line1);
    float line2_len = calc_line_len(line2);

    float dot_prod = dir_line1.x * dir_line2.x + dir_line1.y * dir_line2.y;

    if((line1_len == 0) || (line2_len == 0)){
        return -1;
    }

    float cosTheta = dot_prod / (line1_len * line2_len);

    cosTheta = std::max(-1.0f, std::min(1.0f, cosTheta));

    float angle = std::acos(cosTheta); // Radians
    angle = angle * (180.0 / CV_PI); // Degrees

    std::cout << "angle: " << angle << std::endl;

    return angle;
}

bool lineDetecter::corner_between(const cv::Vec4i line1, const cv::Vec4i line2, const int angle_limit){
    if(calc_angle(line1, line2) > angle_limit){
        return true;
    }

    return false;
}

cv::Vec4i lineDetecter::extend_line(const cv::Vec4i line, const float scale){
    cv::Point line_start(line[0], line[1]);
    cv::Point line_end(line[2], line[3]);

    // Calculate the directional vector
    cv::Point dir = line_end - line_start;

    // Scale the directional vector
    cv::Point scaled_dir = cv::Point(dir.x * scale, dir.y * scale);

    // Calculate the new end point
    cv::Point new_start = line_start - scaled_dir;
    cv::Point new_end = line_end + scaled_dir;

    // Return the extended line
    return cv::Vec4i(new_start.x, new_start.y, new_end.x, new_end.y);
}

cv::Point lineDetecter::calc_inter(const cv::Vec4i line1, const cv::Vec4i line2) {
    cv::Point line1_start(line1[0], line1[1]);
    cv::Point line2_start(line2[0], line2[1]);

    cv::Point line1_end(line1[2], line1[3]);
    cv::Point line2_end(line2[2], line2[3]);

    // Calculate the direction vectors
    cv::Point dir_line1 = line1_end - line1_start;
    cv::Point dir_line2 = line2_end - line2_start;

    float num = (((line2_start.x - line1_start.x) * (dir_line2.y)) - ((line2_start.y - line1_start.y) * (dir_line2.x)));
    float den = (dir_line1.x * dir_line2.y) - (dir_line1.y * dir_line2.x);

    if(den == 0){
        return cv::Point(-1, -1);
    }

    float t = num / den;

    return cv::Point(line1_start.x + t * dir_line1.x, line1_start.y + t * dir_line1.y);
}

void lineDetecter::find_inters(){
    cv::Point inter;
    std::vector<cv::Vec4i> ext_lines = _lines;
    int angle_limit = 15;

    for(size_t i = 0; i < _lines.size()-1; i++){
        if(corner_between(_lines[i], _lines[i+1], angle_limit)){
            ext_lines[i] = extend_line(_lines[i], 2);
            ext_lines[i+1] = extend_line(_lines[i+1], 2);
            inter = calc_inter(_lines[i], _lines[i+1]);
            _inters.push_back(inter);
        }
        else{
            _inters.push_back(cv::Point(-1,-1));
        }
    }
}

void lineDetecter::handle_inters(){
    std::vector<cv::Vec4i> temp_lines = _lines;

    for(size_t i = 0; i < _inters.size(); i++){
        if(_inters[i] == cv::Point(-1,-1)){
            continue;
        }
        else{

            cv::Point line1_pnt1(temp_lines[i][0], temp_lines[i][1]);
            cv::Point line1_pnt2(temp_lines[i][2], temp_lines[i][3]);

            cv::Point line2_pnt1(temp_lines[i+1][0], temp_lines[i+1][1]);
            cv::Point line2_pnt2(temp_lines[i+1][2], temp_lines[i+1][3]);

            if(calc_dist(line1_pnt1, _inters[i]) < calc_dist(line1_pnt2, _inters[i])){
                temp_lines[i][0] = _inters[i].x;
                temp_lines[i][1] = _inters[i].y;
            }
            else{
                temp_lines[i][2] = _inters[i].x;
                temp_lines[i][3] = _inters[i].y;
            }

            if(calc_dist(line2_pnt1, _inters[i]) < calc_dist(line2_pnt2, _inters[i])){
                temp_lines[i+1][0] = _inters[i].x;
                temp_lines[i+1][1] = _inters[i].y;
            }
            else{
                temp_lines[i+1][2] = _inters[i].x;
                temp_lines[i+1][3] = _inters[i].y;
            }
        }
    }

    _comp_path = temp_lines;
}

std::vector<cv::Point> lineDetecter::get_inters(){
    std::vector<cv::Point> temp;
    temp.push_back(cv::Point(_SS_points[0][0], _SS_points[0][1]));
    for(size_t i = 0; i < _inters.size(); i++){
        if(_inters[i] == cv::Point(-1,-1)){
            continue;
        }
        else{
            temp.push_back(_inters[i]);
        }
    }

    temp.push_back(cv::Point(_SS_points[1][0], _SS_points[1][1]));

    return temp;
}
