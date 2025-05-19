#include "linedetecter.h"

<<<<<<< Updated upstream
<<<<<<< Updated upstream
std::vector<cv::Vec3f> _SS_points;

lineDetecter::lineDetecter(cv::Mat img): imageHandler(img) {

    _img = get_img();
    output = img.clone();

    cv::namedWindow("SS image", cv::WINDOW_NORMAL);
    cv::setMouseCallback("SS image", lineDetecter::onMouse, nullptr);

    std::cout << _SS_points.size() << std::endl;

    while (_SS_points.size() < 2) {
        cv::Mat display = output.clone();
        for (const auto& pt : _SS_points)
            cv::circle(display, cv::Point(pt[0], pt[1]), 5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("SS image", display);
        if (cv::waitKey(10) == 27) break; // ESC to exit
    }
    cv::destroyAllWindows();

    cvtColor(_img, output, cv::COLOR_GRAY2BGR);

    //_SS_points = get_SS_points();
=======
=======
>>>>>>> Stashed changes
using namespace cv;

lineDetecter::lineDetecter():imageHandler(imread("/home/aksel/Documents/GitHub/Control-and-regulation-of-robotic-systems/Vision/Cpp - Project/test_img_2.jpg", cv::IMREAD_COLOR)){

    _img = get_img();
    output = imread("/home/aksel/Documents/GitHub/Control-and-regulation-of-robotic-systems/Vision/Cpp - Project/test_img_2.jpg", cv::IMREAD_COLOR);
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes

    detect_lines();
    sort();

<<<<<<< Updated upstream
<<<<<<< Updated upstream
    find_inters();

    handle_inters();

    for(size_t i = 0; i < _inters.size()-1; i++){
        if(_inters[i] == cv::Point(-1,-1)){
            continue;
        }
        else{
            circle(output, _inters[i], 30, cv::Scalar(0,0,255), -1);
            line(output, cv::Point(_inters[i].x, _inters[i].y), cv::Point(_inters[i+1].x, _inters[i+1].y), cv::Scalar(0,255,0), 5, cv::LINE_AA);
        }
=======
=======
>>>>>>> Stashed changes
    // Draw the found lines
    for(size_t i = 0; i < _lines.size(); i++){
        line(output, cv::Point(_comp_path[i][0], _comp_path[i][1]), cv::Point(_comp_path[i][2], _comp_path[i][3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
>>>>>>> Stashed changes
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

    cv::Point start_pnt(_SS_points[0][0], _SS_points[0][1]);

    int temp_id, temp_dist1, temp_dist2, min_dist = INT_MAX;
    bool toFlip = false;

    for(size_t i = 0; i < _lines.size(); i++){
        temp_dist1 = calc_dist(start_pnt, cv::Point(_lines[i][0], _lines[i][1]));
        temp_dist2 = calc_dist(start_pnt, cv::Point(_lines[i][2], _lines[i][3]));

        if(temp_dist1 < min_dist){
            min_dist = temp_dist1;
            temp_id = i;
            toFlip = false;
        }
        if(temp_dist2 < min_dist){
            min_dist = temp_dist2;
            temp_id = i;
            toFlip = true;
        }
    }

    if(toFlip){
        // Flip the line
        cv::Point temp;

        temp.x = _lines[temp_id][0];
        temp.y = _lines[temp_id][1];

        _lines[temp_id][0] = _lines[temp_id][2];
        _lines[temp_id][1] = _lines[temp_id][3];

        _lines[temp_id][2] = temp.x;
        _lines[temp_id][3] = temp.y;
    }

    return temp_id;
}

<<<<<<< Updated upstream
<<<<<<< Updated upstream
void lineDetecter::sort_lines(const int start_id){
    std::vector<cv::Vec4i> sorted_lines, temp_lines = _lines;
    std::vector<cv::Vec4i> sorted_perps;
    cv::Point temp_pnt, temp_target_pnt;
    int temp_dist, temp_id, target_id = start_id;
    bool toFlip = false;

    sorted_lines.push_back(_lines[start_id]);
    sorted_perps.push_back(_perps[start_id]);
    temp_lines.erase(temp_lines.begin() + start_id);

    for(int i = 0; i < _lines.size(); i++){
        int min_dist = INT_MAX;
        temp_target_pnt = cv::Point(_lines[target_id][2], _lines[target_id][3]);
        toFlip = false;

        for(size_t j = 0; j < temp_lines.size(); j++){
            temp_pnt = cv::Point(temp_lines[j][0], temp_lines[j][1]);
            temp_dist = calc_dist(temp_target_pnt, temp_pnt);

            if((min_dist > temp_dist)){
                min_dist = temp_dist;
                temp_id = j;
                toFlip = false;
            }

            temp_pnt = cv::Point(temp_lines[j][2], temp_lines[j][3]);
            temp_dist = calc_dist(temp_target_pnt, temp_pnt);

            if((min_dist > temp_dist)){
                min_dist = temp_dist;
                temp_id = j;
                toFlip = true;
            }
        }

        for(size_t j = 0; j < _lines.size(); j++){
            if(_lines[j] == temp_lines[temp_id]){
                target_id = j;
                if(toFlip){
                    // Flip the line
                    cv::Point temp;

                    temp.x = _lines[j][0];
                    temp.y = _lines[j][1];

                    _lines[j][0] = _lines[j][2];
                    _lines[j][1] = _lines[j][3];

                    _lines[j][2] = temp.x;
                    _lines[j][3] = temp.y;
                }
                sorted_lines.push_back(_lines[j]);
                sorted_perps.push_back(_perps[j]);
                temp_lines.erase(temp_lines.begin() + temp_id);


                break;
            }
        }
    }
    _perps = sorted_perps;
    _lines = sorted_lines;
}

=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
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

    eliminate_small_lines(_min_length);

    eliminate_overlap();
}

<<<<<<< Updated upstream
<<<<<<< Updated upstream
cv::Vec4i lineDetecter::find_closest_pair(const cv::Vec4i line1, const cv::Vec4i line2){

    int min_dist = INT_MAX;
    float dist;
    cv::Vec4i closest_pair;

    // Iterate through all possible pairs of points
    for (size_t i = 0; i < 3; i = i + 2) {

        dist = calc_dist(cv::Point(line1[0], line1[1]), cv::Point(line2[i], line2[i+1]));
        if(dist < min_dist){
            min_dist = dist;
            closest_pair = cv::Vec4i(line1[0], line1[1], line2[i], line2[i+1]);
        }

        dist = calc_dist(cv::Point(line1[2], line1[3]), cv::Point(line2[i], line2[i+1]));
        if(dist < min_dist){
            min_dist = dist;
            closest_pair = cv::Vec4i(line1[2], line1[3], line2[i], line2[i+1]);
        }
    }

    return closest_pair;
}

void lineDetecter::conn_lines(){
    cv::Vec4i close_points;


    cv::Point start_pnt((_SS_points[0][0]), _SS_points[0][1]);
    cv::Point end_pnt((_SS_points[1][0]), _SS_points[1][1]);

    std::cout << "size: " << _lines.size() << std::endl;

    for(size_t i = 0; i < _lines.size(); i++){

        cv::Point line_pnt1((_lines[i][0]), _lines[i][1]);
        cv::Point line_pnt2((_lines[i][2]), _lines[i][3]);

        if(i == 0){

            if(calc_dist(line_pnt1, start_pnt) < calc_dist(line_pnt2, start_pnt)){
                _lines[i][0] = start_pnt.x;
                _lines[i][1] = start_pnt.y;
            }
            else{
                _lines[i][2] = start_pnt.x;
                _lines[i][3] = start_pnt.y;
            }
        }
        else if(i == _lines.size()-1){

            if(calc_dist(line_pnt1, end_pnt) < calc_dist(line_pnt2, end_pnt)){
                _lines[i][0] = end_pnt.x;
                _lines[i][1] = end_pnt.y;
            }
            else{
                _lines[i][2] = end_pnt.x;
                _lines[i][3] = end_pnt.y;
            }
        }

        if(!corner_between(_lines[i], _lines[i+1], 15)){

            std::cout << "found no corner between, i: " << i << std::endl;

            close_points = find_closest_pair(_lines[i], _lines[i+1]);

            for(size_t j = 0; j < 3; j = j + 2){
                if((_lines[i][j] == close_points[0]) && (_lines[i][j+1] == close_points[1])){
                    _lines[i][j] = close_points[2];
                    _lines[i][j+1] = close_points[3];
                }
            }
        }
    }
}

void lineDetecter::flip(){
    cv::Point temp;

    for(size_t i = 1; i < _lines.size(); i++){
        float dist1 = calc_dist(cv::Point(_lines[i][0], _lines[i][1]), cv::Point(_lines[i - 1][2], _lines[i - 1][3]));
        float dist2 = calc_dist(cv::Point(_lines[i][2], _lines[i][3]), cv::Point(_lines[i - 1][2], _lines[i - 1][3]));

        if(dist1 < dist2){
            continue;
        }
        else{
            temp.x = _lines[i][0];
            temp.y = _lines[i][1];

            _lines[i][0] = _lines[i][2];
            _lines[i][1] = _lines[i][3];

            _lines[i][2] = temp.x;
            _lines[i][3] = temp.y;
        }
    }
}

void lineDetecter::sort(){

    perp_line();

    sort_perps();

    _start_id = find_start_line();

    sort_lines(_start_id);

    conn_lines();

    flip();

    eliminate_long_lines(700);
}
=======
Vec4i lineDetecter::find_end_line(std::vector<Vec4i> lines, std::vector<Vec3f> SS_points){

    /*
        Description:
        Calculates and returns the element id of the first line segment in vector lines.
        The first line segment is defined as the line segment closest to the start identifier
    */
    
    Point end_pnt(SS_points[1][0], SS_points[1][1]);

    int temp_id, dist1, dist2, min_dist = INT_MAX;
>>>>>>> Stashed changes

    for(size_t i = 0; i < lines.size(); i++){
        dist1 = calc_dist(end_pnt, Point(lines[i][0], lines[i][1]));
        dist2 = calc_dist(end_pnt, Point(lines[i][2], lines[i][3]));

        if((dist1 < min_dist) || (dist2 < min_dist)){
            min_dist = (dist1 < dist2 ? dist1 : dist2);
            temp_id = i;
        }
    }

    return lines[temp_id];
}

int lineDetecter::calc_dist(Point p1, Point p2){
    
    /*
        Description:
        Calculates the distance between two points, and returns the calculated value.
    */

    int x = p2.x - p1.x;
    int y = p2.y - p1.y;

=======
Vec4i lineDetecter::find_end_line(std::vector<Vec4i> lines, std::vector<Vec3f> SS_points){

    /*
        Description:
        Calculates and returns the element id of the first line segment in vector lines.
        The first line segment is defined as the line segment closest to the start identifier
    */
    
    Point end_pnt(SS_points[1][0], SS_points[1][1]);

    int temp_id, dist1, dist2, min_dist = INT_MAX;

    for(size_t i = 0; i < lines.size(); i++){
        dist1 = calc_dist(end_pnt, Point(lines[i][0], lines[i][1]));
        dist2 = calc_dist(end_pnt, Point(lines[i][2], lines[i][3]));

        if((dist1 < min_dist) || (dist2 < min_dist)){
            min_dist = (dist1 < dist2 ? dist1 : dist2);
            temp_id = i;
        }
    }

    return lines[temp_id];
}

int lineDetecter::calc_dist(Point p1, Point p2){
    
    /*
        Description:
        Calculates the distance between two points, and returns the calculated value.
    */

    int x = p2.x - p1.x;
    int y = p2.y - p1.y;

>>>>>>> Stashed changes
    int dist = sqrt((x*x)+(y*y));

    return dist;
}

Point lineDetecter::calc_inter(const Vec4i line1, const Vec4i line2){
    Point line1_start(line1[0], line1[1]);
    Point line2_start(line2[0], line2[1]);

    Point line1_end(line1[2], line1[3]);
    Point line2_end(line2[2], line2[3]);

    // Calculate the direction vectors
    Point dir_line1 = line1_end - line1_start;
    Point dir_line2 = line2_end - line2_start;

    float num = (((line2_start.x - line1_start.x) * (dir_line2.y)) - ((line2_start.y - line1_start.y) * (dir_line2.x)));
    float den = (dir_line1.x * dir_line2.y) - (dir_line1.y * dir_line2.x);
    
    if(den == 0){
        return Point(-1, -1);
    }

    float t = num / den;

    return Point(line1_start.x + t * dir_line1.x, line1_start.y + t * dir_line1.y);
}

double lineDetecter::calc_angle(const Vec4i line){
    Point start(line[0], line[1]);
    Point end(line[2], line[3]);

    // Calculate the direction vector
    Point dir = end - start;

    // Calculate the angle in degrees
    double angle = atan2(dir.y, dir.x) * (180.0 / CV_PI);

    // Normalize the angle to [0, 180)
    if (angle < 0) {
        angle += 180.0;
    }

    return angle;
}

<<<<<<< Updated upstream
<<<<<<< Updated upstream
bool lineDetecter::corner_between(const cv::Vec4i line1, const cv::Vec4i line2, const int angle_limit){
    float angle = calc_angle(line1, line2);

    if((angle > angle_limit) && (angle < (180 - angle_limit))){
=======
=======
>>>>>>> Stashed changes
std::vector<Vec4i> lineDetecter::find_corner_lines(const Vec4i current_line, const std::vector<Vec4i> lines, double angle_threshold){
    std::vector<Vec4i> corner_lines;

    // Calculate the angle of the current line
    double current_angle = calc_angle(current_line);

    for (const auto& line : lines) {
        // Calculate the angle of the current line in the vector
        double line_angle = calc_angle(line);

        // Calculate the absolute difference in angles
        double angle_diff = std::abs(current_angle - line_angle);

        // Normalize the angle difference to [0, 180)
        if (angle_diff > 180.0) {
            angle_diff = 360.0 - angle_diff;
        }

        // Check if the angle difference exceeds the threshold
        if (angle_diff > angle_threshold) {
            corner_lines.push_back(line);
        }
    }

    return corner_lines;
}

std::vector<Vec4i> lineDetecter::find_intersecting_lines(const Vec4i cur_line, const std::vector<Vec4i> grouped_lines){
    std::vector<Vec4i> intersecting_lines;

    for (const auto& line : grouped_lines) {
        // Calculate the intersection point
        Point intersection = calc_inter(cur_line, line);

        // Check if the intersection point is valid

        if((intersection != Point(-1,-1)) && (1000 > intersection.x) && (intersection.x > 0) && (1000 > intersection.y) && (intersection.y > 0)){
            if(cur_line != line){
                intersecting_lines.push_back(line);
            }
        }
    }

    return intersecting_lines;
}

int lineDetecter::return_id(Vec4i line, std::vector<Vec4i> lines){
    int id = -1;

    for(size_t i = 0; i < lines.size(); i++){
        if((line == lines[i]) || (lines[i] == Vec4i(line[2], line[3], line[0], line[1]))){
            id = i;

            break;
        }
    }

    return id;
}

std::pair<double, int> lineDetecter::corner_check(const Vec4i trgt_line, const std::vector<Vec4i> lines){
    std::vector<Vec4i> lines_opp_angle = find_corner_lines(trgt_line, lines);
    std::vector<Vec4i> intersecting_lines = find_intersecting_lines(trgt_line, lines_opp_angle);
    std::vector<std::pair<double, int>> closest_intersections; // Stores distance and line ID

    if(intersecting_lines.size() == 0){
        return std::make_pair(-1,-1);
    }

    for(size_t i = 0; i < intersecting_lines.size(); i++){
        if((trgt_line == lines[i]) || (lines[i] == Vec4i(trgt_line[2], trgt_line[3], trgt_line[0], trgt_line[1]))){
            continue; // Skip the current line itself
        }

        // SHOULD HAVE MINIMUM VALUE OF INTERSECTION DISTANCE, BUT SHOULD EVALUATE ACCORDING TO THE DISTANCE BETWEEN LINES

        // Calculate the intersection point
        Point intersection = calc_inter(trgt_line, intersecting_lines[i]);

        if(200 < calc_dist(Point(trgt_line[2], trgt_line[3]), intersection)){
            if(i == intersecting_lines.size()-1){
                return std::make_pair(-1,-1);
            }
            else{
                continue;
            }
        }

        // Check if the intersection point is valid
        if(intersection != Point(-1, -1)){
            // Calculate the distance from the trgt_line to the intersecting line
            double dist1 = calc_dist(Point(trgt_line[2], trgt_line[3]), Point(intersecting_lines[i][0], intersecting_lines[i][1]));
            double dist2 = calc_dist(Point(trgt_line[2], trgt_line[3]), Point(intersecting_lines[i][2], intersecting_lines[i][3]));
            double dist = std::min(dist1, dist2);

            // Save the distance and line ID
            int line_id = return_id(intersecting_lines[i], lines);
            closest_intersections.push_back(std::make_pair(dist, line_id));
        }
    }

    // Sort the intersections by distance to the intersecting line
    std::sort(closest_intersections.begin(), closest_intersections.end());

    return closest_intersections[0];
}

Vec4i lineDetecter::orient(Vec4i line, Vec4i next_line){
    double dist1, dist2;

    dist1 = calc_dist(Point(line[2], line[3]), Point(next_line[0], next_line[1]));
    dist2 = calc_dist(Point(line[2], line[3]), Point(next_line[2], next_line[3]));

    if(dist2 < dist1){
        return Vec4i(next_line[2], next_line[3], next_line[0], next_line[1]);
    }

    return next_line;
}

bool lineDetecter::check_end(Vec4i line, Vec4i end_line){

    if(line == end_line || line == Vec4i(end_line[2], end_line[3], end_line[0], end_line[1])){
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        return true;
    }

    return false;
}

std::vector<Vec4i> lineDetecter::find_lines_with_similar_angle(const Vec4i trgt_line, const std::vector<Vec4i> lines, double angle_tolerance = 5.0){
    std::vector<Vec4i> similar_lines;

    // Calculate the angle of the target line
    double target_angle = calc_angle(trgt_line);

    for (const auto& line : lines) {
        // Calculate the angle of the current line
        double line_angle = calc_angle(line);

        // Check if the angle is within the tolerance
        if ((std::abs(target_angle - line_angle) <= angle_tolerance) || (std::abs(target_angle - line_angle) <= 360 - angle_tolerance)){
            similar_lines.push_back(line);
        }
    }

    return similar_lines;
}

std::vector<Vec4i> lineDetecter::within_prox(Vec4i line, std::vector<Vec4i> lines, int prox_val){
    std::vector<Vec4i> lines_close;
    double dist1, dist2;

    for(size_t i = 0; i < lines.size(); i++){
        dist1 = calc_dist(Point(line[2], line[3]), Point(lines[i][0], lines[i][1]));
        dist2 = calc_dist(Point(line[2], line[3]), Point(lines[i][2], lines[i][3]));

        if((prox_val > dist1) || (prox_val > dist2)){
            if((lines[i] != line) && (lines[i] != Vec4i(line[2], line[3], line[0], line[1]))){
                lines_close.push_back(lines[i]);
            }
        }
    }

    return lines_close;
}

Vec4i lineDetecter::extend_line(const Vec4i line, const float scale){
    Point line_start(line[0], line[1]);
    Point line_end(line[2], line[3]);

    // Calculate the directional vector
    Point dir = line_end - line_start;

    // Scale the directional vector
    Point scaled_dir = Point(dir.x * scale, dir.y * scale);

    // Calculate the new end point
    //Point new_start = line_start - scaled_dir;
    Point new_end = line_end + scaled_dir;

    // Return the extended line
    //return Vec4i(new_start.x, new_start.y, new_end.x, new_end.y);
    return Vec4i(line_start.x, line_start.y, new_end.x, new_end.y);
}

std::pair<Vec4i, double> lineDetecter::ex_closest_line(Vec4i cur_line, std::vector<Vec4i> lines, double scale_val){
    double min_dist = DBL_MAX, max_dist_reduc = -DBL_MAX, cur_dist1, cur_dist2, min_dist_before, min_dist_after, saved_dist;
    int cand_id;
    Vec4i ext_line = extend_line(cur_line, scale_val);

    for(size_t i = 0; i < lines.size(); i++){
        if((cur_line == lines[i]) || (lines[i] == Vec4i(cur_line[2], cur_line[3], cur_line[0], cur_line[1]))){
            continue; // Skip the current line itself
        }

        Point intersection = calc_inter(cur_line, lines[i]);

        if (intersection == Point(-1, -1)) {
            continue; // Skip this candidate line if it cannot intersect
        }

        // Calculate the minimum distance before extending
        cur_dist1 = calc_dist(Point(cur_line[2], cur_line[3]), Point(lines[i][0], lines[i][1]));
        cur_dist2 = calc_dist(Point(cur_line[2], cur_line[3]), Point(lines[i][2], lines[i][3]));
        min_dist_before = std::min(cur_dist1, cur_dist2);

        // Calculate the minimum distance after extending
        cur_dist1 = calc_dist(Point(ext_line[2], ext_line[3]), Point(lines[i][0], lines[i][1]));
        cur_dist2 = calc_dist(Point(ext_line[2], ext_line[3]), Point(lines[i][2], lines[i][3]));
        min_dist_after = std::min(cur_dist1, cur_dist2);

        // Calculate the reduction in distance
        double distance_reduc = min_dist_before - min_dist_after;

        // Update the best candidate if this line has the largest reduction
        if(distance_reduc > max_dist_reduc){
            saved_dist = min_dist_after;
            max_dist_reduc = distance_reduc;
            cand_id = i;
        }
    }

    return std::make_pair(lines[cand_id], saved_dist);
}

double lineDetecter::closest_dist(Vec4i line, std::vector<Vec4i> lines){
    double min_dist = DBL_MAX, dist1, dist2;

    for(size_t i = 0; i < lines.size(); i++){
        dist1 = calc_dist(Point(line[2], line[3]), Point(lines[i][0], lines[i][1]));
        dist2 = calc_dist(Point(line[2], line[3]), Point(lines[i][2], lines[i][3]));

        if((min_dist > dist1) || (min_dist > dist2)){
            if((line != lines[i]) && (lines[i] != Vec4i(line[2], line[3], line[0], line[1]))){
                min_dist = (dist2 > dist1 ? dist1 : dist2);
            }
        }
    }

    return min_dist;
}

Vec4i lineDetecter::closest_line(Vec4i line, std::vector<Vec4i> lines){
    double min_dist = DBL_MAX, cur_dist1, cur_dist2;
    int cand_id;

    for(size_t i = 0; i < lines.size(); i++){
        cur_dist1 = calc_dist(Point(line[2], line[3]), Point(lines[i][0], lines[i][1]));
        cur_dist2 = calc_dist(Point(line[2], line[3]), Point(lines[i][2], lines[i][3]));

        if((min_dist > cur_dist1) || (min_dist > cur_dist2)){
            if((line != lines[i]) && (lines[i] != Vec4i(line[2], line[3], line[0], line[1]))){
                min_dist = (cur_dist1 < cur_dist2 ? cur_dist1 : cur_dist2);
                cand_id = i;
            }
        }
    }

    return lines[cand_id];
}

bool lineDetecter::is_line_between(const Vec4i line1, const Vec4i line2, const Vec4i candidate_line){
    // Extract points from the lines
    Point line1_start(line1[0], line1[1]);
    Point line1_end(line1[2], line1[3]);
    Point line2_start(line2[0], line2[1]);
    Point line2_end(line2[2], line2[3]);

    Point cand_start(candidate_line[0], candidate_line[1]);
    Point cand_end(candidate_line[2], candidate_line[3]);

    // Calculate the midpoint of the candidate line
    Point cand_mid((cand_start.x + cand_end.x) / 2, (cand_start.y + cand_end.y) / 2);

    // Calculate the bounding box formed by line1 and line2
    int x_min = std::min({line1_start.x, line1_end.x, line2_start.x, line2_end.x});
    int x_max = std::max({line1_start.x, line1_end.x, line2_start.x, line2_end.x});
    int y_min = std::min({line1_start.y, line1_end.y, line2_start.y, line2_end.y});
    int y_max = std::max({line1_start.y, line1_end.y, line2_start.y, line2_end.y});

    // Check if any of the candidate line's points (start, end, or midpoint) lie within the bounding box
    if ((cand_start.x >= x_min && cand_start.x <= x_max && cand_start.y >= y_min && cand_start.y <= y_max) ||
        (cand_end.x >= x_min && cand_end.x <= x_max && cand_end.y >= y_min && cand_end.y <= y_max) ||
        (cand_mid.x >= x_min && cand_mid.x <= x_max && cand_mid.y >= y_min && cand_mid.y <= y_max)) {
        return true; // Candidate line has at least one point within the bounding box
    }

    return false; // Candidate line does not have any points between line1 and line2
}

Vec4i lineDetecter::find_line_between(const Vec4i line1, const Vec4i line2, const std::vector<Vec4i> lines){
    for(const auto& candidate_line : lines){
        // Check if the candidate line is between line1 and line2
        if(is_line_between(line1, line2, candidate_line)){
            return candidate_line; // Return the first line found that is between line1 and line2
        }
    }

    // If no line is found, return an invalid line
    return Vec4i(-1, -1, -1, -1);
}

bool lineDetecter::corner_between(const Vec4i line1, const Vec4i line2, const int angle_limit){

    double angle, angle1, angle2;

    angle1 = calc_angle(line1);
    angle2 = calc_angle(line2);

    angle = std::abs(angle1 - angle2);

    if((angle > angle_limit) && (angle < 180 - angle_limit)){
        return true;
    }
    
    return false;
}

void lineDetecter::sort(){

    _SS_points = get_SS_points();

    std::vector<Vec4i> temp_lines, result, lines = _lines;
    Vec4i new_trgt, trgt_line, temp_line, end_line = find_end_line(lines, _SS_points);
    std::pair<Vec4i, double> temp_pair;
    std::pair<double, int> inter_pair;
    double end_dist;

    int start_id = find_start_line(), full = lines.size();

    // Make sure the starting line is oriented correctly
    if(calc_dist(Point(lines[start_id][2], lines[start_id][3]), Point(_SS_points[0][0],_SS_points[0][1])) < calc_dist(Point(lines[start_id][0], lines[start_id][1]), Point(_SS_points[0][0], _SS_points[0][1]))){
        double temp = lines[start_id][2];
        lines[start_id][2] = lines[start_id][0];
        lines[start_id][0] = temp;

        temp = lines[start_id][3];
        lines[start_id][3] = lines[start_id][1];
        lines[start_id][1] = temp;
    }

    result.push_back(lines[start_id]);

    lines.erase(lines.begin() + start_id);

    do{
        //From here should use the back elemestd::cout << "test3" << std::endl;nt of the result vector for trgt line. Because the line is removed in lines
        trgt_line = result.back();

        if(lines.size() > 1){
            inter_pair = corner_check(trgt_line, lines);

            // If a intersection point is found within 50 units of the target line, then there is a corner
            // And no further processing is needed
            if((inter_pair.first != -1) && (inter_pair.first < 100)){
                temp_line = orient(trgt_line, lines[inter_pair.second]);

                result.push_back(temp_line);
                lines.erase(lines.begin() + return_id(temp_line, lines));

                if(check_end(temp_line, end_line)){
                    break;
                }

                continue;
            }
        }

        temp_lines = find_lines_with_similar_angle(trgt_line, lines);

        temp_lines = within_prox(trgt_line, temp_lines, 400);

        if(temp_lines.size() != 0){
            temp_pair = ex_closest_line(trgt_line, temp_lines, 1.2);

            double comp_dist = closest_dist(trgt_line, lines);

            // If the extending the line led to finding a line which is closer than the original closest line
            if(temp_pair.second <= comp_dist){
                new_trgt = orient(trgt_line, temp_pair.first);
                result.push_back(new_trgt);
            }
            // If not, then choose the line which is closest
            else{
                new_trgt = closest_line(trgt_line, lines);
                new_trgt = orient(trgt_line, new_trgt);
                result.push_back(new_trgt);
            }
        }
        // No lines with the same angle was found within the proximity range.
        else{

            if(lines.size() != 0){
                // Check for lines with similar angles within all of the remaining lines
                temp_lines = find_lines_with_similar_angle(trgt_line, lines);

                // Check if any lines can intersect with the current line - Checks for corners
                temp_lines = find_intersecting_lines(trgt_line, temp_lines);
            }
            else{
                temp_lines.clear();
            }

            // If any lines can intersect, then find the closest line
            if(temp_lines.size() != 0){
                new_trgt = closest_line(trgt_line, temp_lines);
            }
            // If no lines can intersect, then find the line (of the remaining) which is closest
            else{
                new_trgt = closest_line(trgt_line, lines);
                //temp_pair = ex_closest_line(trgt_line, lines, 1.0, img, true);
                //new_trgt = temp_pair.first;
            }

            new_trgt = orient(trgt_line, new_trgt);
            result.push_back(new_trgt);
        }

        lines.erase(lines.begin() + return_id(new_trgt, lines));

        if(check_end(new_trgt, end_line)){
            break;
        }
        continue;

        if(result.size() >= 2){
            Vec4i line_between = find_line_between(result[result.size()-2], result.back(), lines);

            if(line_between != Vec4i(-1, -1, -1, -1)){

                line_between = orient(result[result.size()-2], line_between);

                result.push_back(line_between);

                temp_line = result[result.size()-2];
                result[result.size()-2] = line_between;
                result[result.size()-1] = temp_line;

                lines.erase(lines.begin() + return_id(line_between, lines));
            }
        }

        
    }while(lines.size() != 0);

    std::vector<Point> corners;

    // Connect corners and lines together
    for(int i = 1; i < result.size(); i++){

        if(i == 1){
            result[i-1][0] = _SS_points[0][0];
            result[i-1][1] = _SS_points[0][1];
        }
        else{            
            result[i-1][2] = result[i][0];
            result[i-1][3] = result[i][1];
        }

        if(i == result.size()-1){
            result[i][2] = _SS_points[1][0];
            result[i][3] = _SS_points[1][1];
        }
        else if(corner_between(result[i], result[i+1], 15)){
            corners.push_back(calc_inter(result[i], result[i+1]));

            result[i-1][2] = corners.back().x;
            result[i-1][3] = corners.back().y;
            result[i][0] = corners.back().x;
            result[i][1] = corners.back().y;
        }
    }

    _comp_path = result;
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
<<<<<<< Updated upstream
<<<<<<< Updated upstream

void lineDetecter::find_inters(){
    cv::Point inter;
    std::vector<cv::Vec4i> ext_lines = _lines;
    _inters.push_back(cv::Point(_SS_points[0][0], _SS_points[0][1]));

    for(size_t i = 0; i < _lines.size()-1; i++){
        if(corner_between(_lines[i], _lines[i+1], _angle_limit)){
            ext_lines[i] = extend_line(_lines[i], 1.2);
            ext_lines[i+1] = extend_line(_lines[i+1], 1.2);
            inter = calc_inter(_lines[i], _lines[i+1]);
            _inters.push_back(inter);
        }
    }

    _inters.push_back(cv::Point(_SS_points[1][0], _SS_points[1][1]));
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

void lineDetecter::perp_line(){
    for(size_t i = 0; i < _lines.size(); i++){
        cv::Point line_start(_lines[i][0], _lines[i][1]);
        cv::Point line_end(_lines[i][2], _lines[i][3]);

        // Calculate the directional vector
        cv::Point dir = line_end - line_start;
        cv::Point middle = line_start + (dir/2);

        // Create a perpendicular vector by swapping the components and changing the sign of one of them
        cv::Point perp_dir(-dir.y, dir.x);

        // Normalize the perpendicular vector
        float norm = sqrt(perp_dir.x * perp_dir.x + perp_dir.y * perp_dir.y);
        perp_dir.x = static_cast<int>((perp_dir.x / norm) * _perp_length);
        perp_dir.y = static_cast<int>((perp_dir.y / norm) * _perp_length);

        // Calculate the new end points for the perpendicular line
        cv::Point new_start = middle + perp_dir;
        cv::Point new_end = middle - perp_dir;

        _perps.push_back(cv::Vec4i(new_start.x, new_start.y, new_end.x, new_end.y));
    }
}

void lineDetecter::sort_perps(){
    std::vector<cv::Vec4i> temp_lines;
    std::vector<cv::Vec4i> temp_perps;
    bool found = false;

    for(size_t i = 0; i < _perps.size(); i++){
        int temp_id = i;
        for (size_t j = 0; j < _lines.size(); j++) {
            if (i == j || j == _start_id) {
                continue;
            }

            if (detect_overlap(_perps[i], _lines[j])) {
                if (calc_line_len(_lines[temp_id]) < calc_line_len(_lines[j])) {
                    temp_id = j;
                }
            }
        }

        found = false;

        for (size_t j = 0; j < temp_lines.size(); j++) {
            if(temp_lines[j] == _lines[temp_id]){
                found = true;
                break;
            }
        }

        if(!found){
            temp_lines.push_back(_lines[temp_id]);
            temp_perps.push_back(_perps[temp_id]);
        }
    }

    _lines = temp_lines;
    _perps = temp_perps;
}

void lineDetecter::eliminate_long_lines(int max_len){
    int len;

    for(size_t i = 0; i < _lines.size(); i++){
        len = calc_line_len(_lines[i]);

        if(len >= max_len){
            _lines.erase(_lines.begin() + i);
            i--;
        }
    }
}

void lineDetecter::onMouse(int event, int x, int y, int, void*) {
    std::cout << "Click now" << std::endl;
    if (event == cv::EVENT_LBUTTONDOWN && _SS_points.size() < 2) {
        _SS_points.push_back(cv::Vec3f(x, y, 5));
        std::cout << "Clicked: " << x << ", " << y << std::endl;
    }
}
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
