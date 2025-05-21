#include "linedetecter.h"

std::vector<cv::Vec3f> _SS_points;

lineDetecter::lineDetecter(cv::Mat img): imageHandler(img){

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

    detect_lines();

    cvtColor(_img, _img, cv::COLOR_GRAY2BGR);

    std::cout << _img.cols << ", " << _img.rows << std::endl;

    sort();

    find_inters();

    for(size_t i = 0; i < _comp_path.size(); i++){
        cv::line(output, cv::Point(_comp_path[i][0], _comp_path[i][1]), cv::Point(_comp_path[i][2], _comp_path[i][3]), cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    }

    namedWindow("output", cv::WINDOW_NORMAL);
    imshow("output", output);

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

int lineDetecter::calc_line_len(cv::Vec4i line){

    int x = line[2] - line[0];
    int y = line[3] - line[1];

    int len = sqrt((x*x)+(y*y));

    return len;
}

void lineDetecter::eliminate_small_lines(int min_len){

    int len;

    for(size_t i = 0; i < _lines.size(); i++){
        len = calc_line_len(_lines[i]);

        if(len <= min_len){
            _lines.erase(_lines.begin() + i);
            i--;
        }
    }
}

bool lineDetecter::detect_overlap(const cv::Vec4i line1, const cv::Vec4i line2, int tolerance){
    // Check if the bounding boxes of the lines overlap
    int x1_min = std::min(line1[0], line1[2]) - tolerance;
    int x1_max = std::max(line1[0], line1[2]) + tolerance;
    int y1_min = std::min(line1[1], line1[3]) - tolerance;
    int y1_max = std::max(line1[1], line1[3]) + tolerance;

    int x2_min = std::min(line2[0], line2[2]) - tolerance;
    int x2_max = std::max(line2[0], line2[2]) + tolerance;
    int y2_min = std::min(line2[1], line2[3]) - tolerance;
    int y2_max = std::max(line2[1], line2[3]) + tolerance;

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

            if(detect_overlap(_lines[i], _lines[j], _overlap_tolerance)){
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
    HoughLinesP(_img, _lines, 1, CV_PI/180, 50, _min_length, 10); // runs the actual detection

    eliminate_overlap();
}

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

cv::Vec4i lineDetecter::orient(cv::Vec4i line, cv::Vec4i next_line){
    double dist1, dist2;

    dist1 = calc_dist(cv::Point(line[2], line[3]), cv::Point(next_line[0], next_line[1]));
    dist2 = calc_dist(cv::Point(line[2], line[3]), cv::Point(next_line[2], next_line[3]));

    if(dist2 < dist1){
        return cv::Vec4i(next_line[2], next_line[3], next_line[0], next_line[1]);
    }

    return next_line;
}

int calc_line_len(cv::Vec4i line){

    int x = line[2] - line[0];
    int y = line[3] - line[1];

    int len = sqrt((x*x)+(y*y));

    return len;
}

double calc_angle(const cv::Vec4i line) {
    cv::Point start(line[0], line[1]);
    cv::Point end(line[2], line[3]);

    // Calculate the direction vector
    cv::Point dir = end - start;

    // Calculate the angle in degrees
    double angle = atan2(dir.y, dir.x) * (180.0 / CV_PI);

    // Normalize the angle to [0, 180)
    if (angle < 0) {
        angle += 180.0;
    }

    return angle;
}

std::vector<cv::Vec4i> find_corner_lines(const cv::Vec4i current_line, const std::vector<cv::Vec4i> lines, double angle_threshold = 30.0){
    std::vector<cv::Vec4i> corner_lines;

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

std::vector<cv::Vec4i> lineDetecter::find_intersecting_lines(const cv::Vec4i cur_line, const std::vector<cv::Vec4i> grouped_lines){
    std::vector<cv::Vec4i> intersecting_lines;

    for (const auto& line : grouped_lines) {
        // Calculate the intersection point
        cv::Point intersection = calc_inter(cur_line, line);

        // Check if the intersection point is valid

        if((intersection != cv::Point(-1,-1)) && (1000 > intersection.x) && (intersection.x > 0) && (1000 > intersection.y) && (intersection.y > 0)){
            if(cur_line != line){
                intersecting_lines.push_back(line);
            }
        }
    }

    return intersecting_lines;
}

int return_id(cv::Vec4i line, std::vector<cv::Vec4i> lines){
    int id = -1;

    for(size_t i = 0; i < lines.size(); i++){
        if((line == lines[i]) || (lines[i] == cv::Vec4i(line[2], line[3], line[0], line[1]))){
            id = i;

            break;
        }
    }

    return id;
}

std::pair<double, int> lineDetecter::corner_check(const cv::Vec4i trgt_line, const std::vector<cv::Vec4i> lines){
    std::vector<cv::Vec4i> lines_opp_angle = find_corner_lines(trgt_line, lines);
    std::vector<cv::Vec4i> intersecting_lines = find_intersecting_lines(trgt_line, lines_opp_angle);
    std::vector<std::pair<double, int>> closest_intersections; // Stores distance and line ID

    if(intersecting_lines.size() == 0){
        return std::make_pair(-1,-1);
    }

    for(size_t i = 0; i < intersecting_lines.size(); i++){
        if((trgt_line == lines[i]) || (lines[i] == cv::Vec4i(trgt_line[2], trgt_line[3], trgt_line[0], trgt_line[1]))){
            continue; // Skip the current line itself
        }

        // Calculate the intersection point
        cv::Point intersection = calc_inter(trgt_line, intersecting_lines[i]);

        if(_inter_min_dist < calc_dist(cv::Point(trgt_line[2], trgt_line[3]), intersection)){
            if(i == intersecting_lines.size()-1){
                return std::make_pair(-1,-1);
            }
            else{
                continue;
            }
        }

        // Check if the intersection point is valid
        if(intersection != cv::Point(-1, -1)){
            // Calculate the distance from the trgt_line to the intersecting line
            double dist1 = calc_dist(cv::Point(trgt_line[2], trgt_line[3]), cv::Point(intersecting_lines[i][0], intersecting_lines[i][1]));
            double dist2 = calc_dist(cv::Point(trgt_line[2], trgt_line[3]), cv::Point(intersecting_lines[i][2], intersecting_lines[i][3]));
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

bool lineDetecter::check_end(cv::Vec4i line, std::vector<cv::Vec3f> SS_points){
    if(calc_dist(cv::Point(line[2], line[3]), cv::Point(SS_points[1][0], SS_points[1][1])) < 150){
        return true;
    }

    return false;
}

std::vector<cv::Vec4i> find_lines_with_similar_angle(const cv::Vec4i trgt_line, const std::vector<cv::Vec4i> lines, double angle_tolerance = 5.0){
    std::vector<cv::Vec4i> similar_lines;

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

std::vector<cv::Vec4i> lineDetecter::within_prox(cv::Vec4i line, std::vector<cv::Vec4i> lines, int prox_val){
    std::vector<cv::Vec4i> lines_close;
    double dist1, dist2;

    for(size_t i = 0; i < lines.size(); i++){
        dist1 = calc_dist(cv::Point(line[2], line[3]), cv::Point(lines[i][0], lines[i][1]));
        dist2 = calc_dist(cv::Point(line[2], line[3]), cv::Point(lines[i][2], lines[i][3]));

        if((prox_val > dist1) || (prox_val > dist2)){
            if((lines[i] != line) && (lines[i] != cv::Vec4i(line[2], line[3], line[0], line[1]))){
                lines_close.push_back(lines[i]);
            }
        }
    }

    return lines_close;
}

std::pair<cv::Vec4i, double> lineDetecter::ex_closest_line(cv::Vec4i cur_line, std::vector<cv::Vec4i> lines, double scale_val){
    double min_dist = DBL_MAX, max_dist_reduc = -DBL_MAX, cur_dist1, cur_dist2, min_dist_before, min_dist_after, saved_dist;
    int cand_id;
    cv::Vec4i ext_line = extend_line(cur_line, scale_val);

    for(size_t i = 0; i < lines.size(); i++){
        if((cur_line == lines[i]) || (lines[i] == cv::Vec4i(cur_line[2], cur_line[3], cur_line[0], cur_line[1]))){
            continue; // Skip the current line itself
        }

        cv::Point intersection = calc_inter(cur_line, lines[i]);

        if (intersection == cv::Point(-1, -1)) {
            continue; // Skip this candidate line if it cannot intersect
        }

        // Calculate the minimum distance before extending
        cur_dist1 = calc_dist(cv::Point(cur_line[2], cur_line[3]), cv::Point(lines[i][0], lines[i][1]));
        cur_dist2 = calc_dist(cv::Point(cur_line[2], cur_line[3]), cv::Point(lines[i][2], lines[i][3]));
        min_dist_before = std::min(cur_dist1, cur_dist2);

        // Calculate the minimum distance after extending
        cur_dist1 = calc_dist(cv::Point(ext_line[2], ext_line[3]), cv::Point(lines[i][0], lines[i][1]));
        cur_dist2 = calc_dist(cv::Point(ext_line[2], ext_line[3]), cv::Point(lines[i][2], lines[i][3]));
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

double lineDetecter::closest_dist(cv::Vec4i line, std::vector<cv::Vec4i> lines){
    double min_dist = DBL_MAX, dist1, dist2;

    for(size_t i = 0; i < lines.size(); i++){
        dist1 = calc_dist(cv::Point(line[2], line[3]), cv::Point(lines[i][0], lines[i][1]));
        dist2 = calc_dist(cv::Point(line[2], line[3]), cv::Point(lines[i][2], lines[i][3]));

        if((min_dist > dist1) || (min_dist > dist2)){
            if((line != lines[i]) && (lines[i] != cv::Vec4i(line[2], line[3], line[0], line[1]))){
                min_dist = (dist2 > dist1 ? dist1 : dist2);
            }
        }
    }

    return min_dist;
}

cv::Vec4i lineDetecter::closest_line(cv::Vec4i line, std::vector<cv::Vec4i> lines){
    double min_dist = DBL_MAX, cur_dist1, cur_dist2;
    int cand_id;

    for(size_t i = 0; i < lines.size(); i++){
        cur_dist1 = calc_dist(cv::Point(line[2], line[3]), cv::Point(lines[i][0], lines[i][1]));
        cur_dist2 = calc_dist(cv::Point(line[2], line[3]), cv::Point(lines[i][2], lines[i][3]));

        if((min_dist > cur_dist1) || (min_dist > cur_dist2)){
            if((line != lines[i]) && (lines[i] != cv::Vec4i(line[2], line[3], line[0], line[1]))){
                min_dist = (cur_dist1 < cur_dist2 ? cur_dist1 : cur_dist2);
                cand_id = i;
            }
        }
    }

    return lines[cand_id];
}

bool lineDetecter::is_line_between(const cv::Vec4i line1, const cv::Vec4i line2, const cv::Vec4i candidate_line, std::vector<cv::Vec4i> lines, cv::Mat img, int tolerance = 15){
    // Extract points from the lines
    cv::Point line1_start(line1[0], line1[1]);
    cv::Point line1_end(line1[2], line1[3]);
    cv::Point line2_start(line2[0], line2[1]);
    cv::Point line2_end(line2[2], line2[3]);

    cv::Point cand_start(candidate_line[0], candidate_line[1]);
    cv::Point cand_end(candidate_line[2], candidate_line[3]);

    // Calculate the midpoint of the candidate line
    cv::Point cand_mid((cand_start.x + cand_end.x) / 2, (cand_start.y + cand_end.y) / 2);

    // Calculate the bounding box formed by line1 and line2
    int x_min = std::min({line1_start.x, line1_end.x, line2_start.x, line2_end.x}) - tolerance;
    int x_max = std::max({line1_start.x, line1_end.x, line2_start.x, line2_end.x}) + tolerance;
    int y_min = std::min({line1_start.y, line1_end.y, line2_start.y, line2_end.y}) - tolerance;
    int y_max = std::max({line1_start.y, line1_end.y, line2_start.y, line2_end.y}) + tolerance;

    // Check if any of the candidate line's points (start, end, or midpoint) lie within the bounding box
    if ((cand_mid.x >= x_min && cand_mid.x <= x_max && cand_mid.y >= y_min && cand_mid.y <= y_max)) {

        //line(img, Point(x_min, y_min), Point(x_min, y_max), Scalar(0, 255, 0), 2);
        //line(img, Point(x_min, y_max), Point(x_max, y_max), Scalar(0, 255, 0), 2);
        //line(img, Point(x_max, y_max), Point(x_max, y_min), Scalar(0, 255, 0), 2);
        //line(img, Point(x_max, y_min), Point(x_min, y_min), Scalar(0, 255, 0), 2);

        cv::Vec4i temp_line = closest_line(line1, lines);

        // Make sure the candidate line is the line which we are trying to add. If lines are not same angle, it can cause issues.
        if((candidate_line == temp_line) && (line1 != temp_line) && (line1 != cv::Vec4i(temp_line[2], temp_line[3], temp_line[0], temp_line[1])) && (line2 != temp_line) && (line2 != cv::Vec4i(temp_line[2], temp_line[3], temp_line[0], temp_line[1]))){
            return true;
        }
    }

    return false; // Candidate line does not have any points between line1 and line2
}

cv::Vec4i lineDetecter::find_line_between(const cv::Vec4i line1, const cv::Vec4i line2, const std::vector<cv::Vec4i> lines, cv::Mat img){

    for(const auto candidate_line : lines){
        // Check if the candidate line is between line1 and line2
        if(is_line_between(line1, line2, candidate_line, lines, img)){
            return candidate_line; // Return the first line found that is between line1 and line2
        }
    }

    // If no line is found, return an invalid line
    return cv::Vec4i(-1, -1, -1, -1);
}

void lineDetecter::sort(){

    bool remove_line = false;

    int prev_result_size = 0, counter = 0;

    std::pair<cv::Vec4i, double> temp_pair;
    std::pair<double, int> inter_pair;

    cv::Vec4i trgt_line, new_trgt, temp_line;
    std::vector<cv::Vec4i> temp_lines, result, lines = _lines;

    _start_id = find_start_line();
    int full = lines.size();

    result.push_back(orient(cv::Vec4i(0, 0, _SS_points[0][0], _SS_points[0][1]), lines[_start_id]));

    lines.erase(lines.begin() + _start_id);

    do{
        /*for(size_t j = 0; j < lines.size(); j++){
            line(_img, cv::Point(lines[j][0], lines[j][1]), cv::Point(lines[j][2], lines[j][3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
            namedWindow("sort", cv::WINDOW_NORMAL);
            imshow("sort", _img);
        }*/

        trgt_line = result.back();

        //line(_img, cv::Point(trgt_line[0], trgt_line[1]), cv::Point(trgt_line[2], trgt_line[3]), cv::Scalar(0,255,0), 3, cv::LINE_AA);

        if(lines.size() > 1){
            inter_pair = corner_check(trgt_line, lines);

            /*std::cout << "corner check" << std::endl;
            line(_img, cv::Point(lines[inter_pair.second][0], lines[inter_pair.second][1]), cv::Point(lines[inter_pair.second][2], lines[inter_pair.second][3]), cv::Scalar(0,255,255), 3, cv::LINE_AA);

            cv::namedWindow("sort", cv::WINDOW_NORMAL);
            cv::imshow("sort", _img);

            cv::waitKey(0);*/

            // If a intersection point is found within 100 units of the target line, then there is a corner
            if((inter_pair.first != -1) && (inter_pair.first < _inter_line_min_dist)){
                temp_line = orient(trgt_line, lines[inter_pair.second]);

                result.push_back(temp_line);

                /*line(_img, cv::Point(temp_line[0], temp_line[1]), cv::Point(temp_line[2], temp_line[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);

                cv::namedWindow("sort", cv::WINDOW_NORMAL);
                cv::imshow("sort", _img);
                cv::waitKey(0);*/

                lines.erase(lines.begin() + return_id(temp_line, lines));

                if(check_end(temp_line, _SS_points)){
                    break;
                }
                else{
                    trgt_line = result.back();
                }
            }
        }

        temp_lines = find_lines_with_similar_angle(trgt_line, lines);

        /*for(size_t j = 0; j < temp_lines.size(); j++){
            line(_img, cv::Point(temp_lines[j][0], temp_lines[j][1]), cv::Point(temp_lines[j][2], temp_lines[j][3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);

            cv::namedWindow("sort", cv::WINDOW_NORMAL);
            cv::imshow("sort", _img);
        }

        cv::waitKey(0);*/

        temp_lines = within_prox(trgt_line, temp_lines, _prox_dist);

        if(temp_lines.size() != 0){
            /*for(size_t i = 0; i < temp_lines.size(); i++){
                line(_img, cv::Point(temp_lines[i][0], temp_lines[i][1]), cv::Point(temp_lines[i][2], temp_lines[i][3]), cv::Scalar(255,255,0), 3, cv::LINE_AA);

                cv::namedWindow("sort", cv::WINDOW_NORMAL);
                cv::imshow("sort", _img);
            }

            cv::waitKey(0);*/

            temp_pair = ex_closest_line(trgt_line, temp_lines, 1.1);

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

            if(check_end(new_trgt, _SS_points)){
                break;
            }
            else{
                trgt_line = result.back();
                remove_line = true;
            }
        }

        /*line(_img, cv::Point(new_trgt[0], new_trgt[1]), cv::Point(new_trgt[2], new_trgt[3]), cv::Scalar(255,255,255), 3, cv::LINE_AA);

        cv::namedWindow("sort", cv::WINDOW_NORMAL);
        cv::imshow("sort", _img);*/

        if(result.size() >= 2){

            cv::Vec4i line_between = find_line_between(result[result.size()-2], result[result.size()-1], lines, _img);

            if(line_between != cv::Vec4i(-1, -1, -1, -1)){

                /*cv::line(_img, cv::Point(line_between[0], line_between[1]), cv::Point(line_between[2], line_between[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);

                cv::namedWindow("sort", cv::WINDOW_NORMAL);
                cv::imshow("sort", _img);
                cv::waitKey(0);*/

                line_between = orient(result[result.size()-2], line_between);

                temp_line = result[result.size()-1];
                result[result.size()-1] = line_between;
                result.push_back(temp_line);

                lines.erase(lines.begin() + return_id(line_between, lines));
            }
        }

        if(remove_line){

            /*cv::line(_img, cv::Point(trgt_line[0], trgt_line[1]), cv::Point(trgt_line[2], trgt_line[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);

            cv::namedWindow("sort", cv::WINDOW_NORMAL);
            cv::imshow("sort", _img);
            cv::waitKey(0);*/

            lines.erase(lines.begin() + return_id(trgt_line, lines));
            remove_line = false;
        }

        /*cv::namedWindow("sort", cv::WINDOW_NORMAL);
        cv::imshow("sort", _img);

        cv::waitKey(0);*/

        if(check_end(new_trgt, _SS_points)){
            break;
        }
        else{
            if(result.size() == prev_result_size){
                counter++;
            }
            else{
                counter = 0;
                prev_result_size = result.size();
            }

            if(counter == 3){
                break;
            }
        }

    }while(lines.size() != 0);

    std::vector<cv::Point> corners;

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
        else if(corner_between(result[i], result[i+1], _angle_limit)){
            corners.push_back(calc_inter(result[i], result[i+1]));

            result[i-1][2] = corners.back().x;
            result[i-1][3] = corners.back().y;
            result[i][0] = corners.back().x;
            result[i][1] = corners.back().y;
        }
    }

    _comp_path = result;
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
    float angle = calc_angle(line1, line2);

    if((angle > angle_limit) && (angle < (180 - angle_limit))){
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
    //cv::Point inter;
    //std::vector<cv::Vec4i> ext_lines = _lines;

    _inters.push_back(cv::Point(_comp_path[0][0], _comp_path[0][1]));

    for(size_t i = 0; i < _comp_path.size(); i++){
        _inters.push_back(cv::Point(_comp_path[i][2], _comp_path[i][3]));
    }

    /*_inters.push_back(cv::Point(_SS_points[0][0], _SS_points[0][1]));

    for(size_t i = 0; i < _lines.size()-1; i++){
        if(corner_between(_lines[i], _lines[i+1], _angle_limit)){
            ext_lines[i] = extend_line(_lines[i], 1.2);
            ext_lines[i+1] = extend_line(_lines[i+1], 1.2);
            inter = calc_inter(_lines[i], _lines[i+1]);
            _inters.push_back(inter);
        }
    }

    _inters.push_back(cv::Point(_SS_points[1][0], _SS_points[1][1]));*/
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

/*void lineDetecter::perp_line(){
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
}*/

void lineDetecter::onMouse(int event, int x, int y, int, void*) {
    //std::cout << "Click now" << std::endl;
    if (event == cv::EVENT_LBUTTONDOWN && _SS_points.size() < 2) {
        _SS_points.push_back(cv::Vec3f(x, y, 5));
        std::cout << "Clicked: " << x << ", " << y << std::endl;
    }
}
