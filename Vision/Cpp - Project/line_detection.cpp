#include "line_detection.h"

int calc_dist(Point p1, Point p2){
    
    /*
        Description:
        Calculates the distance between two points, and returns the calculated value.
    */

    int x = p2.x - p1.x;
    int y = p2.y - p1.y;

    int dist = sqrt((x*x)+(y*y));

    return dist;
}

double calc_angle(const Vec4i line) {
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

int find_start_line(std::vector<Vec4i> lines, std::vector<Vec3f> SS_points){

    /*
        Description:
        Calculates and returns the element id of the first line segment in vector lines.
        The first line segment is defined as the line segment closest to the start identifier
    */
    
    Point start_pnt(SS_points[0][0], SS_points[0][1]);

    int temp_id, dist1, dist2, min_dist = INT_MAX;

    for(size_t i = 0; i < lines.size(); i++){
        dist1 = calc_dist(start_pnt, Point(lines[i][0], lines[i][1]));
        dist2 = calc_dist(start_pnt, Point(lines[i][2], lines[i][3]));

        if((dist1 < min_dist) || (dist2 < min_dist)){
            min_dist = (dist1 < dist2 ? dist1 : dist2);
            temp_id = i;
        }
    }

    return temp_id;
}

std::vector<Vec4i> find_lines_with_similar_angle(const Vec4i trgt_line, const std::vector<Vec4i> lines, double angle_tolerance = 5.0){
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

std::vector<Vec4i> find_intersecting_lines(const Vec4i cur_line, const std::vector<Vec4i> grouped_lines){
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

int calc_line_len(Vec4i line){

    /*
        Description:
        Calculates the length of given line, and returns calculated value.
    */

    int x = line[2] - line[0];
    int y = line[3] - line[1];

    int len = sqrt((x*x)+(y*y));

    return len;
}

void eliminate_small_lines(std::vector<Vec4i>& line_Vec, int min_len){

    /*
        Description:
        Erases lines with length shorter than min_len from the vector of lines/path.
    */

    int len;

    for(size_t i = 0; i < line_Vec.size(); i++){
        len = calc_line_len(line_Vec[i]);

        if(len <= min_len){
            line_Vec.erase(line_Vec.begin() + i);
            i--;
        }
    }
}

bool detect_overlap(const Vec4i line1, const Vec4i line2) {
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

std::vector<Vec4i> eliminate_overlap(const std::vector<Vec4i> lines) {
    std::vector<Vec4i> result;
    std::vector<bool> to_remove(lines.size(), false);

    for (size_t i = 0; i < lines.size(); i++){
        if(to_remove[i]){
            continue;
        }

        for(size_t j = i + 1; j < lines.size(); j++){
            if(to_remove[j]){
                continue;
            }

            if(detect_overlap(lines[i], lines[j])){
                int len1 = calc_line_len(lines[i]);
                int len2 = calc_line_len(lines[j]);

                if(len1 >= len2) {
                    to_remove[j] = true;
                }else {
                    to_remove[i] = true;
                    break;
                }
            }
        }
    }

    for(size_t i = 0; i < lines.size(); i++){
        if(!to_remove[i]){
            result.push_back(lines[i]);
        }
    }

    return result;
}

std::vector<Vec4i> detect_lines(const Mat img, int min_len){
    std::vector<Vec4i> temp_lines; // will hold the results of the detection
    HoughLinesP(img, temp_lines, 1, CV_PI/180, 50, 50, 10); // runs the actual detection

    eliminate_small_lines(temp_lines, min_len);

    temp_lines = eliminate_overlap(temp_lines);

    return temp_lines;
}

Vec4i find_closest_pair(const Vec4i line1, const Vec4i line2){

    float min_dist = INT_MAX;
    float dist;
    Vec4i closest_pair;

    // Iterate through all possible pairs of points
    for (size_t i = 0; i < 3; i = i + 2) {

        dist = calc_dist(Point(line1[0], line1[1]), Point(line2[i], line2[i+1]));
        if(dist < min_dist){
            min_dist = dist;
            closest_pair = Vec4i(line1[0], line1[1], line2[i], line2[i+1]);
        }

        dist = calc_dist(Point(line1[2], line1[3]), Point(line2[i], line2[i+1]));
        if(dist < min_dist){
            min_dist = dist;
            closest_pair = Vec4i(line1[2], line1[3], line2[i], line2[i+1]);
        }
    }

    return closest_pair;
}

std::vector<Vec4i> conn_lines(const std::vector<Vec4i> lines, const std::vector<Vec3f> SS_points){
    std::vector<Vec4i> temp_lines = lines;
    Vec4i clos_points;


    Point start_pnt(SS_points[0][0], SS_points[0][1]);
    Point end_pnt(SS_points[1][0], SS_points[1][1]);

    for(size_t i = 0; i < lines.size(); i++){

        Point line_pnt1(lines[i][0], lines[i][1]);
        Point line_pnt2(lines[i][2], lines[i][3]);

        if(i == 0){

            if(calc_dist(line_pnt1, start_pnt) < calc_dist(line_pnt2, start_pnt)){
                temp_lines[i][0] = start_pnt.x;
                temp_lines[i][1] = start_pnt.y;
            }
            else{
                temp_lines[i][2] = start_pnt.x;
                temp_lines[i][3] = start_pnt.y;
            }
        }
        else if(i == lines.size()-1){
            if(calc_dist(line_pnt1, end_pnt) < calc_dist(line_pnt2, end_pnt)){
                temp_lines[i][0] = end_pnt.x;
                temp_lines[i][1] = end_pnt.y;
            }
            else{
                temp_lines[i][2] = end_pnt.x;
                temp_lines[i][3] = end_pnt.y;
            }
        }
        
        if(!corner_between(lines[i], lines[i+1], 15)){
            
            clos_points = find_closest_pair(lines[i], lines[i+1]);

            for(size_t j = 0; j < 3; j = j + 2){
                if((lines[i][j] == clos_points[0]) && (lines[i][j+1] == clos_points[1])){
                    temp_lines[i][j] = clos_points[2];
                    temp_lines[i][j+1] = clos_points[3];
                }
            }
        }
    }

    return temp_lines;
}

std::vector<Vec4i> flip(const std::vector<Vec4i> lines, const std::vector<Vec3f> SS_points){
    std::vector<Vec4i> temp_lines = lines;
    Point temp;
    float dist1, dist2;

    for(size_t i = 0; i < lines.size(); i++){

        dist1 = calc_dist(Point(lines[i][0], lines[i][1]), Point(lines[i - 1][2], lines[i - 1][3]));
            dist2 = calc_dist(Point(lines[i][2], lines[i][3]), Point(lines[i - 1][2], lines[i - 1][3]));

            if(dist1 < dist2){
                continue;
            }
            else{
                temp.x = lines[i][0];
                temp.y = lines[i][1];

                temp_lines[i][0] = lines[i][2];
                temp_lines[i][1] = lines[i][3];

                temp_lines[i][2] = temp.x;
                temp_lines[i][3] = temp.y;
            }
    }

    return temp_lines;
}

std::vector<Vec4i> within_prox(Vec4i line, std::vector<Vec4i> lines, int prox_val){
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

double closest_dist(Vec4i line, std::vector<Vec4i> lines){
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

std::pair<Vec4i, double> ex_closest_line(Vec4i cur_line, std::vector<Vec4i> lines, double scale_val){
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

Vec4i closest_line(Vec4i line, std::vector<Vec4i> lines){
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

std::vector<Vec4i> find_corner_lines(const Vec4i& current_line, const std::vector<Vec4i>& lines, double angle_threshold = 30.0){
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

int return_id(Vec4i line, std::vector<Vec4i> lines){
    int id = -1;

    for(size_t i = 0; i < lines.size(); i++){
        if((line == lines[i]) || (lines[i] == Vec4i(line[2], line[3], line[0], line[1]))){
            id = i;

            break;
        }
    }

    return id;
}

Vec4i orient(Vec4i line, Vec4i next_line){
    double dist1, dist2;

    dist1 = calc_dist(Point(line[2], line[3]), Point(next_line[0], next_line[1]));
    dist2 = calc_dist(Point(line[2], line[3]), Point(next_line[2], next_line[3]));

    if(dist2 < dist1){
        return Vec4i(next_line[2], next_line[3], next_line[0], next_line[1]);
    }

    return next_line;
}

bool is_line_between(const Vec4i line1, const Vec4i line2, const Vec4i candidate_line, std::vector<Vec4i> lines, Mat img, int tolerance = 15){
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

        Vec4i temp_line = closest_line(line1, lines);

        // Make sure the candidate line is the line which we are trying to add. If lines are not same angle, it can cause issues.
        if((candidate_line == temp_line) && (line1 != temp_line) && (line1 != Vec4i(temp_line[2], temp_line[3], temp_line[0], temp_line[1])) && (line2 != temp_line) && (line2 != Vec4i(temp_line[2], temp_line[3], temp_line[0], temp_line[1]))){
            return true;
        }
    }

    return false; // Candidate line does not have any points between line1 and line2
}

Vec4i find_line_between(const Vec4i line1, const Vec4i line2, const std::vector<Vec4i> lines, Mat img){
    
    for(const auto candidate_line : lines){
        // Check if the candidate line is between line1 and line2
        if(is_line_between(line1, line2, candidate_line, lines, img)){
            return candidate_line; // Return the first line found that is between line1 and line2
        }
    }

    // If no line is found, return an invalid line
    return Vec4i(-1, -1, -1, -1);
}

std::pair<double, int> corner_check(const Vec4i trgt_line, const std::vector<Vec4i> lines){
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

bool check_end(Vec4i line, std::vector<Vec3f> SS_points){
    if(calc_dist(Point(line[2], line[3]), Point(SS_points[1][0], SS_points[1][1])) < 150){
        return true;
    }

    return false;
}

Vec4i find_end_line(std::vector<Vec4i> lines, std::vector<Vec3f> SS_points){
    
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

std::vector<Vec4i> sort(std::vector<Vec4i> lines, const std::vector<Vec3f> SS_points, Mat img){
    std::vector<Vec4i> temp_lines, result;
    Vec4i new_trgt, trgt_line, temp_line, line_between, end_line = find_end_line(lines, SS_points);
    std::pair<Vec4i, double> temp_pair;
    std::pair<double, int> inter_pair;
    double end_dist;
    bool remove_line = false;
    int prev_result_size = 0, counter = 0;

    int start_id = find_start_line(lines, SS_points), full = lines.size();

    // Orient the starting line
    result.push_back(orient(Vec4i(0, 0, SS_points[0][0], SS_points[0][1]), lines[start_id]));

    // Remove it from lines vector
    lines.erase(lines.begin() + start_id);

    do{
        for(size_t j = 0; j < lines.size(); j++){
            line(img, Point(lines[j][0], lines[j][1]), Point(lines[j][2], lines[j][3]), Scalar(255,0,0), 3, LINE_AA);
            namedWindow("sort", WINDOW_NORMAL);
            imshow("sort", img);
        }

        trgt_line = result.back();

        line(img, Point(trgt_line[0], trgt_line[1]), Point(trgt_line[2], trgt_line[3]), Scalar(0,255,0), 3, LINE_AA);

        if(lines.size() > 1){
            inter_pair = corner_check(trgt_line, lines);

            // If a intersection point is found within 100 units of the target line, then there is a corner
            if((inter_pair.first != -1) && (inter_pair.first < 100)){
                temp_line = orient(trgt_line, lines[inter_pair.second]);

                result.push_back(temp_line);

                line(img, Point(temp_line[0], temp_line[1]), Point(temp_line[2], temp_line[3]), Scalar(0,0,255), 3, LINE_AA);

                namedWindow("sort", WINDOW_NORMAL);
                imshow("sort", img);
                waitKey(0);

                lines.erase(lines.begin() + return_id(temp_line, lines));

                if(check_end(temp_line, SS_points)){
                    break;
                }
                else{
                    trgt_line = result.back();
                }
            }
        }

        temp_lines = find_lines_with_similar_angle(trgt_line, lines);

        for(size_t j = 0; j < temp_lines.size(); j++){
            line(img, Point(temp_lines[j][0], temp_lines[j][1]), Point(temp_lines[j][2], temp_lines[j][3]), Scalar(255,0,0), 3, LINE_AA);

            namedWindow("sort", WINDOW_NORMAL);
            imshow("sort", img);
        }

        waitKey(0);

        temp_lines = within_prox(trgt_line, temp_lines, 350);

        if(temp_lines.size() != 0){
            for(size_t i = 0; i < temp_lines.size(); i++){
                line(img, Point(temp_lines[i][0], temp_lines[i][1]), Point(temp_lines[i][2], temp_lines[i][3]), Scalar(255,255,0), 3, LINE_AA);

                namedWindow("sort", WINDOW_NORMAL);
                imshow("sort", img);
            }

            waitKey(0);

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

            if(check_end(new_trgt, SS_points)){
                break;
            }
            else{
                trgt_line = result.back();
                remove_line = true;
            }
        }

        line(img, Point(new_trgt[0], new_trgt[1]), Point(new_trgt[2], new_trgt[3]), Scalar(255,255,255), 3, LINE_AA);

        namedWindow("sort", WINDOW_NORMAL);
        imshow("sort", img);

        if(result.size() >= 2){

            Vec4i line_between = find_line_between(result[result.size()-2], result[result.size()-1], lines, img);

            if(line_between != Vec4i(-1, -1, -1, -1)){

                line(img, Point(line_between[0], line_between[1]), Point(line_between[2], line_between[3]), Scalar(0,0,255), 3, LINE_AA);

                namedWindow("sort", WINDOW_NORMAL);
                imshow("sort", img);
                waitKey(0);

                line_between = orient(result[result.size()-2], line_between);

                temp_line = result[result.size()-1];
                result[result.size()-1] = line_between;
                result.push_back(temp_line);

                lines.erase(lines.begin() + return_id(line_between, lines));
            }
        }

        if(remove_line){

            line(img, Point(trgt_line[0], trgt_line[1]), Point(trgt_line[2], trgt_line[3]), Scalar(0,0,255), 3, LINE_AA);

            namedWindow("sort", WINDOW_NORMAL);
            imshow("sort", img);
            waitKey(0);

            lines.erase(lines.begin() + return_id(trgt_line, lines));
            remove_line = false;
        }

        namedWindow("sort", WINDOW_NORMAL);
        imshow("sort", img);

        waitKey(0);

        if(check_end(new_trgt, SS_points)){
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

    for(int i = 0; i < result.size(); i++){
        line(img, Point(result[i][0], result[i][1]), Point(result[i][2], result[i][3]), Scalar(0,0,255), 3, LINE_AA);
        circle(img, Point(result[i][0], result[i][1]), 5, Scalar(0,255,0), -1);
        circle(img, Point(result[i][2], result[i][3]), 5, Scalar(0,0,255), -1);

        namedWindow("sort", WINDOW_NORMAL);
        imshow("sort", img);

        waitKey(0);
    }

    std::vector<Point> corners;

    // Connect corners and lines together
    for(int i = 1; i < result.size(); i++){

        if(i == 1){
            result[i-1][0] = SS_points[0][0];
            result[i-1][1] = SS_points[0][1];
        }
        else{            
            result[i-1][2] = result[i][0];
            result[i-1][3] = result[i][1];
        }

        if(i == result.size()-1){
            result[i][2] = SS_points[1][0];
            result[i][3] = SS_points[1][1];
        }
        else if(corner_between(result[i], result[i+1], 15)){
            corners.push_back(calc_inter(result[i], result[i+1]));

            result[i-1][2] = corners.back().x;
            result[i-1][3] = corners.back().y;
            result[i][0] = corners.back().x;
            result[i][1] = corners.back().y;
        }
    }

    for(size_t i = 0; i < result.size(); i++){
        line(img, Point(result[i][0], result[i][1]), Point(result[i][2], result[i][3]), Scalar(255,0,255), 3, LINE_AA);
    }

    namedWindow("sort", WINDOW_NORMAL);
    imshow("sort", img);

    waitKey(0);

    return result;
}

int calc_angle(const Vec4i line1, const Vec4i line2){

    Point line1_start(line1[0], line1[1]);
    Point line2_start(line2[0], line2[1]);

    Point line1_end(line1[2], line1[3]);
    Point line2_end(line2[2], line2[3]);

    Point dir_line1 = line1_end - line1_start;
    Point dir_line2 = line2_end - line2_start;

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

    return angle;
}

bool corner_between(const Vec4i line1, const Vec4i line2, const int angle_limit){

    double angle, angle1, angle2;

    angle1 = calc_angle(line1);
    angle2 = calc_angle(line2);

    angle = std::abs(angle1 - angle2);

    if((angle > angle_limit) && (angle < 180 - angle_limit)){
        return true;
    }
    
    return false;
}

Vec4i extend_line(const Vec4i line, const float scale){
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
    return Vec4i(line_start.x, line_start.y, new_end.x, new_end.y);
}

Point calc_inter(const Vec4i line1, const Vec4i line2) {
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