#include "line_detection.h"

Point get_scrn_param(){

    /*
        Description:
        Gets the paramters of the screen - Height and width in terms of pixels. Is used for error
        detection in function "find_all_inters".
    */

    Display* display = XOpenDisplay(NULL);

    Screen* screen = DefaultScreenOfDisplay(display);

    return Point(screen->width, screen->height);
}

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

int find_start_line(std::vector<Vec4i> lines, std::vector<Vec3f> SS_points){

    /*
        Description:
        Calculates and returns the element id of the first line segment in vector lines.
        The first line segment is defined as the line segment closest to the start identifier
    */

    int temp_val;
    
    Point start_pnt(SS_points[0][0], SS_points[0][1]);

    int temp_id, temp_dist, min_dist = INT_MAX;

    for(size_t i = 0; i < lines.size(); i++){
        temp_dist = calc_dist(start_pnt, Point(lines[i][0], lines[i][1]));

        if(temp_dist < min_dist){
            min_dist = temp_dist;
            temp_id = i;
        }
    }

    return temp_val;
}

std::vector<Vec4i> sort_lines(const std::vector<Vec4i> lines, const int start_id){
    std::vector<Vec4i> sorted_lines, temp_lines = lines;
    Point temp_pnt, temp_target_pnt;
    int temp_dist, min_dist, temp_id, target_id = start_id;

    for(int i = 0; i < lines.size(); i++){
        min_dist = INT_MAX;
        temp_target_pnt = Point(lines[target_id][2], lines[target_id][3]);

        for(size_t j = 0; j < temp_lines.size(); j++){
            temp_pnt = Point(temp_lines[j][0], temp_lines[j][1]);
            temp_dist = calc_dist(temp_target_pnt, temp_pnt);
            
            if((min_dist > temp_dist)){
                min_dist = temp_dist;
                temp_id = j;
            }
        }

        for(size_t i = 0; i < lines.size(); i++){
            if(lines[i] == temp_lines[temp_id]){
                target_id = i;
                sorted_lines.push_back(lines[i]);
                temp_lines.erase(temp_lines.begin() + temp_id);
                break;
            }
        }
    }

    return sorted_lines;
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

std::vector<Vec4i> detect_lines(const Mat img){

    std::vector<Vec4i> temp_lines; // will hold the results of the detection
    HoughLinesP(img, temp_lines, 1, CV_PI/180, 50, 50, 10); // runs the actual detection

    eliminate_small_lines(temp_lines, 180);

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

    std::cout << "size: " << lines.size() << std::endl;

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

            std::cout << "found no corner between, i: " << i << std::endl;
            
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

        /*if(i == 0){

            Point line_pnt1(lines[i][0], lines[i][1]);
            Point line_pnt2(lines[i][2], lines[i][3]);

            Point start_pnt(SS_points[0][0], SS_points[0][1]);

            if(calc_dist(line_pnt1, start_pnt) < calc_dist(line_pnt2, start_pnt)){
                temp_lines[i][0] = start_pnt.x;
                temp_lines[i][1] = start_pnt.y;
            }
            else{
                temp.x = temp_lines[i][0];
                temp.y = temp_lines[i][1];

                temp_lines[i][0] = start_pnt.x;
                temp_lines[i][1] = start_pnt.y;

                temp_lines[i][2] = temp.x;
                temp_lines[i][3] = temp.y;
            }
        }
        /*else if(i == lines.size()-2){

            Point line_pnt1(lines[i][0], lines[i][1]);
            Point line_pnt2(lines[i][2], lines[i][3]);

            Point end_pnt(SS_points[1][0], SS_points[1][1]);

            if(calc_dist(line_pnt1, end_pnt) < calc_dist(line_pnt2, end_pnt)){
                temp.x = temp_lines[i][2];
                temp.y = temp_lines[i][3];

                temp_lines[i][2] = end_pnt.x;
                temp_lines[i][3] = end_pnt.y;

                temp_lines[i][0] = temp.x;
                temp_lines[i][1] = temp.y;
            }
            else{
                temp_lines[i][2] = end_pnt.x;
                temp_lines[i][3] = end_pnt.y;
            }
        }
        else{

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
        }*/

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

std::vector<Vec4i> sort(const std::vector<Vec4i> lines, const std::vector<Vec3f> SS_points){
    std::vector<Vec4i> temp_lines;

    int start_id = find_start_line(lines, SS_points);

    temp_lines = sort_lines(lines, start_id);

    temp_lines = conn_lines(temp_lines, SS_points);

    temp_lines = flip(temp_lines, SS_points);

    return temp_lines;
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
    if(calc_angle(line1, line2) > angle_limit){
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
    Point new_start = line_start - scaled_dir;
    Point new_end = line_end + scaled_dir;

    // Return the extended line
    return Vec4i(new_start.x, new_start.y, new_end.x, new_end.y);
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

std::vector<Point> find_inters(const std::vector<Vec4i> lines){

    Point inter;
    std::vector<Point> inters;
    std::vector<Vec4i> ext_lines = lines;

    for(size_t i = 0; i < lines.size()-1; i++){
        if(corner_between(lines[i], lines[i+1], 15)){
            ext_lines[i] = extend_line(lines[i], 1.2);
            ext_lines[i+1] = extend_line(lines[i+1], 1.2);
            inter = calc_inter(lines[i], lines[i+1]);
            inters.push_back(inter);
        }
        else{
            inters.push_back(Point(-1,-1));
        }
    }

    return inters;
}

std::vector<Vec4i> handle_inters(const std::vector<Vec4i> lines, const std::vector<Point> inters){
    std::vector<Vec4i> temp_lines = lines;

    for(size_t i = 0; i < inters.size(); i++){
        if(inters[i] == Point(-1,-1)){
            continue;
        }
        else{

            Point line1_pnt1(temp_lines[i][0], temp_lines[i][1]);
            Point line1_pnt2(temp_lines[i][2], temp_lines[i][3]);

            Point line2_pnt1(temp_lines[i+1][0], temp_lines[i+1][1]);
            Point line2_pnt2(temp_lines[i+1][2], temp_lines[i+1][3]);

            if(calc_dist(line1_pnt1, inters[i]) < calc_dist(line1_pnt2, inters[i])){
                temp_lines[i][0] = inters[i].x;
                temp_lines[i][1] = inters[i].y;
            }
            else{
                temp_lines[i][2] = inters[i].x;
                temp_lines[i][3] = inters[i].y;
            }

            if(calc_dist(line2_pnt1, inters[i]) < calc_dist(line2_pnt2, inters[i])){
                temp_lines[i+1][0] = inters[i].x;
                temp_lines[i+1][1] = inters[i].y;
            }
            else{
                temp_lines[i+1][2] = inters[i].x;
                temp_lines[i+1][3] = inters[i].y;
            }
        }
    }

    return temp_lines;
}