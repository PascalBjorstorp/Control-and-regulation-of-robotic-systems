#include "functions.h"

using namespace cv;

Point get_scrn_param(){
    Display* display = XOpenDisplay(NULL);

    Screen* screen = DefaultScreenOfDisplay(display);

    return Point(screen->width, screen->height);
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

    for(int i = 0; i < line_Vec.size(); i++){
        len = calc_line_len(line_Vec[i]);

        if(len <= min_len){
            line_Vec.erase(line_Vec.begin() + i);
            i--;
        }
    }
}

int find_start(const std::vector<Vec4i> line_Vec){

    /*
        Description:
        Finds the line segment that is furthest in the top left corner (Min y and x value),
        which is assumed to be the starting point.
    */

   //Point min should start as a value that is definitly larger than the start point.
   Point min(3000,3000);
   int start_segment_id;

    // Iterate through lines - Line with lowest coords (top left), ID should be returned
    for(int i = 0; i < line_Vec.size(); i++){
        // Check only for starting point of line.
        /*if(line_Vec[i][0] <= min.x){
            std::cout << "new x: " << line_Vec[i][0] << std::endl;
            min.x = line_Vec[i][0];
            start_segment_id = i;
        }*/


        if(((line_Vec[i][0] <= min.x) && (line_Vec[i][1] <= min.y))){
            min.x = line_Vec[i][0];
            min.y = line_Vec[i][1];
            start_segment_id = i;
        }
    }

    return start_segment_id;
}

Point calc_ctrl_point(const Point start_pnt, const Point end_pnt, float offset_fac){
    
    /*
        Description:
        Calculates a control point, from given start and end points, used for
        making a bezier curve connecting lines.
    */
    
    Point mid_pnt = (start_pnt + end_pnt)/2;
    Point dir = end_pnt - start_pnt;

    Point perpendicular(-dir.y, dir.x);

    double length = std::sqrt(perpendicular.x * perpendicular.x + perpendicular.y * perpendicular.y);
    Point offset_val = perpendicular * (offset_fac / length);

    // Calculate the control point
    Point ctrl_pnt = mid_pnt + offset_val;

    return ctrl_pnt;
}


// Should make a bezier curve between two points, if they are further than some value from each other.
void bezier_curve(Mat& output, std::vector<Vec4i> lines, int res, int range){
    Point start_pnt, end_pnt, ctrl_pnt, diff, temp;
    int dist;

    for(int i = 0; i < lines.size()-1; i++){
        start_pnt = Point(lines[i][2], lines[i][3]);
        end_pnt = Point(lines[i+1][0], lines[i+1][1]);
        diff = start_pnt - end_pnt;

        dist = sqrt((diff.x * diff.x) + (diff.y * diff.y));

        if(dist > range){
            ctrl_pnt = calc_ctrl_point(start_pnt, end_pnt, 10);

            for(int j = 0; j < res; j++){
                float p1 = j / (res - 1);
                float p2 = 1.0 - p1;

                temp = p2 * p2 * start_pnt + 2 * p2 * p1 * ctrl_pnt + p1 * p1 * end_pnt;
                circle(output, temp, 5, Scalar(0,0,255), -1);
            }
        }

    }
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

    // Should not be an issue, but better to catch it than not.
    try{
        if((line1_len == 0) || (line2_len == 0)){
            throw std::runtime_error("Length of line segment is zero");
        }
    }
    catch(const std::exception& e){
        std::cerr << "Error: " << e.what() << std::endl;
        exit(0);
    }

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

Point line_inter(const Vec4i line1, const Vec4i line2){

    Point line1_start(line1[0], line1[1]);
    Point line2_start(line2[0], line2[1]);

    Point line1_end(line1[2], line1[3]);
    Point line2_end(line2[2], line2[3]);
    
    // Calculate the direction vectors
    Point dir_line1 = line1_end - line1_start;
    Point dir_line2 = line2_end - line2_start;

    float denom = dir_line1.x * dir_line2.y - dir_line1.y * dir_line2.x;

    // Check if lines are parallel
    if (denom == 0) {
        return Point(-1,-1);
    }

    // Calculate the numerators
    float num1 = (line1_start.x - line2_start.x) * dir_line2.y - (line1_start.y - line1_start.y) * dir_line2.x;
    //float num2 = (line1_start.x - line2_start.x) * dir_line1.y - (line1_start.y - line1_start.y) * dir_line1.x;

    // Calculate the intersection parameters
    float t1 = num1 / denom;
    //float t2 = num2 / denom;

    // Calculate the intersection point

    Point x = line1_start + t1 * dir_line1;
    Point y = line2_start + t1 * dir_line2;

    Point intersection(x.x, y.y);
    //Point intersection = line1_start + t1 * dir_line1;

    // For checking if the lines would intersect if extended, we don't need to check the bounds
    return intersection;
}

std::vector<Point> find_all_inters(const std::vector<Vec4i> lines){

    Point temp;
    std::vector<Point> inters;

    Point screen_params = get_scrn_param();

    for(int i = 0; i < lines.size(); i++){
        for(int j = 0; j < lines.size(); j++){
            if(i != j){
                //Only calculate intersection for line segment pairs, that have a larger difference in angle than 15 degrees.
                if(calc_angle(lines[i], lines[j]) > 15){
                    temp = line_inter(lines[i], lines[j]);

                    if(((temp.x > 0) && (temp.x < screen_params.x)) && ((temp.y > 0) && (temp.y < screen_params.y))){
                        std::cout << "lines i: " << lines[i] << " lines[j]: " << lines[j] << std::endl;
                        std::cout << "intersect point: " << temp << std::endl;
                        inters.push_back(temp);
                    }
                }
            }
        }
    }

    return inters;
}