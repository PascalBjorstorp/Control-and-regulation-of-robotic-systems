#include "functions.h"

using namespace cv;

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