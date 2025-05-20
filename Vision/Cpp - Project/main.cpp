#include <iostream>
#include <vector>
#include <cmath>

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "SS_detect.h"
#include "morph_funcs.h"
#include "line_detection.h"

using namespace std;
using namespace cv;

/*--------- Sources --------------
    - Skeletonization: http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/
    - Morphlogical functions: https://docs.opencv.org/3.4/db/df6/tutorial_erosion_dilatation.html
    - Canny: https://docs.opencv.org/3.4/da/d22/tutorial_py_canny.html
    - Hough lines: https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html?fbclid=IwZXh0bgNhZW0CMTEAAR3O16f3HZkb7_CG6jp1L6GrLG9qtdtHNTZlSUySlI17kJetv9hQsBpvz6s_aem_WRzuSBVr2zyhYlxOm4nu3Q
    - Line intersection: https://web.archive.org/web/20060911055655/http://local.wasp.uwa.edu.au/%7Epbourke/geometry/lineline2d/

*/

Point calc_midpoint(const Vec4i& line) {
    return Point((line[0] + line[2]) / 2, (line[1] + line[3]) / 2);
}

// Function to calculate the Euclidean distance between two points
double calc_distance(const Point& p1, const Point& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double calc_segment_distance(const Vec4i& line1, const Vec4i& line2) {
    Point p1(line1[0], line1[1]), q1(line1[2], line1[3]);
    Point p2(line2[0], line2[1]), q2(line2[2], line2[3]);

    // Helper function to calculate the squared distance between two points
    auto dist2 = [](const Point& a, const Point& b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    };

    // Helper function to calculate the projection of point c onto line segment ab
    auto point_to_segment_dist2 = [&](const Point& a, const Point& b, const Point& c) {
        double l2 = dist2(a, b); // Length of segment squared
        if (l2 == 0.0) return dist2(a, c); // a == b case
        double t = ((c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y)) / l2;
        t = max(0.0, min(1.0, t)); // Clamp t to [0, 1]
        Point projection(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
        return dist2(c, projection);
    };

    // Calculate the minimum distance squared between the two line segments
    double min_dist2 = min({
        point_to_segment_dist2(p1, q1, p2),
        point_to_segment_dist2(p1, q1, q2),
        point_to_segment_dist2(p2, q2, p1),
        point_to_segment_dist2(p2, q2, q1)
    });

    return sqrt(min_dist2); // Return the actual distance
}

double calc_avg_distance(const vector<Vec4i>& lines1, const vector<Vec4i>& lines2) {

    double total_distance = 0.0;

    for (const auto& line1 : lines1) {
        // Find the closest line in lines2
        double min_distance = DBL_MAX;
        for (const auto& line2 : lines2) {
            double distance = calc_segment_distance(line1, line2);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }

        // Add the minimum distance to the total
        total_distance += min_distance;
    }

    // Calculate the average distance
    return total_distance / lines1.size();
}

bool is_line_outside_scope(const Vec4i& line, const vector<Vec4i>& other_lines, double threshold) {
    double min_distance = DBL_MAX;

    // Calculate the minimum distance to any line in the other vector
    for (const auto& other_line : other_lines) {
        double distance = calc_segment_distance(line, other_line);
        if (distance < min_distance) {
            min_distance = distance;
        }
    }

    // Check if the minimum distance is above the threshold
    return min_distance > threshold;
}

int main(int argc, char** argv) 
{ 
    Mat img = imread("/home/mads-hyrup/Documents/Uni/4.Semester/Cpp - Project/TestPic/Cropped_board1.jpg", IMREAD_COLOR); 

    //Check if image was read properly.
    if (img.empty()) { 
        cout << "Image File "
             << "Not Found" << endl; 
        cin.get(); 
        return -1; 
    }

    // Initialize vector which will hold "lines" - coordinates for start and end points.
    std::vector<Vec4i> lines;

    std::vector<Point> inters;

    std::vector<Vec4i> comp_path;

    // Initialize Mat types for making masks for blue and red - To find start and stop points
    Mat green_img, blue_img, red_img, output, temp_img;

    double avg_distance;

    std::vector<Vec3f> SS_points;

    for(int i = 1; i < 6; i++){

        SS_points.clear();
        lines.clear();
        inters.clear();

        String id = "Cropped_board" + to_string(i); 

        String fileName = "/home/mads-hyrup/Documents/Uni/4.Semester/Cpp - Project/TestPic/" + id + ".jpg";
        cout << "------ " << id << " ------" << endl;

        img = imread(fileName, IMREAD_COLOR);

        green_img = isolate_green(img);
        red_img = isolate_red(img);

        namedWindow("green", WINDOW_NORMAL);
        imshow("green", green_img);

        namedWindow("red", WINDOW_NORMAL);
        imshow("red", red_img);

        detect_SS(green_img, SS_points);
        detect_SS(red_img, SS_points);

        rmv_SS(img, SS_points);

        namedWindow("rmv", WINDOW_NORMAL);
        imshow("rmv", img);

        cvtColor(img, img, cv::COLOR_BGR2GRAY);

        //perform_dilate(img);

        namedWindow("dilate", WINDOW_NORMAL);
        imshow("dilate", img);

        img = perform_skeletonization(img, 124);

        namedWindow("skel", WINDOW_NORMAL);
        imshow("skel", img);

        cvtColor(img, output, COLOR_GRAY2BGR);

        // Detect the lines in img
        lines = detect_lines(img, 3);

        cvtColor(img, temp_img, COLOR_GRAY2BGR);

        for(size_t i = 0; i < lines.size()-1; i++){
            line(temp_img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, LINE_AA);
        }

        namedWindow("lines", WINDOW_NORMAL);
        imshow("lines", temp_img);

        lines = sort(lines, SS_points, temp_img);

        cvtColor(img, temp_img, COLOR_GRAY2BGR);

        circle(temp_img, Point(SS_points[0][0], SS_points[0][1]), 20, Scalar(0,100,0), -1);

        //avg_distance = calc_avg_distance(comp_path, temp_lines[i]);
    }

    namedWindow("output", WINDOW_NORMAL);
    imshow("output", output);

    waitKey(0);
    destroyAllWindows();
    return 0; 
}