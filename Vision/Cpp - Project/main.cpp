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

/*--------- Testing Ideas -------------
    Using skeletonization:
    - Varrying threshold function lower limit (Upper should always be 255)
    - Elimination of Hough lines shorter than x length
        - Increasing minimum length of hough lines, while increasing amount of dilation - meaning more coherrency of lines
    - Filling out gaps made after eliminating "too short" line segments
        - Using bezier curve
        - Finding intersecting lines, that deviate in angle, more than some value (ex. 15 degrees).
            - Lines should be lengthened, such that they intersect.
        ^(Each of the two above methods may have pros and cons, in terms of figures they best describe)
            ^^Should be tested.
*/

/* --------- Improvements ------------
    - Since the camera will be moving with the pan-tilt system, meaning there will be no change in relative
      position and angle, it is possible to record the change in position, of the center point of the ball, 
      for each frame. Using the frame rate of the video stream it is possible to determine the amount of time between
      each frame, meaning velocity for the ball can be calculated. The direction of the movement can also be calculated
      by making a vector from the last recorded point to the new point. A ratio describing pixel to real world distance
      is needed. It is also crucial that distortion in the video is decreased as much as possible.
*/

/* -------- Considerations ----------
    - To find the start and end points of the "track", they should either be placed at specific
    points (Top left and bottom right) or be marked with some form of marker (ex. colored dot)
    - It is neccessary for the vector of lines to be sorted, such that each line comes in 
    chronological order, in terms of the path/route the ball would travel. This is neccesary
    such that waypoints/commands come in correct sequence.
        - Might need to project the lines to be longer, to fill gaps from eliminated lines,
        in order to sort the lines in chronological order.
    
    - While having end points in red and green, it is important to notice that red is within the 
    threshold values (currently used), and it can therefore be detected by houghlineP, which is 
    not ideal. We will therefore record the position of the start/stop points, and color them white
    afterwards.
*/

int main(int argc, char** argv) 
{ 
    Mat img = imread("/home/mads-hyrup/Documents/Uni/4.Semester/Cpp - Project/test_img_ended.jpg", IMREAD_COLOR); 

    //Check if image was read properly.
    if (img.empty()) { 
        cout << "Image File "
             << "Not Found" << endl; 
        cin.get(); 
        return -1; 
    } 

    //Display image after being read
    namedWindow("Window1", WINDOW_GUI_NORMAL);
    imshow("Window1", img);

    // Initialize Mat types for making masks for blue and red - To find start and stop points
    Mat green_img, blue_img, red_img, output;

    // Initialize vector which will hold "lines" - coordinates for start and end points.
    std::vector<Vec4i> lines;

    // Read mask for red and blue colors into Mat type
    red_img = isolate_red(img);
    green_img = isolate_green(img);
    blue_img = isolate_blue(img);

    // Use hough circles algorithm to find coordinates for start and end points - save in SS_points
    std::vector<Vec3f> SS_points;
    detect_SS(green_img, SS_points);
    detect_SS(red_img, SS_points);

    // Remove the start and stop identifiers from image (white out)
    rmv_SS(img, SS_points);

    // Transform img to gray scale, changes amount of color channels for the Mat type
    cvtColor(img, img, cv::COLOR_BGR2GRAY);

    // Perform skeletonization on the image
    img = perform_skeletonization(img);

    // Dilate the image afterwards - Reduces the amount of fragmented segments in the skeleton
    perform_dilate(img);

    cvtColor(img, output, COLOR_GRAY2BGR);

    // Detect the lines in img
    lines = detect_lines(img);

    // Sort the detected lines
    lines = sort(lines, SS_points);

    for(size_t i = 0; i < lines.size(); i++){
        line(output, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, LINE_AA);
    }

    namedWindow("Window2", WINDOW_GUI_NORMAL);
    imshow("Window2", output);

    std::vector<Point> inters = find_inters(lines);

    std::vector<Vec4i> comp_path = handle_inters(lines, inters);

    for(size_t i = 0; i < comp_path.size(); i++){
        line(output, Point(comp_path[i][0], comp_path[i][1]), Point(comp_path[i][2], comp_path[i][3]), Scalar(0,0,255), 3, LINE_AA);
    }

    for(size_t i = 0; i < comp_path.size(); i++){
        circle(output, Point(comp_path[i][0], comp_path[i][1]), 30, Scalar(0,255,0), -1);

        namedWindow("output", WINDOW_NORMAL);
        imshow("output", output);
        waitKey(0);

        circle(output, Point(comp_path[i][2], comp_path[i][3]), 30, Scalar(0,0,255), -1);

        namedWindow("output", WINDOW_NORMAL);
        imshow("output", output);
        waitKey(0);
    }

    for(size_t i = 0; i < SS_points.size(); i++){
        circle(output, Point(SS_points[i][0], SS_points[i][1]), 30, Scalar(255,255,0), -1);
    }

    for(size_t i = 0; i < inters.size(); i++){
        if(inters[i] == Point(-1,-1)){
            continue;
        }
        else{
            circle(output, inters[i], 30, Scalar(255,0,0), -1);
        }
    }

    Vec4i templine;

    templine = perp_line(lines[1], 50);

    line(output, Point(templine[0], templine[1]), Point(templine[2], templine[3]), Scalar(255,0,0), 5);

    namedWindow("output", WINDOW_NORMAL);
    imshow("output", output);

    waitKey(0);
    destroyAllWindows();
    return 0; 
}