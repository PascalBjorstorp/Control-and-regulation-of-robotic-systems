#include <iostream>
#include <vector>
#include <cmath>

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "functions.h"

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
*/

int main(int argc, char** argv) 
{ 
    Mat img = imread("/home/mads-hyrup/Documents/Uni/4.Semester/Cpp - Project/test_img.jpg", IMREAD_GRAYSCALE); 
    Mat test_image = Mat::zeros(10, 10, CV_8UC1);

    //Check if image was read properly.
    if (img.empty()) { 
        cout << "Image File "
             << "Not Found" << endl; 
        cin.get(); 
        return -1; 
    } 

    //Display image after being read
    //namedWindow("Window1", WINDOW_GUI_NORMAL);
    //imshow("Window1", img);

    threshold(img, img, 145, 255, THRESH_BINARY_INV);

    //Display threshold output image
    //namedWindow("threshold", WINDOW_NORMAL);
    //imshow("threshold", img);

    //Declare variables with correct color channels - 8 bit 1 color
    Mat skel(Mat::zeros(img.size(), CV_8UC1)), temp(Mat::zeros(img.size(), CV_8UC1)), eroded(Mat::zeros(img.size(), CV_8UC1));

    //Declare element - Matrix used for certain functions
    Mat element = getStructuringElement(MORPH_CROSS, Size(3,3));

    //Perform skeletonization of image
    do{
        erode(img, eroded, element);
        dilate(eroded, temp, element);
        subtract(img, temp, temp);
        bitwise_or(skel, temp, skel);
        eroded.copyTo(img);
    
    }while(countNonZero(img) != 0);

    //Show skeleton image output
    //namedWindow("Skeleton", WINDOW_GUI_NORMAL);
    //imshow("Skeleton", skel);

    //Copy skeleton line image (output), to img - Better for my brain 
    skel.copyTo(img);

    //Dilate image to limit the amount of line fractions
    morphologyEx(img, img, MORPH_DILATE, element);

    //Show dilated for referecing
    //namedWindow("dilate", WINDOW_GUI_NORMAL);
    //imshow("dilate", img);

    //Declare Mat values for output showing - Use cvt to make correct amount of channels.
    Mat output, test_output;
    cvtColor(img, output, COLOR_GRAY2BGR);
    cvtColor(img, test_output, COLOR_GRAY2BGR);

    // Probabilistic Hough Line Transform
    vector<Vec4i> lines, test_lines; // will hold the results of the detection
    HoughLinesP(img, lines, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection

    // To show what it looks like without limiting line length
    HoughLinesP(img, test_lines, 1, CV_PI/180, 50, 50, 10 );

    eliminate_small_lines(lines, 180); 
    
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    }

    for( size_t i = 0; i < test_lines.size(); i++ )
    {
        Vec4i l = test_lines[i];
        line( test_output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    }

    /*namedWindow("Lines - Limited", WINDOW_NORMAL);
    imshow("Lines - Limited", output);*/

    namedWindow("Lines - not limited", WINDOW_NORMAL);
    imshow("Lines - not limited", test_output);

    namedWindow("Lines - Limited", WINDOW_NORMAL);
    imshow("Lines - Limited", output);
    
    //testing to see if lines appear in sorted order - They do not
    /*for(int i = 0; i < lines.size(); i++){

        Point temp(lines[i][2], lines[i][3]);
        circle(output, temp, 50, Scalar(0,0,255), -1);
        
        namedWindow("Highlight end point", WINDOW_NORMAL);
        imshow("Highlight end point", output);
        waitKey(0);
    }*/

    /*int starting_pnt;

    starting_pnt = find_start(lines);

    circle(output, Point(lines[starting_pnt][0], lines[starting_pnt][1]), 50, Scalar(255,0,0), -1);
    circle(output, Point(lines[starting_pnt][2], lines[starting_pnt][3]), 50, Scalar(255,0,0), -1);*/

    /*bezier_curve(output, lines, 10, 100);

    namedWindow("Bezier", WINDOW_NORMAL);
    imshow("Bezier", output);*/

    vector<Point> inters;

    inters = find_all_inters(lines);

    for(int i = 0; i < inters.size(); i++){
        circle(output, inters[i], 50, Scalar(255,0,0), -1);
    }

    namedWindow("Inters", WINDOW_NORMAL);
    imshow("Inters", output);

    Mat image;
    cvtColor(test_image, image, COLOR_GRAY2BGR);

    vector<Vec4i> int_test;

    int_test.push_back(Vec4i(3,5,7,5));
    int_test.push_back(Vec4i(6,1,6,6));

    for(int i = 0; i < int_test.size(); i++){
        line(image, Point(int_test[i][0], int_test[i][1]), Point(int_test[i][2],int_test[i][3]), Scalar(0,0,255), 1);
    }

    namedWindow("int - test", WINDOW_NORMAL);
    imshow("int - test", image);

    inters = find_all_inters(int_test);

    for(int i = 0; i < inters.size(); i++){
        std::cout << "int point: " << inters[i] << std::endl;
        circle(image, inters[i], 1, Scalar(128,0,128), -1);
    }

    namedWindow("int - test - int", WINDOW_NORMAL);
    imshow("int - test - int", image);

    //std::cout << "inters: " << inters << std::endl;

    waitKey(0);
    destroyAllWindows();
    return 0; 
}