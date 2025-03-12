#include <iostream>
#include <vector>
#include <cmath>

#include "imagehandler.h"
#include "linedetecter.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//#include "functions.h"

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
    Mat img = imread("/home/aksel/Documents/GitHub/Control-and-regulation-of-robotic-systems/Vision/Cpp - Project/test_img_ended.jpg", IMREAD_COLOR);

    //Check if image was read properly.
    if (img.empty()) { 
        cout << "Image File "
             << "Not Found" << endl; 
        cin.get(); 
        return -1; 
    } 

    lineDetecter line_detection(img);

    return 0; 
}
