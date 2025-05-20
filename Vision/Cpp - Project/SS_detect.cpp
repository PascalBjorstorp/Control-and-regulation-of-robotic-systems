#include "SS_detect.h"

Mat isolate_red(const Mat img){

    Mat hsv, red_mask1, red_mask2, red_img;

    cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    
    inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), red_mask1); // Lower red
    inRange(hsv, Scalar(170, 100, 100), Scalar(180, 255, 255), red_mask2); // Upper red
    red_img = red_mask1 | red_mask2;

    return red_img;
}

Mat isolate_blue(const Mat img){
    Mat hsv, blue_img;

    cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    inRange(hsv, Scalar(100, 150, 50), Scalar(140, 255, 255), blue_img);

    return blue_img;
}

Mat isolate_green(const Mat img){
    Mat hsv, green_img;

    cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    inRange(hsv, Scalar(35, 100, 50), Scalar(85, 255, 255), green_img); // Correct green
    
    return green_img;
}


void detect_SS(const Mat img, std::vector<Vec3f>& SS_points){
    Mat temp;
    Vec3f fin_circ;
    std::vector<Vec3f> temp_circ;

    GaussianBlur(img, temp, Size(9, 9), 2, 2);

    HoughCircles(temp, temp_circ, HOUGH_GRADIENT, 1, 1, 100, 30, 0, 0);

    for(int i = 0; i < temp_circ.size(); i++){
        if(temp_circ[i][2] > fin_circ[2]){
            fin_circ = temp_circ[i];
        }
    }

    SS_points.push_back(fin_circ);
}

void rmv_SS(Mat& img, const std::vector<Vec3f> SS_points){
    for(int i = 0; i < SS_points.size(); i++){
        circle(img, Point(SS_points[i][0], SS_points[i][1]), SS_points[i][2] + 10, Scalar(255,255,255), -1);
    }
}