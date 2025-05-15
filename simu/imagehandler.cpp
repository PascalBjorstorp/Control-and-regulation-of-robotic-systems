#include "imagehandler.h"

imageHandler::imageHandler(cv::Mat img):_img(img)
{
    cv::Mat normalImg = img;
    isolate_green();
    isolate_blue();
    isolate_red();
    //detect_SS(&_green_img);
    //detect_SS(&_red_img);
    //rmv_SS();
    cvtColor(_img, _img, cv::COLOR_BGR2GRAY);
    perform_skeletonization();
    perform_dilate();
}

void imageHandler::isolate_red(){

    cv::Mat hsv, red_mask1, red_mask2;

    cv::cvtColor(_img, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask1); // Lower red
    cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), red_mask2); // Upper red
    _red_img = red_mask1 | red_mask2;
}

void imageHandler::isolate_blue(){
    cv::Mat hsv;

    cv::cvtColor(_img, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(100, 150, 50), cv::Scalar(140, 255, 255), _blue_img);
}

void imageHandler::isolate_green(){
    cv::Mat hsv;

    cvtColor(_img, hsv, cv::COLOR_BGR2HSV);

    inRange(hsv, cv::Scalar(35, 100, 50), cv::Scalar(85, 255, 255), _green_img);
}
/*
void imageHandler::detect_SS(cv::Mat *img){
    cv::Mat temp;
    cv::Vec3f fin_circ;
    std::vector<cv::Vec3f> temp_circ;

    GaussianBlur(*img, temp, cv::Size(9, 9), 2, 2);

    HoughCircles(temp, temp_circ, cv::HOUGH_GRADIENT, 1, 1, 100, 30, 0, 0);

    for(int i = 0; i < temp_circ.size(); i++){
        if(temp_circ[i][2] > fin_circ[2]){
            fin_circ = temp_circ[i];
        }
    }

    _SS_points.push_back(fin_circ);
}
*/
/*
void imageHandler::rmv_SS(){
    for(int i = 0; i < _SS_points.size(); i++){
        circle(_img, cv::Point(_SS_points[i][0], _SS_points[i][1]), _SS_points[i][2] + 10, cv::Scalar(255,255,255), -1);
    }
}
*/
void imageHandler::perform_skeletonization(){
    threshold(_img, _img, _threshhold, 255, cv::THRESH_BINARY_INV);

    //Declare variables with correct color channels - 8 bit 1 color
    cv::Mat skel(cv::Mat::zeros(_img.size(), CV_8UC1)), temp(cv::Mat::zeros(_img.size(), CV_8UC1)), eroded(cv::Mat::zeros(_img.size(), CV_8UC1));

    //Declare element - Matrix used for certain functions
    cv::Mat element = getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));

    //Perform skeletonization of image
    do{
        erode(_img, eroded, element);
        dilate(eroded, temp, element);
        subtract(_img, temp, temp);
        bitwise_or(skel, temp, skel);
        eroded.copyTo(_img);

    }while(countNonZero(_img) != 0);

    _img = skel;
}

void imageHandler::perform_dilate(){

    cv::Mat element = getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));

    //Dilate image to limit the amount of line fractions
    morphologyEx(_img, _img, cv::MORPH_DILATE, element);
}
