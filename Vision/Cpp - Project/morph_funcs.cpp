#include "morph_funcs.h"

Mat perform_skeletonization(const Mat img, const int threshold_value){
    threshold(img, img, threshold_value, 255, THRESH_BINARY_INV);

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

    return skel;
}

void perform_dilate(Mat& img){

    Mat element = getStructuringElement(MORPH_CROSS, Size(3,3));

    //Dilate image to limit the amount of line fractions
    morphologyEx(img, img, MORPH_DILATE, element);
}