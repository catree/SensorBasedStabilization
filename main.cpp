/*
 * main.cpp
 *
 *  Created on: Oct 31, 2014
 *      Author: tong
 */

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include"VideoStabilization.h"
#include "Quaternion.h"
#include "test.h"

using namespace cv;
using namespace std;

int main() {
    //string videoName = "20141222_170219";
    string videoName = "20141222_185445";
    VideoStabilization a(videoName);
    a.smooth();
    a.show();
    a.output();

//    test(videoName);

    //waitKey(0);
//    Mat b = imread("1.jpg"), b1;
//    a.rotate(b, b1, a.rotationMat(EulerAngles(0, 0.1, 0)));
//    imshow("ha", b1);
//    waitKey(0);
    return 0;
}

