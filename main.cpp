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

using namespace cv;
using namespace std;

int main() {
    namedWindow("ha");
    VideoStabilization a("20141207_151016");
    a.smooth();
    a.show();

    a.output();
    //waitKey(0);
//    Mat b = imread("1.jpg"), b1;
//    a.rotate(b, b1, a.rotationMat(EulerAngles(0, 0.1, 0)));
//    imshow("ha", b1);
//    waitKey(0);
    return 0;
}

