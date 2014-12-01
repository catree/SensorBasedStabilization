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
    VideoStabilization a("20141123_214501");
    a.smooth();
    a.show();
    //Quaternion q;
    //q.setToRotateInertialToObject(EulerAngles(0.1, 0.2, 0.2));
    //cout << q.w << q.x << q.y << q.z;

    a.output();
    //waitKey(0);
    //Mat b = imread("1.jpg");
    return 0;
}

