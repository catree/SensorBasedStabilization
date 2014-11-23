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
    VideoStabilization a("20141115_225315");
    a.smooth();
    a.show();
    a.output();
    waitKey(0);
    return 0;
}

