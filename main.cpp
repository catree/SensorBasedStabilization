/*
 * main.cpp
 *
 *  Created on: Oct 31, 2014
 *      Author: tong
 */

#include<opencv/cv.h>
#include<opencv/highgui.h>
#include"VideoStabilization.h"

using namespace cv;
using namespace std;

int main() {
	VideoStabilization a("20141115_225140");
	a.show();
    //namedWindow("1");
    //waitKey(0);
    return 0;
}

