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
#include "Matrix4x3.h"

using namespace cv;
using namespace std;

int main() {
    const string videoName = "20150126_154432";
    //const string videoName = "20141229_161612"; //z
    //const string videoName = "20150104_105644"; //x
    //const string videoName = "20150104_105709"; //y

    VideoStabilization a(videoName);
    a.smooth();
    a.show();
    a.output();

    /*Quaternion q = angle2Quaternion(-0.1, -0.2, -0.3);
    EulerAngles e = VideoStabilization::quaternionToAngle(q);
    cout << e.pitch << " " << e.heading << " " << e.bank;*/
    /*testAngle(videoName, "117");
    waitKey(0);*/

/*    Mat b = imread("1.jpg"), b1;
    a.rotate(b, b1,         a.rotationMat(EulerAngles(0.1, 0, 0.1)));
    a.rotate(b, b1, a.rotationMat(EulerAngles(-0.1, 0, -0.1)));
//    a.rotate(b1, b1, a.rotationMat(EulerAngles(0, 0, -0.1)));
//    a.rotate(b1, b1, a.rotationMat(EulerAngles(-0.1, 0, 0)));
    imshow("ha", b1);
    waitKey(0);*/
    return 0;
}

