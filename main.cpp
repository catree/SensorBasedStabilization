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
    const string videoName = "20150116_110815";
    //const string videoName = "20141229_161612"; //z
    //const string videoName = "20150104_105644"; //x
    //const string videoName = "20150104_105709"; //y

    VideoStabilization a(videoName);
    a.smooth();
    a.show();
    a.output();
    /*testAngle(videoName, "70");
    waitKey(0);*/

/*    Mat b = imread("1.jpg"), b1;
    a.rotate(b, b1, a.rotationMat(EulerAngles(0.1, 0, 0.1)));
    a.rotate(b, b1, a.rotationMat(EulerAngles(-0.1, 0, -0.1)));
//    a.rotate(b1, b1, a.rotationMat(EulerAngles(0, 0, -0.1)));
//    a.rotate(b1, b1, a.rotationMat(EulerAngles(-0.1, 0, 0)));
    imshow("ha", b1);
    waitKey(0);*/
    /*Quaternion q, q1, q2, q3, q4, q5;
    Matrix4x3 m;
    EulerAngles e;
    q.setToRotateInertialToObject(EulerAngles(0, 0, 0.05));
    q*=q;
    q*=q;
    q*=q;
    q*=q;
    q*=q;
    q*=q;
    q*=q;
    q*=q;
    q*=q;
    q*=q;
    q*=q;
    q*=q;
    e.fromInertialToObjectQuaternion(q);*/
    return 0;
}

