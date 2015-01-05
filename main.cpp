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
    //const string videoName = "20150104_105709";
    const string videoName = "20150104_145554";
    VideoStabilization a(videoName);
    /*a.smooth();
    a.show();
    a.output();*/

//    test(videoName);

/*    Mat b = imread("1.jpg"), b1;
    a.rotate(b, b1, a.rotationMat(EulerAngles(0.1, 0, 0.1)));
    a.rotate(b, b1, a.rotationMat(EulerAngles(-0.1, 0, -0.1)));
//    a.rotate(b1, b1, a.rotationMat(EulerAngles(0, 0, -0.1)));
//    a.rotate(b1, b1, a.rotationMat(EulerAngles(-0.1, 0, 0)));
    imshow("ha", b1);
    waitKey(0);*/
/*    Quaternion q, q1, q2, q3, q4, q5;
    Matrix4x3 m;
    EulerAngles e;
    q.setToRotateInertialToObject(EulerAngles(1, 0.5, 0));
    e.fromInertialToObjectQuaternion(q);
    q.w = 0.848, q.x = 0.237, q.y = 0.474, q.z = 0;
    m.fromQuaternion(q);
    q1.setToRotateInertialToObject(EulerAngles(-0.5, -0.2, 0));
    q2.setToRotateInertialToObject(EulerAngles(-0.5, 0, 0));
    q3.setToRotateInertialToObject(EulerAngles(0, -0.2, 0));
    q4 = q * q1;
    q5 = q * q3 * q2;*/
    return 0;
}

