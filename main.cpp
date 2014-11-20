/*
 * main.cpp
 *
 *  Created on: Oct 31, 2014
 *      Author: tong
 */

#include<opencv/cv.h>
#include<opencv/highgui.h>
#include"VideoStabilization.h"
#include "Quaternion.h"
#include "MathUtil.h"

using namespace cv;
using namespace std;

int main() {
    VideoStabilization a("20141115_225315");
    a.smooth();
    a.show();
    a.output();
    /*stringstream ss;
    string s;
    ss << 1 << "sd";
    ss >> s;
    cout << s;*/
    /*Quaternion quaternion, quaternion1, quaternion2;
    quaternion1.identity();
    quaternion.setToRotateInertialToObject(EulerAngles(kPi/2,0,0));

    //quaternion1.setToRotateAboutY(kPi/4);
    //quaternion2 = slerp(quaternion, quaternion1,0);
    EulerAngles eulerAngles;
    eulerAngles.fromInertialToObjectQuaternion(quaternion);
    cout << quaternion.getRotationAngle() << " " << quaternion1.getRotationAngle() <<
            " " << eulerAngles.heading;*/
    //namedWindow("1");
    //waitKey(0);
    return 0;
}

