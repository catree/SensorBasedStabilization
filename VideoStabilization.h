#ifndef VIDEO_STABILIZATION
#define VIDEO_STABILIZATION

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<fstream>
#include<vector>
#include<iostream>
#include<string>
#include"Quaternion.h"
#include "EulerAngles.h"
#include "Parameters.h"

using namespace cv;
using namespace std;

class VideoStabilization {
private:
    const double d = 0.6;
    const double alphaMin = 0.6;
    const double cropPercent = 0.15, innerPercent = 0.025;
    const double beta = 2;

    const int captureWidth, captureHeight;
    const float sensorRate;
    const double fuvX, fuvY;
    const string inputType;

    double frameRate;
    Mat K;
    Mat outputFrame, cropFrame;

    int frames;
    string name;
    vector<int> timestamps;
    vector<EulerAngles> rotAngles;
    vector<Quaternion> rotQuaternions;
    vector<Quaternion> p, v, pDelta;
    vector<double> alpha;

    Quaternion angleToQuaternion(double angX, double angY, double angZ);

    double computeAlpha(Quaternion rotQuaternion);

    void computeRotation(Quaternion const &v, Quaternion const &p, int index);

    void getFrameByJpg(Mat &frame, int index);

    void getFrameByMp4(Mat &frame);

public:
    static EulerAngles quaternionToAngle(Quaternion q);

    void rotate(const Mat &src, Mat &dst, const Mat &R);

    static Mat rotationMat(Quaternion rotQuaternion);

    VideoStabilization(string videoName, CameraParams cameraParams = cameraParamsXIAOMI4);

    void smooth();

    void show();

    bool output();

};


#endif