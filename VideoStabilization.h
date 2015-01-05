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

using namespace cv;
using namespace std;


class VideoStabilization {
private:
    const double d = 0.6;
    const double alphaMin = 0.6;
    const double cropPercent = 0.15, innerPercent = 0.025;
    const int beta = 1.2;

    const int captureWidth = 544, captureHeight = 960;
    const double frameRate = 30, sensorRate = 100;
    const double fuvX = 793, fuvY = 793;
    Mat K;
    Mat outputFrame, cropFrame;

    int frames;
    string name;
    vector<int> timestamps;
    vector<double> angvX, angvY, angvZ;
    vector<EulerAngles> rotAngles;
    vector<Quaternion> p, v;
    vector<double> alpha;

    Quaternion angleToQuaternion(double angX, double angY, double angZ);

    double computeAlpha(const EulerAngles &rotAngle);

    EulerAngles computeRotation(const Quaternion &v, const Quaternion &p);

public:
    void rotate(const Mat &src, Mat &dst, const Mat &R);

    Mat rotationMat(EulerAngles rotAngle);

    VideoStabilization(string videoName);

    void smooth();

    void show();

    bool output();
};


#endif