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
    const double fuvX = 799, fuvY = 799;
    Mat K;
    Mat outputFrame, cropFrame;

    int frames;
    string name;
    vector<int> timestamps;
    vector<double> angvX, angvY, angvZ;
    vector<EulerAngles> rotAngles;
    vector<Quaternion> rotQuaternions;
    vector<Quaternion> p, v;
    vector<double> alpha;

    Quaternion angleToQuaternion(double angX, double angY, double angZ);

    double computeAlpha(Quaternion rotQuaternion);

    void computeRotation(Quaternion const &v, Quaternion const &p, int index);

public:
    void rotate(const Mat &src, Mat &dst, const Mat &R);

    static Mat rotationMat(Quaternion rotQuaternion);

    VideoStabilization(string videoName);

    void smooth();

    void show();

    bool output();

};


#endif