#ifndef VIDEO_STABILIZATION
#define VIDEO_STABILIZATION

#include<opencv/cv.h>
#include<opencv/highgui.h>
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
    const double d = 0.95;
    const double cropPercent = 0.1, innerPercent = 0.05;
    const int beta = 3;

    const double cellSize = 1.12;
    const int fLength = 4000; //um
    const int maxWidth = 3840, maxHeight = 2160;
    const int captureWidth = 960, captureHeight = 544;
    const double frameRate = 30;
    Mat K;


    int frames;
    string name;
    vector<int> timestamps;
    vector<double> angvX, angvY, angvZ;
    vector<EulerAngles> rotAngles;
    vector<Quaternion> p;

    Quaternion angleToQuaternion(double angX, double angY, double angZ);

    double computeAlpha(const EulerAngles &rotAngle);

    EulerAngles computeRotation(const Quaternion &v, const Quaternion &p);

    Mat rotationMat(EulerAngles rotAngle);

public:
    VideoStabilization(string videoName);

    void smooth();

    void show();

    bool output();
};


#endif