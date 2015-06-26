#ifndef PARAMETERS
#define PARAMETERS

#include<string>

using namespace std;

struct CameraParams {
    int width, height; //image size
    float sensorRate; //sensor sample rate
    double fuvX, fuvY; //focal length
    string fileType; //jpg or mp4
    int frameLength; //only needed for jello effect correction

    CameraParams(int w, int h, float sr, double fx, double fy, int fl, string type = "jpg")
            : width(w), height(h), sensorRate(sr), fuvX(fx), fuvY(fy), frameLength(fl), fileType(type) {
    }
};

const CameraParams cameraParamsXIAOMI4(480, 864, 50, 744.5, 744.5, 22);
const CameraParams cameraParamsHUAWEIP7(544, 960, 100, 799, 799, 22);
const CameraParams cameraParmasVSTAB(1920, 1080, 200, 1746.644, 1746.644, 22, "mp4");
#endif