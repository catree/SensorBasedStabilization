#ifndef PARAMETERS
#define PARAMETERS

#include<string>

using namespace std;

struct CameraParams {
    int width, height;
    float sensorRate;
    double fuvX, fuvY;
    string fileType;

    CameraParams(int w, int h, float sr, double fx, double fy, string type = "jpg")
            : width(w), height(h), sensorRate(sr), fuvX(fx), fuvY(fy), fileType(type) {
    }
};

const CameraParams cameraParamsXIAOMI4(480, 864, 50, 744.5, 744.5);
const CameraParams cameraParamsHUAWEIP7(544, 960, 100, 799, 799);
const CameraParams cameraParmasVSTAB(1920, 1080, 200, 1746.6, 1746.6, "mp4");
#endif