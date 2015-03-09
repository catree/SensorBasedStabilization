struct CameraParams {
    int width, height;
    float sensorRate;
    double fuvX, fuvY;

    CameraParams(int w, int h, float sr, double fx, double fy)
            : width(w), height(h), sensorRate(sr), fuvX(fx), fuvY(fy) {
    }
};

const CameraParams cameraParamsXIAOMI4(480, 864, 50, 744.5, 744.5);