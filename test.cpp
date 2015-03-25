#include "test.h"

double getPSNR(const Mat &I1, const Mat &I2) {
    Mat s1;
    absdiff(I1, I2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2

    Scalar s = sum(s1);         // sum elements per channel

    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels
    vector<vector<Point> > contours;
    Mat tmp;
    cvtColor(I2, tmp, CV_RGB2GRAY);
    findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    double area = contours.size() > 0 ? contourArea(contours[0]) : I1.total();

    if (sse <= 1e-10) // for small values return zero
        return 0;
    else {
        double mse = sse / (I1.channels() * area);
        double psnr = 10.0 * log10((255 * 255) / mse);
        return psnr;
    }
}

void test(string videoName) {
    const int rate = 100;

    ifstream dataFile((videoName + "/TXT_" + videoName + ".txt").c_str());

    int start = 0;
    double tmp, frames;
    dataFile >> frames >> tmp;
    for (int i = 0; i < frames; ++i) {
        dataFile >> tmp;
        if (!start)
            start = tmp;
        dataFile >> tmp;

    }

    //vector<Quaternion> q;
    int i = 0;
    Quaternion p;
    p.identity();


    while (!dataFile.eof()) {
        dataFile >> tmp;
        double x, y, z;
        dataFile >> x >> y >> z;
        EulerAngles eulerAngles(y / rate, x / rate, z / rate);
        Quaternion tq;
        tq.setToRotateInertialToObject(eulerAngles);

        if (i % (100 / rate) == 0)
            p = p * tq;
        i++;
        EulerAngles e;
        e.fromInertialToObjectQuaternion(p);
        cout << tmp - start << " " << e.pitch << " " << e.heading << " " << e.bank << endl;
    }

}


double searchAngle(const Mat &src, const Mat &dst, double st, double ed, double step, char dir, double &mpsnr) {
    int captureWidth = src.cols, captureHeight = src.rows;
    //const double fuvX = 799, fuvY = 799;
    const double fuvX = 744.5, fuvY = 744.5;
    double cx = captureWidth / 2 - 0.5, cy = captureHeight / 2 - 0.5;
    Mat K = Mat::zeros(3, 3, CV_64F);
    K.at<double>(0, 0) = fuvX;
    K.at<double>(1, 1) = fuvY;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    K.at<double>(2, 2) = 1;

    double max = -100, angle = 0;
    for (double x = st; x >= ed; x -= step) {
        Mat R = VideoStabilization::rotationMat(angle2Quaternion(x * (dir == 0), x * (dir == 1), 0));
        Mat H = K * R.inv() * K.inv(), src2, dst2, diff;
        warpPerspective(dst, dst2, H, Size(src.cols, src.rows), INTER_LINEAR + WARP_INVERSE_MAP);

        src.copyTo(src2, dst2);
        double psnr = getPSNR(src2, dst2);
        if (psnr > max) {
            max = psnr;
            angle = x;
        }
    }
    mpsnr = max;
    return angle;
}

Quaternion angle2Quaternion(double angX, double angY, double angZ) {
    Quaternion q;
    double magnitude = sqrt(angX * angX + angY * angY + angZ * angZ);
    angX /= -magnitude;
    angY /= magnitude;
    angZ /= magnitude;
    double thetaOverTwo = magnitude / 2.0f;
    double sinThetaOverTwo = sin(thetaOverTwo);
    double cosThetaOverTwo = cos(thetaOverTwo);
    q.x = sinThetaOverTwo * angX;
    q.y = sinThetaOverTwo * angY;
    q.z = sinThetaOverTwo * angZ;
    q.w = cosThetaOverTwo;
    return q;
}

void testAngle(string videoName, string index) {
    Mat src = imread(videoName + "/IMG_" + videoName + "_0.jpg");
    Mat dst = imread(videoName + "/IMG_" + videoName + "_" + index + ".jpg");
    transpose(src, src);
    flip(src, src, 1);
    transpose(dst, dst);
    flip(dst, dst, 1);
    double psnr;
    double sAngle = 0.16 + 0.02;
    char dir = 1;
    double eAngle = searchAngle(src, dst, sAngle, sAngle - 0.04, 0.005, dir, psnr);
    cout << psnr << " " << searchAngle(src, dst, eAngle + 0.005, eAngle - 0.005, 0.001, dir, psnr);
}
