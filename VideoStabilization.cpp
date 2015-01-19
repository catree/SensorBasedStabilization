#include"VideoStabilization.h"
#include "Matrix4x3.h"
#include "test.h"

VideoStabilization::VideoStabilization(string videoName) : name(videoName) {
    double cx = captureWidth / 2, cy = captureHeight / 2;
    K = Mat::zeros(3, 3, CV_64F);
    K.at<double>(0, 0) = fuvX;
    K.at<double>(1, 1) = fuvY;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    K.at<double>(2, 2) = 1;

    ifstream dataFile((videoName + "/TXT_" + videoName + ".txt").c_str());
    double tmp;
    dataFile >> frames;
    dataFile >> tmp;

    long startTime;
    dataFile >> startTime >> tmp;
    timestamps = vector<int>(frames);
    timestamps[0] = 0;
    for (int i = 1; i < frames; ++i) {
        long time;
        dataFile >> time;
        timestamps[i] = time - startTime;
        dataFile >> tmp;
    }
    Quaternion q;
    p = vector<Quaternion>(frames);
    q.identity();
    long sensorTime;
    dataFile >> sensorTime;
    for (int i = 0; i < frames; ++i) {
        double avx, avy, avz;
        while ((sensorTime < timestamps[i] + startTime) && !dataFile.eof()) {
            dataFile >> avx >> avy >> avz;
            dataFile >> sensorTime;
            q *= angleToQuaternion(avx, avy, avz);
        }
        p[i] = q;
    }
    dataFile.close();
}

void VideoStabilization::show() {
    EulerAngles e, ev;
    for (int i = 0; i < frames; ++i) {
        e.fromInertialToObjectQuaternion(p[i]);
        ev.fromInertialToObjectQuaternion(v[i]);
        cout << i << " " << timestamps[i] << " "
                << e.pitch << " " << e.heading << " " << e.bank
                << "|" << ev.pitch << " " << ev.heading << " " << ev.bank
                << "|" << rotAngles[i].pitch << " " << rotAngles[i].heading << " " << rotAngles[i].bank
                << "|" << alpha[i]
                << endl;
    }
}

void VideoStabilization::smooth() {
    vector<Quaternion> vDelta(frames);
    rotAngles = vector<EulerAngles>(frames);
    rotQuaternions = vector<Quaternion>(frames);

    v = vector<Quaternion>(frames);
    alpha = vector<double>(frames);
    Quaternion qi;
    qi.identity();

    v[0] = p[0];
    vDelta[0] = qi;
    for (int i = 1; i < frames; ++i) {
        v[i] = v[i - 1];
        //v[i] *= vDelta[i - 1];
        v[i] = p[i - 1];
        computeRotation(v[i], p[i], i);

        alpha[i] = computeAlpha(rotQuaternions[i]);
        if (alpha[i] == 1) {
            vDelta[i] = slerp(qi, vDelta[i - 1], d);
        }
        else {
            //Quaternion pDelta2 = conjugate(v[i - 1]) * p[i] * conjugate(p[i - 1]) * v[i - 1];
            //Quaternion pDelta2 = pDelta[i];
            //Quaternion pDelta2 = conjugate(v[i]) * p[i] * pDelta[i] * conjugate(p[i]) * v[i];
            Quaternion pDelta2 = conjugate(v[i]) * p[min(i + 1, frames - 1)];
            Quaternion vDelta2 = slerp(qi, vDelta[i - 1], d);
            //vDelta[i] = slerp(pDelta2, vDelta[i - 1], alpha[i]);
            //vDelta[i] = slerp(pDelta2, vDelta[i - 1], 0.995);
            vDelta[i] = qi;
            //vDelta[i] = slerp(pDelta2, vDelta2, alpha[i]);
            //vDelta[i] = slerp(pDelta[i] * conjugate(p[i - 1]) * v[i - 1], vDelta[i - 1], alpha);
        }

    }
}

Quaternion VideoStabilization::angleToQuaternion(double angX, double angY, double angZ) {
    Quaternion q;
    double magnitude = sqrt(angX * angX + angY * angY + angZ * angZ);
    angX /= -magnitude;
    angY /= magnitude;
    angZ /= magnitude;
    double thetaOverTwo = magnitude / sensorRate / 2.0f;
    double sinThetaOverTwo = sin(thetaOverTwo);
    double cosThetaOverTwo = cos(thetaOverTwo);
    q.x = sinThetaOverTwo * angX;
    q.y = sinThetaOverTwo * angY;
    q.z = sinThetaOverTwo * angZ;
    q.w = cosThetaOverTwo;
    return q;
}

double VideoStabilization::computeAlpha(Quaternion rotQuaternion) {
    double p1[3][1] = {{captureWidth * cropPercent}, {captureHeight * cropPercent}, {1}};
    double p2[3][1] = {{captureWidth * cropPercent}, {captureHeight * (1 - cropPercent)}, {1}};
    double p3[3][1] = {{captureWidth * (1 - cropPercent)}, {captureHeight * cropPercent}, {1}};
    double p4[3][1] = {{captureWidth * (1 - cropPercent)}, {captureHeight * (1 - cropPercent)}, {1}};
    vector<Mat> points(4);
    points[0] = Mat(3, 1, CV_64F, p1);
    points[1] = Mat(3, 1, CV_64F, p2);
    points[2] = Mat(3, 1, CV_64F, p3);
    points[3] = Mat(3, 1, CV_64F, p4);
    double omega = 0;
    double innerWidth = captureWidth * innerPercent, innerHeight = captureHeight * innerPercent;
    Mat R = rotationMat(rotQuaternion);
    for (int i = 0; i < 4; ++i) {
        Mat newPoint = K * R.inv() * K.inv() * points[i];
        double cx = newPoint.at<double>(0, 0) / newPoint.at<double>(2, 0);
        double cy = newPoint.at<double>(1, 0) / newPoint.at<double>(2, 0);
        omega = max(omega, (innerWidth - cx) / innerWidth);
        omega = max(omega, (cx - (captureWidth - innerWidth)) / innerWidth);
        omega = max(omega, (innerHeight - cy) / innerHeight);
        omega = max(omega, (cy - (captureHeight - innerHeight)) / innerHeight);
    }
    if (omega > 1)
        omega = 1;
    return alphaMin + (1 - alphaMin) * (1 - pow(omega, beta));
}

void VideoStabilization::computeRotation(const Quaternion &v, const Quaternion &p, int index) {
    rotQuaternions[index] = conjugate(p) * v;
    rotAngles[index].fromInertialToObjectQuaternion(rotQuaternions[index]);
}

Mat VideoStabilization::rotationMat(Quaternion rotQuaternion) {
    Matrix4x3 matrix4x3;
    matrix4x3.fromQuaternion(rotQuaternion);
    Mat mat = Mat::zeros(3, 3, CV_64F);
    mat.at<double>(0, 0) = matrix4x3.m11;
    mat.at<double>(0, 1) = matrix4x3.m21;
    mat.at<double>(0, 2) = matrix4x3.m31;
    mat.at<double>(1, 0) = matrix4x3.m12;
    mat.at<double>(1, 1) = matrix4x3.m22;
    mat.at<double>(1, 2) = matrix4x3.m32;
    mat.at<double>(2, 0) = matrix4x3.m13;
    mat.at<double>(2, 1) = matrix4x3.m23;
    mat.at<double>(2, 2) = matrix4x3.m33;
    /*mat.at<double>(0, 0) = matrix4x3.m11;
    mat.at<double>(0, 1) = matrix4x3.m12;
    mat.at<double>(0, 2) = matrix4x3.m13;
    mat.at<double>(1, 0) = matrix4x3.m21;
    mat.at<double>(1, 1) = matrix4x3.m22;
    mat.at<double>(1, 2) = matrix4x3.m23;
    mat.at<double>(2, 0) = matrix4x3.m31;
    mat.at<double>(2, 1) = matrix4x3.m32;
    mat.at<double>(2, 2) = matrix4x3.m33;*/
    return mat;
}

bool VideoStabilization::output() {
    if (rotQuaternions.size() != frames)
        return false;
    VideoWriter videoWriter(name + "/VID_" + name + ".avi", CV_FOURCC('M', 'J', 'P', 'G'),
            frameRate, Size(captureWidth, captureHeight));
    VideoWriter videoWriter1(name + "/VID_" + name + "_new.avi", CV_FOURCC('M', 'J', 'P', 'G'),
            frameRate, Size(captureWidth, captureHeight));

    Mat oFrame = imread(name + "/IMG_" + name + "_0.jpg");
    transpose(oFrame, oFrame);
    flip(oFrame, oFrame, 1);
    for (int i = 0; i < frames; ++i) {
        stringstream ss;
        ss << name << "/IMG_" << name << "_" << i << ".jpg";
        string imagePath;
        ss >> imagePath;
        Mat frame = imread(imagePath);
        transpose(frame, frame);
        flip(frame, frame, 1);
        videoWriter << frame;
        rotate(frame, outputFrame, rotationMat(rotQuaternions[i]));

        double psnr;
        double sAngle = (floor(rotAngles[i].pitch * 100) + 2) / 100.0;
        double eAngle = searchAngle(oFrame, frame, sAngle, sAngle - 0.04, 0.005, psnr);
        cout << i << " " << searchAngle(oFrame, frame, eAngle + 0.005, eAngle - 0.005, 0.001, psnr);
        Mat frame2;
        oFrame.copyTo(frame2, outputFrame);
        cout << " " << getPSNR(frame2, outputFrame) << " " << psnr << endl;
        oFrame = frame;

        cropFrame = outputFrame(Range(cropPercent * captureHeight, (1 - cropPercent) * captureHeight),
                Range(cropPercent * captureWidth, (1 - cropPercent) * captureWidth));
        //imshow("ha", outputFrame);
        videoWriter1 << outputFrame;
        //waitKey(1);
    }
    cout << "Output complete!";
    return true;
}

void VideoStabilization::rotate(const Mat &src, Mat &dst, const Mat &R) {
    Mat H = K * R.inv() * K.inv();
    warpPerspective(src, dst, H, Size(src.cols, src.rows), INTER_LINEAR + WARP_INVERSE_MAP);
}
