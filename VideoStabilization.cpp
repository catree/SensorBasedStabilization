#include"VideoStabilization.h"

VideoStabilization::VideoStabilization(string videoName) : name(videoName) {
    //double fuvX = fLength * captureWidth * 1000 / (cellSize * maxWidth);
    double fuvX = 766;
    double fuvY = 766;
    //double fuvY = fLength * captureHeight * 1000 / (cellSize * maxHeight);
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
    for (int i = 0; i < frames; ++i) {
        long sensorTime = 0;
        double avx, avy, avz;
        dataFile >> sensorTime;
        while (sensorTime < timestamps[i] + startTime && !dataFile.eof()) {
            dataFile >> avx >> avy >> avz;
            dataFile >> sensorTime;
        }
        angvX.push_back(avx);
        angvY.push_back(avy);
        angvZ.push_back(avz);
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
                << "|" << angvX[i] / frameRate << " " << angvY[i] / frameRate << " " << angvZ[i] / frameRate
                << "|" << rotAngles[i].pitch << " " << rotAngles[i].heading << " " << rotAngles[i].bank
                << endl;
    }
}

void VideoStabilization::smooth() {
    vector<Quaternion> vDelta(frames), pDelta(frames);
    rotAngles = vector<EulerAngles>(frames);
    p = vector<Quaternion>(frames);
    v = vector<Quaternion>(frames);
    Quaternion qi;
    qi.identity();

    p[0] = qi;
    v[0] = p[0];
    pDelta[0] = angleToQuaternion(angvX[0], angvY[0], angvZ[0]);
    vDelta[0] = qi;
    for (int i = 1; i < frames; ++i) {
        /*p[i] = p[i - 1] * pDelta[i - 1];
        p[i].normalize();
        v[i] = v[i - 1] * vDelta[i - 1];
        v[i].normalize();*/
        p[i] = p[i - 1];
        p[i] *= pDelta[i - 1];
        v[i] = v[i - 1];
        v[i] *= vDelta[i - 1];
        //p[i] *= pDelta[i - 1];
        //v[i] *= vDelta[i - 1];
        rotAngles[i] = computeRotation(v[i], p[i]);

        //if (i == 110)
        //  cout << endl;
        pDelta[i] = angleToQuaternion(angvX[i], angvY[i], angvZ[i]);
        double alpha = computeAlpha(rotAngles[i]);
        if (alpha == 1) {
            vDelta[i] = slerp(qi, v[i - 1], d);
        }
        else {
            Quaternion pDelta2 = pDelta[i];
            //pDelta2 *= conjugate(p[i - 1]) * v[i - 1];
            vDelta[i] = slerp(pDelta2, vDelta[i - 1], alpha);
            //vDelta[i] = slerp(pDelta[i] * conjugate(p[i - 1]) * v[i - 1], vDelta[i - 1], alpha);
        }

    }
}

Quaternion VideoStabilization::angleToQuaternion(double angX, double angY, double angZ) {
    Quaternion q;
    EulerAngles e(angY / frameRate, angX / frameRate, angZ / frameRate);
    q.setToRotateInertialToObject(e);
    return q;
}

double VideoStabilization::computeAlpha(const EulerAngles &rotAngle) {
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
    Mat R = rotationMat(rotAngle);
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
    return 1 - pow(omega, beta);
}

EulerAngles VideoStabilization::computeRotation(const Quaternion &v, const Quaternion &p) {
    EulerAngles rotAngle;
    rotAngle.fromInertialToObjectQuaternion(conjugate(p) * v);
    return rotAngle;
}

Mat VideoStabilization::rotationMat(EulerAngles rotAngle) {
    //double ry = rotAngle.heading;
    double rx = -rotAngle.heading;
    //double rx = rotAngle.pitch;
    double ry = -rotAngle.pitch;
    double rz = rotAngle.bank;
    double z[3][3] = {{cos(rz), sin(rz), 0}, {-sin(rz), cos(rz), 0}, {0, 0, 1}};
    double x[3][3] = {{1, 0, 0}, {0, cos(rx), sin(rx)}, {0, -sin(rx), cos(rx)}};
    double y[3][3] = {{cos(ry), 0, -sin(ry)}, {0, 1, 0}, {sin(ry), 0, cos(ry)}};
    Mat Ry(3, 3, CV_64F, y);
    Mat Rx(3, 3, CV_64F, x);
    Mat Rz(3, 3, CV_64F, z);
    return Rz * Ry * Rx;
}

bool VideoStabilization::output() {
    if (rotAngles.size() != frames)
        return false;
    VideoWriter videoWriter(name + "/VID_" + name + ".avi", CV_FOURCC('M', 'J', 'P', 'G'),
            30, Size(captureWidth, captureHeight));
    VideoWriter videoWriter1(name + "/VID_" + name + "_new.avi", CV_FOURCC('M', 'J', 'P', 'G'),
            30, Size(captureWidth, captureHeight));
    for (int i = 0; i < frames; ++i) {
        stringstream ss;
        ss << name << "/IMG_" << name << "_" << i << ".jpg";
        string imagePath;
        ss >> imagePath;
        Mat frame = imread(imagePath);

        videoWriter << frame;
        rotate(frame, outputFrame, rotationMat(rotAngles[i]));
        cropframe = outputFrame(Range(cropPercent * captureHeight, (1 - cropPercent) * captureHeight),
                Range(cropPercent * captureWidth, (1 - cropPercent) * captureWidth));
        videoWriter1 << outputFrame;
        /*rectangle(outputFrame, Rect(innerPercent * captureWidth, innerPercent * captureHeight,
                        (1 - 2 * innerPercent) * captureWidth, (1 - 2 * innerPercent) * captureHeight),
                Scalar(255, 0, 0));
        rectangle(outputFrame, Rect(cropPercent * captureWidth, cropPercent * captureHeight,
                        (1 - 2 * cropPercent) * captureWidth, (1 - 2 * cropPercent) * captureHeight),
                Scalar(0, 0, 255));*/

        cout << i << endl;
        imshow("ha", cropframe);
        waitKey(1);
    }
    cout << "Output complete!";
    return true;
}

void VideoStabilization::rotate(const Mat &src, Mat &dst, const Mat &R) {
    Mat H = K * R.inv() * K.inv();
    warpPerspective(src, dst, H, Size(src.cols, src.rows), INTER_LINEAR + WARP_INVERSE_MAP);
}
