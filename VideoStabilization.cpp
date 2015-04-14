#include"VideoStabilization.h"
#include "Matrix4x3.h"
#include "test.h"

const double VideoStabilization::d = 0.95;
const double VideoStabilization::alphaMin = 0;
const double VideoStabilization::cropPercent = 0.1, VideoStabilization::innerPercent = 0.05;
const double VideoStabilization::beta = 4;

VideoStabilization::VideoStabilization(string videoName, CameraParams cameraParams) :
        name(videoName), inputType(cameraParams.fileType), frameLength(cameraParams.frameLength),
        captureWidth(cameraParams.width), captureHeight(cameraParams.height),
        sensorRate(cameraParams.sensorRate), fuvX(cameraParams.fuvX), fuvY(cameraParams.fuvY) {
    initK();

    ifstream dataFile((videoName + "/TXT_" + videoName + ".txt").c_str());
    if (!dataFile.is_open())
        assert("The file does not exist!");

    double tmp;
    dataFile >> frames;
    dataFile >> tmp;
    frameRate = round(tmp);
    const int totalSlices = frames * slices;

    long startTime;
    dataFile >> startTime >> tmp;

    timestamps = vector<int>(totalSlices);
    timestamps[0] = 0;
    for (int i = 0; i < frames; ++i) {
        if (i > 0) {
            long time;
            dataFile >> time;
            dataFile >> tmp;
            timestamps[i * slices] = time - startTime;
        }
        for (int j = 1; j < slices; ++j) {
            timestamps[i * slices + j] = timestamps[i * slices] + j * frameLength / slices;
        }
    }

    p = vector<Quaternion>(totalSlices);
    pDelta = vector<Quaternion>(totalSlices);
    v = vector<Quaternion>(totalSlices);
    vDelta = vector<Quaternion>(totalSlices);
    rotQuaternions = vector<Quaternion>(totalSlices);

    alpha = vector<double>(frames);
    rotAngles = vector<EulerAngles>(frames);

    long sensorTime = 0;
    Quaternion q, qi;
    q.identity();
    qi.identity();
    for (int i = 0; i < timestamps.size(); ++i) {
        double avx, avy, avz;
        while ((sensorTime < timestamps[i] + startTime) && !dataFile.eof()) {
            dataFile >> sensorTime;
            dataFile >> avx >> avy >> avz;
            q *= angleToQuaternion(avx, avy, avz);
        }
        float slerpFactor = (sensorTime - timestamps[i] - startTime) * sensorRate / 1000.0f;
        Quaternion partQ = slerp(qi, angleToQuaternion(avx, avy, avz), slerpFactor);
        q *= conjugate(partQ);
        if (i > 0)
            pDelta[i - 1] = q;
        q = partQ;
    }

    dataFile.close();
}

void VideoStabilization::show() {
    EulerAngles e, ev;
    for (int i = 0; i < frames; ++i) {
        e = quaternionToAngle(p[i]);
        ev = quaternionToAngle(v[i]);
        cout << i << " " << e.pitch << " " << e.heading << " " << e.bank
        << "|" << ev.pitch << " " << ev.heading << " " << ev.bank
        << "|" << rotAngles[i].pitch << " " << rotAngles[i].heading << " " << rotAngles[i].bank
        << "|" << alpha[i]
        << endl;
    }
}

void VideoStabilization::smooth() {
    Quaternion qi;
    qi.identity();

    v[0] = p[0] = qi;
    vDelta[0] = qi;
    for (int i = 1; i < frames; ++i) {
        p[i] = p[i - 1] * pDelta[i - 1];
        v[i] = v[i - 1];
        v[i] *= vDelta[i - 1];
        computeRotation(v[i], p[i], i);

        alpha[i] = computeAlpha(rotQuaternions[i]);
        if (alpha[i] == 1) {
            vDelta[i] = slerp(qi, vDelta[i - 1], d);
        }
        else {
            Quaternion pDelta2 = conjugate(v[i - 1]) * p[i] * conjugate(p[i - 1]) * v[i - 1];
            vDelta[i] = slerp(pDelta2, vDelta[i - 1], alpha[i]);
        }

    }
}

Quaternion VideoStabilization::angleToQuaternion(double angX, double angY, double angZ) {
    Quaternion q;
    double magnitude = sqrt(angX * angX + angY * angY + angZ * angZ);
    angX /= -magnitude; //rotate 180 degrees along the x axis
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
    double p1[3][1] = {{captureWidth * cropPercent},
                       {captureHeight * cropPercent},
                       {1}};
    double p2[3][1] = {{captureWidth * cropPercent},
                       {captureHeight * (1 - cropPercent)},
                       {1}};
    double p3[3][1] = {{captureWidth * (1 - cropPercent)},
                       {captureHeight * cropPercent},
                       {1}};
    double p4[3][1] = {{captureWidth * (1 - cropPercent)},
                       {captureHeight * (1 - cropPercent)},
                       {1}};
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
    rotAngles[index] = quaternionToAngle(rotQuaternions[index]);
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
    /*VideoWriter videoWriter1(name + "/VID_" + name + "_new.avi", CV_FOURCC('M', 'J', 'P', 'G'),
                             frameRate, Size(captureWidth, captureHeight));*/
    VideoWriter videoWriter2(name + "/VID_" + name + "_newc.avi", CV_FOURCC('M', 'J', 'P', 'G'),
                             frameRate,
                             Size(captureWidth * (1 - 2 * cropPercent), captureHeight * (1 - 2 * cropPercent)));

    for (int i = 0; i < frames; ++i) {
        cout << "Generating frame " << i << "..." << endl;

        Mat frame;
        if (inputType == "mp4")
            getFrameByMp4(frame);
        else
            getFrameByJpg(frame, i);
        videoWriter << frame;
        rotate(frame, outputFrame, rotationMat(rotQuaternions[i]));

        cropFrame = outputFrame(Range(cropPercent * captureHeight, (1 - cropPercent) * captureHeight),
                                Range(cropPercent * captureWidth, (1 - cropPercent) * captureWidth));
        //imshow("ha", cropFrame);
        //videoWriter1 << outputFrame;
        videoWriter2 << cropFrame;
        //waitKey(1);
    }
    cout << "Output complete!";
    return true;
}


void VideoStabilization::getFrameByJpg(Mat &frame, int index) {
    stringstream ss;
    ss << name << "/IMG_" << name << "_" << index << ".jpg";
    string imagePath;
    ss >> imagePath;
    frame = imread(imagePath);
    transpose(frame, frame);
    flip(frame, frame, 1);
}

void VideoStabilization::getFrameByMp4(Mat &frame) {
    static VideoCapture video(name + "/" + name + ".mp4");
    video >> frame;
}

void VideoStabilization::rotate(const Mat &src, Mat &dst, const Mat &R) {
    Mat H = K * R.inv() * K.inv();
    warpPerspective(src, dst, H, Size(src.cols, src.rows), INTER_LINEAR + WARP_INVERSE_MAP);
}

EulerAngles VideoStabilization::quaternionToAngle(Quaternion q) {
    EulerAngles e;
    float sinThetaOverTwo = sqrt(1 - q.w * q.w);
    float theta = acos(q.w) * 2;
    if (sinThetaOverTwo > 0.000001)
        e = EulerAngles(q.y / sinThetaOverTwo * theta, -q.x / sinThetaOverTwo * theta,
                        q.z / sinThetaOverTwo * theta);
    else
        e = EulerAngles(2 * q.y / q.w, -2 * q.x, 2 * q.z / q.w);
    return e;
}

void VideoStabilization::initK() {
    double cx = captureWidth / 2 - 0.5, cy = captureHeight / 2 - 0.5;
    K = Mat::zeros(3, 3, CV_64F);
    K.at<double>(0, 0) = fuvX;
    K.at<double>(1, 1) = fuvY;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    K.at<double>(2, 2) = 1;
}
