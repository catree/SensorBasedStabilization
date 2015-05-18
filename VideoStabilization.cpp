#include"VideoStabilization.h"

const double VideoStabilization::d = 0.95;
const double VideoStabilization::cropPercent = 0.1, VideoStabilization::innerPercent = 0.05;
const double VideoStabilization::beta = 4;

VideoStabilization::VideoStabilization(string videoName, CameraParams cameraParams, string dir) :
        name(videoName), directory(dir), inputType(cameraParams.fileType), frameLength(cameraParams.frameLength),
        captureWidth(cameraParams.width), captureHeight(cameraParams.height),
        sensorRate(cameraParams.sensorRate), fuvX(cameraParams.fuvX), fuvY(cameraParams.fuvY) {
    initK();

    ifstream dataFile((directory + videoName + "/TXT_" + videoName + ".txt").c_str());
    if (!dataFile.is_open())
        assert("The file does not exist!");

    double tmp;
    dataFile >> frames;
    dataFile >> tmp;
    frameRate = round(tmp);
    const int totalSlices = frames * slices;

    long long startTime;
    dataFile >> startTime >> tmp;

    timestamps = vector<int>(totalSlices);
    timestamps[0] = 0;
    for (int i = 0; i < frames; ++i) {
        if (i > 0) {
            long long time;
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
    v = vector<Quaternion>(frames);
    vDelta = vector<Quaternion>(frames);
    rotQuaternions = vector<Quaternion>(totalSlices);
    alpha = vector<double>(frames);
    rotAngles = vector<EulerAngles>(totalSlices);

    long long sensorTime = 0;
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
        e = quaternionToAngle(p[i * slices]);
        ev = quaternionToAngle(v[i]);
        cout << i << " " << e.pitch << " " << e.heading << " " << e.bank
        << "|" << ev.pitch << " " << ev.heading << " " << ev.bank
        << "|" << rotAngles[i * slices].pitch << " " << rotAngles[i * slices].heading << " " <<
        rotAngles[i * slices].bank
        << "|" << alpha[i]
        << endl;
    }
}

void VideoStabilization::smooth() {
    Quaternion qi;
    qi.identity();

    v[0] = p[0] = qi;
    vDelta[0] = qi;
    for (int k = 1; k < p.size(); ++k) {
        p[k] = p[k - 1];
        p[k] *= pDelta[k - 1];
        int i = k / slices;
        if (k % slices == 0) {
            v[i] = v[i - 1];
            v[i] *= vDelta[i - 1];
        }
        computeRotation(v[i], p[k], k);
        if (k % slices == 0) {
            alpha[i] = computeAlpha(rotQuaternions[k]);
            if (alpha[i] == 1) {
                vDelta[i] = slerp(qi, vDelta[i - 1], d);
            }
            else {
                Quaternion pDelta2 = conjugate(v[i - 1]) * p[k] * conjugate(p[k - slices]) * v[i - 1];
                vDelta[i] = slerp(pDelta2, vDelta[i - 1], alpha[i]);
            }
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
    return 1 - pow(omega, beta);
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
    VideoWriter videoWriter(directory + name + "/VID_" + name + ".avi", CV_FOURCC('M', 'J', 'P', 'G'),
                            frameRate, Size(captureWidth, captureHeight));
    /*VideoWriter videoWriter1(name + "/VID_" + name + "_new.avi", CV_FOURCC('M', 'J', 'P', 'G'),
                             frameRate, Size(captureWidth, captureHeight));*/
    const int cropWidth = captureWidth * (1 - 2 * cropPercent);
    const int cropHeight = captureHeight * (1 - 2 * cropPercent);
    VideoWriter videoWriter2(directory + name + "/VID_" + name + "_newc.avi", CV_FOURCC('M', 'J', 'P', 'G'),
                             frameRate,
                             Size(cropWidth, cropHeight));

    if (inputType == "mp4") {
        video = VideoCapture(directory + name + "/" + name + ".mp4");
    }

    for (int i = 0; i < frames; ++i) {
        cout << "Generating frame " << i << "..." << endl;

        Mat frame;
        if (inputType == "mp4")
            getFrameByMp4(frame);
        else
            getFrameByJpg(frame, i);
        videoWriter << frame;
        cropFrame = frame(Range(0, cropHeight), Range(0, cropWidth));

        for (int j = 0; j < slices; ++j) {
            rotate(frame, outputFrame, rotationMat(rotQuaternions[i * slices + j]));
            int sliceY1 = j * cropHeight / slices;
            int sliceY2 = (j + 1) * cropHeight / slices;
            int sliceX1 = 0;
            int sliceX2 = cropFrame.cols;
            const int offsetY = captureHeight * cropPercent;
            const int offsetX = captureWidth * cropPercent;
            if (slices == 1)
                cropFrame = outputFrame(Range(sliceY1 + offsetY, sliceY2 + offsetY),
                                        Range(sliceX1 + offsetX, sliceX2 + offsetX));
            else {
                outputFrame(Range(sliceY1 + offsetY, sliceY2 + offsetY),
                            Range(sliceX1 + offsetX, sliceX2 + offsetX)).copyTo(
                        cropFrame(Range(sliceY1, sliceY2), Range(sliceX1, sliceX2)));
            }
        }
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
    frame = imread(directory + imagePath);
    transpose(frame, frame);
    flip(frame, frame, 1);
}

void VideoStabilization::getFrameByMp4(Mat &frame) {
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
