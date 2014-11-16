#include"VideoStabilization.h"

VideoStabilization::VideoStabilization(string videoName) {
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
        while (sensorTime < timestamps[i] + startTime) {
            dataFile >> sensorTime;
            dataFile >> avx >> avy >> avz;
        }
        angvX.push_back(avx);
        angvY.push_back(avy);
        angvZ.push_back(avz);
    }
}

void VideoStabilization::show() {
    for (int i = 0; i < frames; ++i) {
        cout << timestamps[i] << " " << angvX[i] << " " << angvY[i] << " "
                << angvZ[i] << '\n';
    }
}
