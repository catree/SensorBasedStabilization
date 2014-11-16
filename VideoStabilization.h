#ifndef VIDEO_STABILIZATION
#define VIDEO_STABILIZATION

#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<fstream>
#include<vector>
#include<iostream>
#include<string>

using namespace cv;
using namespace std;

class VideoStabilization{
private:
	int frames;
	vector<int> timestamps;
	vector<double> angvX, angvY, angvZ;
public:
	VideoStabilization(string videoName);
	void show();
};


#endif