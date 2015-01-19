#ifndef TEST
#define TEST


#include<fstream>
#include<vector>
#include<iostream>
#include<string>
#include<cmath>
#include <opencv2/opencv.hpp>
#include"Quaternion.h"
#include "EulerAngles.h"
#include "VideoStabilization.h"

using namespace std;
using namespace cv;

void test(string videoName);

double searchAngle(const Mat &src, const Mat &dst, double st, double ed, double step, double &mpsnr);

Quaternion angle2Quaternion(double angX, double angY, double angZ);

double getPSNR(const Mat &I1, const Mat &I2);

void testAngle(string videoName, string index);

#endif