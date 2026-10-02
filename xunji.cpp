// ==================== 编译 / 运行指令 ====================
// 在树莓派上编译（只有循迹功能，不需要 tesseract）：
//
//   g++ xunji.cpp -o xunji -lpigpio `pkg-config --cflags --libs opencv4`
//
// 运行（pigpio 需要 root，或者先启动守护进程）：
//   sudo pigpiod && ./xunji
//   或：sudo ./xunji
//
// 退出：焦点放在任意图像窗口上按 ESC
// ======================================================

#include <iostream>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc/types_c.h>
#include "pigpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>

using namespace std;
using namespace cv;

#define serval_mid 53

int MAX_YU = 140;
int MIN_YU = 60;

float kr = 0.00000001, kl = -0.0000001, lr = 0, ll = 0, br, bl;

Mat frame, ca;

double speed_init = 9600; // 巡航速度对应的PWM值

double last_error = 0;
double kp = 0.25;
double kd = 0.1;
double min_angle = 20;
double max_angle = 20;

double picture();
void Set_duo(int angle);
void GetROI(Mat src, Mat &ROI);
void Set_dian();
vector<Point2f> get_lines_fangcheng(vector<Vec4i> lines);
void Set_gpio();
void Control_Xun(double error1);

// 电调解锁并启动
void Set_dian()
{
	gpioPWM(13, 10000);
	gpioDelay(1000000);
	for (int i = 3; i > 0; --i) // 等待电调稳定
	{
		printf("%d...\n", i);
		fflush(stdout);
		sleep(1);
	}
	double value = speed_init;
	gpioPWM(13, value);
}

int main()
{
	Set_gpio(); // 打开GPIO口

	VideoCapture capture;
	capture.open(0);
	if (!capture.isOpened())
	{
		cout << "Can not open video file!" << endl;
		return -1;
	}
	capture.set(CAP_PROP_FRAME_WIDTH, 320);
	capture.set(CAP_PROP_FRAME_HEIGHT, 240);

	for (int i = 0; i < 30; i++)
	{
		capture.read(frame);
	}

	Set_dian(); // 摄像头准备好后再启动电机

	int ci = 0;
	while (capture.read(frame))
	{
		ci++;
		if (ci < 5)
			continue;
		else
			ci = 0;

		char key = waitKey(1);
		if (key == 27)
			break;

		double error_xun = picture();
		Control_Xun(error_xun); // 循迹
	}

	destroyAllWindows();
	return 0;
}

void Set_gpio()
{
	if (gpioInitialise() < 0)
		exit(1);
	gpioSetMode(13, PI_OUTPUT);
	gpioSetPWMrange(13, 40000);
	gpioSetPWMfrequency(13, 200);

	gpioSetMode(12, PI_OUTPUT);
	gpioSetPWMrange(12, 30000);
	gpioSetPWMfrequency(12, 50);
}

void Set_duo(int angle)
{
	double value = (0.5 + (2 / 180.0) * angle) / 20 * 30000;
	cout << "设置舵机" << endl;
	gpioPWM(12, value);
}

void Control_Xun(double error1)
{
	double angle_xun = kp * error1 + kd * (error1 - last_error);
	last_error = error1;

	angle_xun = serval_mid - angle_xun;

	if (angle_xun > serval_mid + max_angle)
		angle_xun = serval_mid + max_angle;
	if (angle_xun < serval_mid - min_angle)
		angle_xun = serval_mid - min_angle;

	cout << "angle_xun:" << angle_xun << endl;
	Set_duo(angle_xun);
}

void GetROI(Mat src, Mat &ROI)
{
	int width = src.cols;
	int height = src.rows;
	Rect rect(Point(0, (height / 20) * 10), Point(width, (height / 20) * 17));
	ROI = src(rect);
}

double picture()
{
	Mat roi, hui, gao, binaryImage;
	GetROI(frame, roi);							  // 截取ROI区域
	cvtColor(roi, hui, COLOR_BGR2GRAY);			  // 转换为灰度图
	GaussianBlur(hui, gao, Size(5, 5), 0.5, 0.5); // 高斯滤波
	threshold(gao, binaryImage, 150, 255, cv::THRESH_BINARY); // 二值化
	Canny(gao, ca, MIN_YU, MAX_YU, 3);			  // Canny边缘检测
	imshow("canny", ca);
	imshow("binaryImage", binaryImage);

	int area = countNonZero(ca);

	if (area > 2500)
	{
		MIN_YU += 2;
		MAX_YU += 4;
	}
	if (area < 2000)
	{
		MIN_YU -= 2;
		MAX_YU -= 4;
	}

	vector<Vec4i> plines;
	vector<Point2f> a;
	HoughLinesP(ca, plines, 1, 0.05, 50, 30, 5);
	a = get_lines_fangcheng(plines);

	int i = 0;
	kr = 0.00000001, kl = -0.0000001, lr = 0, ll = 0;

	for (i = 0; i < plines.size(); i++)
	{

		float x1 = a[i].x;
		float x2 = a[i].y + frame.rows / 2;

		if ((x1 > 0.25 && x1 < 2) || (x1 < -0.25 && x1 > -2))
		{
			if (x1 > 0)
			{
				lr++;

				if (x1 > kr)
				{
					kr = x1;
					br = x2;
				}
			}
			else if (x1 < 0)
			{
				ll++;

				if (x1 < kl)
				{
					kl = x1;
					bl = x2;
				}
			}
		}
	}
	float ave_x = 0;
	int flagl = 0, flagr = 0;
	flagl = 0, flagr = 0;

	if (lr == 0)
	{
		flagr = 1;
	}
	if (ll == 0)
	{
		flagl = 1;
	}

	double error;

	for (i = 130; i < 230; i++)
	{
		int l, r;
		if (flagl)
			l = 0;
		else
			l = (i - bl) / kl;
		if (flagr)
			r = frame.cols;
		else
			r = (i - br) / kr;

		double mid = (l + r) / 2;

		ave_x += mid;

		Point pa(r, i);
		Point pb(l, i);

		Point p((l + r) / 2, i);
		circle(frame, pa, 4, Scalar(55, 25, 0));
		circle(frame, pb, 4, Scalar(55, 25, 0));
		circle(frame, p, 1, Scalar(255, 255, 255));
	}
	ave_x = ave_x / 100;

	error = ave_x - frame.cols / 2;

	imshow("处理界面", frame);
	cout << "中线均值:" << ave_x << endl;

	return error;
}

vector<Point2f> get_lines_fangcheng(vector<Vec4i> lines)
{
	float k = 0;
	float b = 0;
	vector<Point2f> lines_fangcheng;
	for (unsigned int i = 0; i < lines.size(); i++)
	{

		k = (double)(lines[i][3] - lines[i][1]) / (double)(lines[i][2] - lines[i][0]);
		b = (double)lines[i][1] - k * (double)lines[i][0];
		lines_fangcheng.push_back(Point2f(k, b));
	}
	return lines_fangcheng;
}
