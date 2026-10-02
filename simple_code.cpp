#include <iostream>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc/types_c.h>
#include "pigpio.h"
#include <thread>
#include <cmath>
#include <chrono>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <signal.h>
using namespace std;
using namespace cv;

#define Usage()                                            \
	{                                                      \
		std::cerr << "usage: ./showpic FILE" << std::endl; \
	}

#define BAUD 9600


int MAX_YU = 140;
int MIN_YU = 60;


float kr = 0.00000001, kl = -0.0000001, lr = 0, ll = 0, br, bl;
bool flag_ren = false;
bool flag_start;
double angle;	
double angle_bi;

Mat kernel_3 = Mat::ones(cv::Size(3, 3), CV_8U);
Mat kernel = getStructuringElement(MORPH_RECT, Size(1, 1));
Mat frame, ca;

double speed_mid = 10800;
double speed_init = 10800;
clock_t start_, end_, mid_time;

double last_error = 0;
double kp = 0.14;
double kd = 0.1;
double min_angle = 30;
double max_angle = 20;

double picture();
void Set_duo(int angle);
void GetROI(Mat src, Mat &ROI);
void Set_dian();
vector<Point2f> get_lines_fangcheng(vector<Vec4i> lines);
void Set_gpio();
Rect Obstacles(Mat img);
int car_start(Mat img);
Rect blue(Mat img);
bool crossing(Mat image);
void Control_stop();
void Control_Xun(double error1);
void Control_avoid(double len1,double error2);

bool crossing(Mat image)
{
	Mat roi, labels, stats, centroids, hui;
	GetROI(image, roi);
	cvtColor(roi, hui, COLOR_BGR2GRAY);
	Mat binaryImage;
	threshold(hui, binaryImage, 180, 255, cv::THRESH_BINARY);
	imshow("a", binaryImage);
	int numComponents = cv::connectedComponentsWithStats(binaryImage, labels, stats, centroids);
	int crossingnum = 0;
	for (int i = 1; i < numComponents; i++)
	{
		int area = stats.at<int>(i, cv::CC_STAT_AREA);
		if (area > 200)
		{
			crossingnum++;
		}
		// cout<<area<<endl;
	}
	cout << "CrossingNum:" << crossingnum << endl;
	if (crossingnum >= 4)
	{
		cout << "��|l|l|" << endl;
		return true;
	}
	return false;
}

void Set_dian()
{
	gpioPWM(13, 12000);
	gpioDelay(1000000);

	gpioPWM(13, 11500);
	gpioDelay(1000000);
	
	gpioPWM(13, 11000);
	gpioDelay(1000000);

	double value = speed_init;
	gpioPWM(13, value);
}
void Control_stop()
{
	gpioPWM(13, 12300);
	gpioDelay(100000);

	gpioPWM(13, 12000);
	gpioDelay(3000000);

	gpioPWM(13, speed_mid);
}

int main()
{
	Set_gpio();

	VideoCapture capture;
	capture.open(0);
	if (!capture.isOpened())
	{
		cout << "Can not open video file!" << endl;
		return -1;
	}
	capture.set(CAP_PROP_FRAME_WIDTH, 320);
	capture.set(CAP_PROP_FRAME_HEIGHT, 240);

	int ci = 0;
	Rect list;

	for (int i = 0; i < 30; i++)
	{
		capture.read(frame);
	}

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

		imshow("frame", frame);

		int m;
		m = 1;
		 
		if (flag_start == false)
		{
			m = car_start(frame);
			m=1;
			if (m == 0)
			{
				cout << "未识别到蓝色挡板" << endl;
				continue;
			}
		}
		else
		{
			m = 1;
		}
		int n = 0;

		if (m == 1 && n == 0)
		{
			if (flag_start == false)
			{
				Set_dian();
				start_ = clock();
        cout<<"开启电机"<<endl;
				flag_start = true;
			}
      
          list = Obstacles(frame);
          
          if (!list.empty())
          {
              double len = list.x + list.width/2 - frame.cols / 2;                                        
          cout<<"len:  "<<len<<endl;
          double error_bi = picture();
          Control_avoid(len,error_bi);
          }
          //gpioPWM(13, 10800);
			double error_xun = picture();
			Control_Xun(error_xun);
			flag_ren=true;
			if (flag_ren == false)
			{
				bool flag1 = crossing(frame);
				
				if (flag1 == true)
				{
					cout << "识别到斑马线" << endl;
					Control_stop();
					flag_ren = true;
				}
			}
		}
	}

	destroyAllWindows();
	system("pause");
	exit(EXIT_SUCCESS);
	return 0;
}

void Set_gpio()
{

	if (gpioInitialise() < 0)
		exit(1);
	gpioSetMode(13, PI_OUTPUT);
	gpioSetPWMrange(13, 40000);
	gpioSetPWMfrequency(13, 200);

	if (gpioInitialise() < 0)
		exit(1);
	gpioSetMode(12, PI_OUTPUT);
	gpioSetPWMrange(12, 30000); 
	gpioSetPWMfrequency(12, 50);
}

void Set_duo(int angle)
{
	double value = (0.5 + (2 / 180.0) * angle) / 20 * 30000;
	cout<<"设置舵机"<<endl;
	gpioPWM(12, value);
}

void Control_Xun(double error1)
{
	double angle = kp * error1 + kd * (error1 - last_error);
	last_error = error1;

	angle = 90 - angle;

	if (angle > 90 + max_angle)
		angle = 90 + max_angle;
	if (angle < 90 - min_angle)
		angle = 90 - min_angle;

	cout << "angle:" << angle << endl;
	//cout<<"循迹处理完毕"<<endl;
	Set_duo(angle);
}

void Control_avoid(double len1,double error2) 
{
    if (len1 >= 0) {
    error2 += (len1+20);
    } else {
    error2 += (len1-20);
    }
    
  	double angle_bi = kp * error2 + kd * (error2 - last_error);
	last_error = error2;

	angle_bi = 90 - angle_bi;

	if (angle_bi > 90 + max_angle)
		angle_bi = 90 + max_angle;
	if (angle_bi < 90 - min_angle)
		angle_bi = 90 - min_angle;

	cout << "angle:" << angle << endl;
	//cout<<"循迹处理完毕"<<endl;
	Set_duo(angle);
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
	// morphologyEx(gao, gao, MORPH_OPEN, kernel_3);  // 开运算
	// threshold(gao, binaryImage, 150, 255, cv::THRESH_BINARY); // 二值化
	Canny(gao, ca, MIN_YU, MAX_YU, 3); // Canny边缘检测
	imshow("canny", ca);
	// imshow("binaryImage", binaryImage);

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
	kr = 0.00000001, kl = -0.0000001, lr = 0, ll = 0, br, bl;
	int sum = 0;

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
	gpioPWM(13, 10600);
	Control_Xun(error);

	imshow("处理界面", frame);
	cout << "中线均值:" << ave_x;

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

Rect blue(Mat img)
{

	Mat HSV, roi;
	GetROI(img, roi);
	cvtColor(roi, HSV, COLOR_BGR2HSV);
	Scalar Lower(90, 50, 50);
	Scalar Upper(130, 255, 255);

	Mat mask;
	inRange(HSV, Lower, Upper, mask);

	Mat erosion_dilation;
	erode(mask, erosion_dilation, kernel_3, Point(-1, -1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());
	dilate(erosion_dilation, erosion_dilation, kernel_3, Point(-1, -1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());

	Mat target;
	bitwise_and(roi, roi, target, erosion_dilation);

	Mat binary;
	threshold(erosion_dilation, binary, 127, 255, THRESH_BINARY);

	imshow("blue", binary);
	vector<vector<Point>> contours;
	findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	int maxContourIndex = -1;
	double maxContourArea = 0.0;
	for (int i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		if (area > maxContourArea)
		{
			maxContourArea = area;
			maxContourIndex = i;
		}
	}
	if (maxContourIndex != -1)
	{
		return boundingRect(contours[maxContourIndex]);
	}
	else
	{
		return Rect();
	}
}

Rect Obstacles(Mat img)
{
	Rect obstacle_object = blue(img);
	if (obstacle_object.height > 20 && obstacle_object.height < 100)
	{
		rectangle(img, Point(obstacle_object.x, obstacle_object.y + frame.rows / 2), Point(obstacle_object.x + obstacle_object.width, obstacle_object.y + obstacle_object.height + frame.rows / 2), Scalar(0, 255, 0), 3);
		return obstacle_object;
	}
	return Rect();
}

int car_start(Mat img)
{
	Rect start_flag = blue(img);

	// cout << start_flag.width << "   " << start_flag.height << endl;
	if (start_flag.width > 50 && start_flag.height > 50)
	{
		return 1;
	}
	return 0;
}
