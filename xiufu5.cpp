// ==================== 编译 / 运行指令 ====================
// 在树莓派上编译（循迹 + 斑马线检测，不需要 tesseract）：
//
//   g++ -std=c++17 xiufu5.cpp -o xiufu5 -lpigpio `pkg-config --cflags --libs opencv4`
//   sudo xauth add $(xauth -f ~pi/.Xauthority list|tail -1)   //图传权限
// 运行（pigpio 需要 root，或者先启动守护进程）：
//   sudo pigpiod && ./xiufu5
//   或：sudo ./xiufu5
//
// 退出：焦点放在任意图像窗口上按 ESC
// 说明：isVisual 用宏定义控制，1=打开可视化窗口
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

#define serval_mid 55
#define isVisual 1 // 是否打开斑马线检测的可视化窗口
bool redisdetected=false;    //红绿灯是否检测到标志位
enum status
{
    BLUD_DETECTED,
    ZEBRA_DETECTED       ,
    RED_LIGHT_DETECTED    ,
    OBSTACLE_DETECTED     ,
    STOP_REGION_DETECTED  ,
    CODE_DETECTED        ,
};



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

int maskcount = 0;
int hasBanma = 0; // 是否检测到斑马线

double picture();
void Set_duo(int angle);
void GetROI(Mat src, Mat &ROI);
void Set_dian();
vector<Point2f> get_lines_fangcheng(vector<Vec4i> lines);
void Set_gpio();
void Control_Xun(double error1);
int crossroad(Mat frame);
void car_run();
void car_stop();
int blue_board(Mat frame);
int red_light(Mat frame);
int green_light(Mat frame);
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
	gpioPWM(13, 10000); 
}




void car_run()
{
	gpioPWM(13, speed_init);
}


int main()
{
	Set_gpio(); // 打开GPIO口
    status status = RED_LIGHT_DETECTED;  //默认状态先检测蓝色挡板
   
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

	cout << "实际分辨率: " << frame.cols << " x " << frame.rows << endl;

	Set_dian(); // 摄像头准备好后再启动电机

	int ci = 0;
	bool zebra_reported = false;
  car_run();
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

         //car_run();
		// 斑马线检测
        switch (status)
        {
            case BLUD_DETECTED:
            cout << "请放置蓝色挡板 请放置蓝色挡板 请放置蓝色挡板 请放置蓝色挡板 请放置蓝色挡板" << endl;
            if (blue_board(frame))
            {
                cout << "检测到蓝色板子 检测到蓝色板子 检测到蓝色板子 检测到蓝色板子 检测到蓝色板子" << endl;
                car_run();
                status = ZEBRA_DETECTED;
            }
            break;

            case ZEBRA_DETECTED:
                if (crossroad(frame))
                {
                        cout << "检测到斑马线，开始停车 检测到斑马线，开始停车 检测到斑马线，开始停车 检测到斑马线，开始停车" << endl;
                        car_stop();
                        gpioDelay(1000*1000*1.5);
                        	gpioPWM(13, 10000);
                          gpioDelay(1000*1000*1.5);
                        car_run();
                        cout << "继续运行 继续运行 继续运行 继续运行 继续运行" << endl;
                        status = RED_LIGHT_DETECTED;
                }
            break;

            case RED_LIGHT_DETECTED:
            if(!redisdetected)
            {
               if(red_light(frame))
              {
                  car_stop();
                  cout << "检测到红色灯，开始停车 检测到红色灯，开始停车 检测到红色灯，开始停车 检测到红色灯，开始停车" << endl;
                  redisdetected=true;
                 // status = BLUD_DETECTED;
              }
            }
            if(redisdetected==true)    //红灯已经检测到了，等待绿灯启动
            {
                if(green_light(frame))
                {
                    car_run();
                    cout << "检测到绿灯，继续运行 检测到绿灯，继续运行 检测到绿灯，继续运行 检测到绿灯，继续运行" << endl;
                    redisdetected=false;
                    status = OBSTACLE_DETECTED;
                }
            }
            break;
        }

        
		
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
//cout << "设置舵机" << endl;
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

//	cout << "angle_xun:" << angle_xun << endl;
	Set_duo(angle_xun);
}

void GetROI(Mat src, Mat &ROI)
{
	int width = src.cols;
	int height = src.rows;
	Rect rect(Point(0, (height / 20) * 10), Point(width, (height / 20) * 17));
	ROI = src(rect);
}

void car_stop()
{
	gpioPWM(13, 10000);
  //  gpioDelay(1000*1000*3);
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
//	cout << "中线均值:" << ave_x << endl;

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

/**
 * frame 输入图像
 * 检测思路：在远端梯形区域内逐行统计"细白色条带"的数量，
 * 同一行出现 >= BanMaNums 条、连续 >= NeedRows 行满足，则判定为斑马线
 *
 * ★ 修复点：
 *   1. ROI 顶点顺序改为正确的顺时针顺序
 *   2. 连续行逻辑用 consecutiveRows，行内用独立 blockCount
 *   3. 函数开头重置 hasBanma
 *   4. 参数提为 const，便于调参
 */
int crossroad(Mat frame)
{
	if (frame.empty())
		return 0;

	int rows = frame.rows;
	int cols = frame.cols;

	Mat src, mask1;

	// ---- 1. 白色范围（HSV）----
	// ★ 树莓派摄像头白平衡不同，若识别不到，请调整这三个值
	Scalar lower_white(0, 0, 160);
	Scalar upper_white(180, 50, 255);
//	Scalar lower_white(0,   0, 200);
//	Scalar upper_white(180, 40, 255);
	// ---- 2. 转 HSV 并颜色分割 ----
	Mat hsv;
	cvtColor(frame, hsv, COLOR_BGR2HSV);
	inRange(hsv, lower_white, upper_white, mask1);

	// ---- 3. 形态学闭运算去噪 ----
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	dilate(mask1, mask1, kernel);
	erode(mask1, mask1, kernel);

	// ---- 4. 多边形 ROI（★ 修复：正确的顺时针顶点顺序）----
	// 顺序：左下 → 左中 → 左上 → 右上 → 右中 → 右下
	Point bottom_left(0, rows);
	Point mid_left(0, rows * 0.7);
	Point top_left(cols * 0.4, rows * 0.55);
	Point top_right(cols * 0.6, rows * 0.55);
	Point mid_right(cols, rows * 0.7);
	Point bottom_right(cols, rows);

	vector<Point> vertices = {bottom_left, mid_left, top_left,
							  top_right, mid_right, bottom_right};
	vector<vector<Point>> pts = {vertices};

	Mat polyMask = Mat::zeros(frame.size(), CV_8UC1);
	fillPoly(polyMask, pts, Scalar(255));

	// 只保留 ROI 内的白色区域
	mask1.copyTo(src, polyMask);

	// ---- 5. 逐行扫描统计（★ 修复：连续行逻辑）----
	const int BanMaMinWidth = 10;   // 单个斑马块最小宽度（像素）
	const int BanMaMaxWidth = 40;  // 单个斑马块最大宽度（像素）
	const int BanMaNums     = 3;   // 一行最少斑马块数
	const int NeedRows      = 6;   // 需要连续合格的行数
	const int RowStep       = 2;   // 每隔几行扫一次
	const int ColMargin     = 10;  // 左右边界忽略的像素数

	int consecutiveRows = 0; // ★ 连续合格行计数器
	hasBanma = 0;            // ★ 函数开头重置

// ============================================================
// 逐行扫描 + 连续行确认
// 目标：判断图像里是否存在"斑马线"
// ============================================================

// ★ 外层循环：逐行扫描
//   i = 当前扫描的行号（y 坐标）
//   从 rows * 0.55 开始 → 只扫描图像下半部分（55% 高度以下）
//   到 rows 结束 → 扫到图像最底部
//   i += RowStep → 每隔 RowStep（=2）行扫一次，加快速度
for (int i = static_cast<int>(rows * 0.55); i < rows; i += RowStep)
{
    // ★ blockCount：记录"当前这一行"里找到的合格斑马块数量
    //   每行开始时清零，保证每行独立计数，不受上一行影响
    //   初始为 0，每找到一个合格白色段就 +1
    int blockCount = 0;

    // ★ 内层循环：逐列扫描当前行
    //   j = 当前扫描的列号（x 坐标）
    //   从 ColMargin（=10）开始 → 忽略最左边 10 个像素（防止边界噪声）
    //   到 cols - ColMargin 结束 → 忽略最右边 10 个像素
    //   注意：这里没有 j++，因为 j 的递增在循环体内部处理
    for (int j = ColMargin; j < cols - ColMargin; )
    {
        // ★ 判断当前像素是不是黑色（值为 0）
        //   src 是二值图：黑色 = 0，白色 = 255
        //   遇到黑色，说明后面可能跟着一段白色（斑马块）
        if (src.at<uchar>(i, j) == 0)
        {
            // 跳过这个黑色像素，从下一个像素开始找白色段
            j++;

            // ★ whiteLen：记录"当前这段连续白色"的长度（像素个数）
            int whiteLen = 0;

            // ★ while 循环：统计后面连续白色像素的个数
            //   条件1：j < cols - ColMargin → 不越界（不超出右边界）
            //   条件2：src.at<uchar>(i, j) == 255 → 当前像素是白色
            //   两个条件都满足 → 继续往后数
            while (j < cols - ColMargin && src.at<uchar>(i, j) == 255)
            {
                j++;        // 移到下一个像素
                whiteLen++; // 白色段长度 +1
            }

            // ★ 判断这段白色是不是"一个合格的斑马块"
            //   合格条件：whiteLen 在 [BanMaMinWidth, BanMaMaxWidth) 之间
            //   即：6 <= whiteLen < 40
            //
            //   为什么限制宽度？
            //     - 太窄（< 6）：可能是噪声、细线，不是斑马线
            //     - 太宽（>= 40）：可能是墙、天空、大片反光，不是斑马线
            //     - 只有中等宽度才像斑马线的条纹
            if (whiteLen >= BanMaMinWidth && whiteLen < BanMaMaxWidth)
            {
                blockCount++; // 找到一个合格斑马块，当前行计数 +1
            }
        }
        else // ★ 当前像素是白色（255），不是黑色
        {
            // 直接跳过，移到下一个像素
            // （因为白色段的统计是在"遇到黑色后"才开始的）
            j++;
        }
    }
    // 内层循环结束：当前行扫描完毕，blockCount = 这一行的合格斑马块数量

    // ★ 单行判断 + 连续行判断
    //   判断当前行的 blockCount 是否达到"一行最少斑马块数"（=3）
    if (blockCount >= BanMaNums) // 当前行合格（≥3 个斑马块）
    {
        // ★ 连续合格行 +1
        consecutiveRows++;

        // ★ 判断是否达到"连续合格行数"要求（=3）
        if (consecutiveRows >= NeedRows)
        {
            hasBanma = 1; // ★ 连续 3 行都合格 → 判定检测到斑马线
            break;        // 已确认，提前退出外层循环（不用再扫了）
        }
    }
    else // ★ 当前行不合格（斑马块 < 3 个）
    {
        // ★ 立即清零连续计数器
        //   为什么？因为要保证"连续"是真的连续
        //   如果中间有一行不合格，说明斑马线断了，必须重新计数
        consecutiveRows = 0;
    }
}
// 外层循环结束：要么检测到斑马线（hasBanma=1），要么扫完所有行都没检测到（hasBanma=0）

	// ---- 6. 可视化 ----
	if (isVisual)
	{
		imshow("roi of banma:", src);
		imshow("mask", mask1);

		// ★ 调试信息
		// cout << "src non-zero: " << countNonZero(src)
		//      << "  consecutiveRows: " << consecutiveRows << endl;
	}

	return hasBanma;
}



/**
 * 蓝色挡板检测（全图检测）
 * frame 输入图像
 * 返回：1 = 检测到蓝色挡板，0 = 未检测到
 */
int blue_board(Mat frame)
{
    if (frame.empty())
        return 0;

    int rows = frame.rows;
    int cols = frame.cols;

    Mat hsv, mask_blue;

    // ---- 1. 转 HSV ----
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // ---- 2. 蓝色范围（HSV）----
    // ★ 树莓派摄像头白平衡不同，若识别不到，请调整这几个值
    //   OpenCV 中 H: 0~180, S: 0~255, V: 0~255
    //   典型蓝色 H 在 100~130
    Scalar lower_blue(100, 80, 60);
    Scalar upper_blue(130, 255, 255);

    inRange(hsv, lower_blue, upper_blue, mask_blue);

    // ---- 3. 形态学去噪（开运算去小噪点，闭运算填补空洞）----
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask_blue, mask_blue, MORPH_OPEN, kernel);
    morphologyEx(mask_blue, mask_blue, MORPH_CLOSE, kernel);
    dilate(mask_blue, mask_blue, kernel);

    // ---- 4. 统计蓝色像素数量（全图）----
    int blueArea = countNonZero(mask_blue);
    int totalArea = rows * cols;

    double ratio = (double)blueArea / totalArea;

    // ---- 5. 轮廓面积判定（更稳）----
    vector<vector<Point>> contours;
    findContours(mask_blue, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    double maxContourArea = 0;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area > maxContourArea)
            maxContourArea = area;
    }

    // ---- 6. 阈值判定 ----
    const double MinRatio       = 0.03;   // 蓝色占比至少 3%
    const double MinContourArea = 1500.0; // 最大蓝色连通块面积至少 1500 像素

    int detected = 0;
    if (ratio > MinRatio && maxContourArea > MinContourArea)
    {
        detected = 1;
    }

    // ---- 7. 可视化调试 ----
    if (isVisual)
    {
        imshow("blue mask", mask_blue);
        cout << "blue ratio: " << ratio
             << "  maxContourArea: " << maxContourArea << endl;
    }

    return detected;
}



/**
 * 红色灯检测（移植自 Python 的 detect_green_circle）
 * frame 输入图像
 * 返回：1 = 检测到红灯，0 = 未检测到
 *
 * 说明：
 *   Python 里给的 HSV 范围 lower=[0,65,243] upper=[179,194,255]
 *   这其实是"高亮度、低饱和度"的白色/亮色区域，不是纯红。
 *   在 OpenCV 中红色 H 分布在 [0,10] 和 [170,180] 两段，
 *   所以这里同时检测两段红色，再合并 mask。
 *   如果你的红灯在实拍中偏亮偏白，可以把 lower/upper 的 V 阈值调低、
 *   S 阈值调高，参考下面注释里的"亮色模式"。
 */
int red_light(Mat frame)
{
    if (frame.empty())
        return 0;

    int rows = frame.rows;
    int cols = frame.cols;

    Mat hsv, mask1, mask2, mask_red;

    // ---- 1. 转 HSV ----
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // ---- 2. 红色范围（HSV 两段）----
    // 红灯典型：H 接近 0 或 180，S 较高，V 较高
    // 第一段：H [0, 10]
    Scalar lower_red1(0,   80, 100);
    Scalar upper_red1(10, 255, 255);
    // 第二段：H [170, 180]
    Scalar lower_red2(170, 80, 100);
    Scalar upper_red2(180, 255, 255);

    inRange(hsv, lower_red1, upper_red1, mask1);
    inRange(hsv, lower_red2, upper_red2, mask2);
    mask_red = mask1 | mask2;

    // ---- 3. 形态学去噪（与 Python 一致：开运算 + 闭运算）----
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask_red, mask_red, MORPH_OPEN,  kernel);
    morphologyEx(mask_red, mask_red, MORPH_CLOSE, kernel);

    // ---- 4. 找轮廓 + 最小外接圆 ----
    vector<vector<Point>> contours;
    findContours(mask_red, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat output = frame.clone();

    int detected = 0;
    const double MinArea = 200.0;   // 与 Python 一致

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area < MinArea)
            continue;

        Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);

        // 画圆（与 Python 一致：红圆 + 蓝心）
        circle(output, center, (int)radius, Scalar(0, 0, 255), 2);
        circle(output, center, 3,          Scalar(255, 0, 0), -1);

        cout << "  红灯 圆心=(" << center.x << "," << center.y
             << "), 半径=" << radius << endl;

        detected = 1;
    }

    // ---- 5. 可视化调试 ----
    if (isVisual)
    {
        imshow("red mask", mask_red);
        imshow("red result", output);
    }

    return detected;
}


/**
 * 绿色灯检测（移植自 Python 的 detect_green_circle）
 * frame 输入图像
 * 返回：1 = 检测到绿灯，0 = 未检测到
 *
 * Python 原始 HSV 范围：
 *   lower_green = [72, 64, 224]
 *   upper_green = [81, 221, 255]
 * 来自 Halcon H[103,115] S[64,221] V[224,255] 换算
 */
int green_light(Mat frame)
{
    if (frame.empty())
        return 0;

    Mat hsv, mask_green;

    // ---- 1. 转 HSV ----
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // ---- 2. 绿色范围（HSV）----
//    Scalar lower_green(72, 64, 224);
//    Scalar upper_green(81, 221, 255);
    Scalar lower_green(69, 185, 112);
    Scalar upper_green(77, 255, 225);

    inRange(hsv, lower_green, upper_green, mask_green);

    // ---- 3. 形态学去噪（开运算 + 闭运算，与 Python 一致）----
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask_green, mask_green, MORPH_OPEN,  kernel);
    morphologyEx(mask_green, mask_green, MORPH_CLOSE, kernel);

    // ---- 4. 找轮廓 + 最小外接圆 ----
    vector<vector<Point>> contours;
    findContours(mask_green, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat output = frame.clone();

    int detected = 0;
    const double MinArea = 200.0;   // 与 Python 一致

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area < MinArea)
            continue;

        Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);

        // 画圆（与 Python 一致：红圆 + 蓝心）
        circle(output, center, (int)radius, Scalar(0, 0, 255), 2);
        circle(output, center, 3,          Scalar(255, 0, 0), -1);

        cout << "  绿灯 圆心=(" << center.x << "," << center.y
             << "), 半径=" << radius << endl;

        detected = 1;
    }

    // ---- 5. 可视化调试 ----
    if (isVisual)
    {
        imshow("green mask", mask_green);
        imshow("green result", output);
    }

    return detected;
}