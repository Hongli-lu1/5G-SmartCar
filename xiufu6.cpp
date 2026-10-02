// ==================== 编译 / 运行指令 ====================
// 在树莓派上编译（循迹 + 斑马线检测 + 蓝色障碍物避障，不需要 tesseract）：
//
//   g++ -std=c++17 xiufu6.cpp -o Zebra -lpigpio `pkg-config --cflags --libs opencv4`
//   sudo xauth add $(xauth -f ~pi/.Xauthority list|tail -1)   // 图传权限
//
// 运行（pigpio 需要 root，或者先启动守护进程）：
//   sudo pigpiod && ./Zebra
//   或：sudo ./Zebra
//
// 退出：焦点放在任意图像窗口上按 ESC
// 说明：
//   isVisual 用宏控制，1=打开可视化窗口
//   isAvoid  用宏控制，1=开启障碍物避障
// ======================================================

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc/types_c.h>
#include <opencv4/opencv2/opencv.hpp>

#include "pigpio.h"

using namespace std;
using namespace cv;

// ==================== 宏定义 / 常量 ====================
#define serval_mid 55
#define isVisual   1 // 是否打开可视化窗口
#define isAvoid    1 // 是否启用障碍物避障（仅 OBSTACLE_DETECTED 状态生效）

// ==================== 状态机 ====================
enum status
{
    BLUD_DETECTED,
    ZEBRA_DETECTED,
    RED_LIGHT_DETECTED,
    OBSTACLE_DETECTED,
    STOP_REGION_DETECTED,
    CODE_DETECTED,
};

// ==================== 全局变量 ====================
int MAX_YU = 140;
int MIN_YU = 60;

float kr = 0.00000001, kl = -0.0000001, lr = 0, ll = 0, br, bl;

Mat frame, ca;

double speed_init = 9600; // 巡航速度对应的 PWM 值

double last_error = 0;
double kp = 0.2;
double kd = 0.1;
double min_angle = 30;
double max_angle = 30;

int maskcount = 0;
int hasBanma = 0; // 是否检测到斑马线

status car_state = BLUD_DETECTED; // 当前状态（全局，供循迹判断是否处于避障阶段）

int obstacle_x = 0;     // 障碍物中心横坐标（全图坐标，供中线计算）
int obstacle_count = 0; // 检测到的蓝色障碍物数量

const int OBSTACLE_LOST_FRAMES = 20; // 连续多少帧看不到障碍物，认为已通过

// ==================== 函数声明 ====================
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
int find_blue_obstacle(Mat &img, int &obs_x);

// ==================== 电机 / 舵机 ====================

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

void car_stop()
{
    gpioPWM(13, 10000);
    // gpioDelay(1000 * 1000 * 3);
}

void Set_duo(int angle)
{
    double value = (0.5 + (2 / 180.0) * angle) / 20 * 30000;
    cout << "设置舵机" << endl;
    gpioPWM(12, value);
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

// ==================== 主流程 ====================
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

    // ★ 打印实际分辨率，便于确认
    cout << "实际分辨率: " << frame.cols << " x " << frame.rows << endl;

    Set_dian(); // 摄像头准备好后再启动电机

    int ci = 0;                  // 帧计数器：每 5 帧才处理一次循迹
    bool zebra_reported = false; // 斑马线是否已上报（预留，目前未启用）

    // ---- 障碍物避障阶段的辅助变量 ----
    bool obstacle_seen = false; // obstacle_seen：避障阶段是否已经看到过障碍物
    int obstacle_lost = 0;      // obstacle_lost：障碍物连续丢失的帧计数

    while (capture.read(frame))
    {
        ci++;
        if (ci < 3)
            continue;
        else
            ci = 0;

        char key = waitKey(1);
        if (key == 27)
            break;

        // ---- 赛程状态机 ----
        switch (car_state)
        {
        case BLUD_DETECTED:
            cout << "请放置蓝色挡板 请放置蓝色挡板 请放置蓝色挡板 请放置蓝色挡板 请放置蓝色挡板" << endl;
            if (blue_board(frame))
            {
                cout << "检测到蓝色板子 检测到蓝色板子 检测到蓝色板子 检测到蓝色板子 检测到蓝色板子" << endl;
                car_run();
                car_state = ZEBRA_DETECTED;
            }
            break;

        case ZEBRA_DETECTED:
            if (crossroad(frame))
            {
                cout << "Zebra is Detected Zebra is Detected Zebra is Detected Zebra is Detected Zebra is Detected" << endl;
                zebra_reported = true;
                car_stop();
                cout << "检测到斑马线，开始停车 检测到斑马线，开始停车 检测到斑马线，开始停车 检测到斑马线，开始停车" << endl;
                gpioDelay(1000 * 1000 * 3);
                car_run();
                cout << "继续运行 继续运行 继续运行 继续运行 继续运行" << endl;
                cout << "进入障碍物避障阶段 进入障碍物避障阶段 进入障碍物避障阶段" << endl;
                car_state = OBSTACLE_DETECTED;
            }
            break;

        case OBSTACLE_DETECTED:
            // 此处读取的是上一轮 picture() 的障碍物检测结果（一帧延迟，无影响）：
            //   看到障碍物 → 重置丢失计数；
            //   障碍物消失后连续 OBSTACLE_LOST_FRAMES 帧 → 判定已通过，进入红灯检测
            if (obstacle_count > 0) // obstacle_count：上一轮 picture() 检测到的蓝色障碍物个数
            {
                obstacle_seen = true; // obstacle_seen：标记"已经看到过障碍物"
                obstacle_lost = 0;    // obstacle_lost：清零连续丢失帧计数
            }
            else if (obstacle_seen) // 当前没检测到，但之前看到过 → 开始累计丢失
            {
                obstacle_lost++; // obstacle_lost：连续未检测到障碍物的帧数 +1
                if (obstacle_lost >= OBSTACLE_LOST_FRAMES) // OBSTACLE_LOST_FRAMES：判定"已通过"的丢失帧阈值
                {
                    cout << "障碍物已通过，进入红灯检测 障碍物已通过，进入红灯检测" << endl;
                    car_state = RED_LIGHT_DETECTED; // car_state：状态机切换到红灯检测
                }
            }
            break;

        case RED_LIGHT_DETECTED:
            cout << "检测到红色灯，开始停车 检测到红色灯，开始停车 检测到红色灯，开始停车 检测到红色灯，开始停车" << endl;
            break;
        }

        double error_xun = picture();
        Control_Xun(error_xun); // 循迹（避障阶段会按障碍物调整中线）
    }

    destroyAllWindows();
    return 0;
}

// ==================== 循迹 ====================
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
    GetROI(frame, roi);                                       // 截取ROI区域
    cvtColor(roi, hui, COLOR_BGR2GRAY);                       // 转换为灰度图
    GaussianBlur(hui, gao, Size(5, 5), 0.5, 0.5);             // 高斯滤波
    threshold(gao, binaryImage, 150, 255, cv::THRESH_BINARY); // 二值化
    Canny(gao, ca, MIN_YU, MAX_YU, 3);                        // Canny边缘检测
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

    // ---- 蓝色障碍物检测（仅在 OBSTACLE_DETECTED 状态生效）----
    obstacle_x = 0;
    obstacle_count = 0;
    bool avoid = isAvoid && (car_state == OBSTACLE_DETECTED);
    if (avoid)
    {
        obstacle_count = find_blue_obstacle(frame, obstacle_x);
        if (obstacle_count > 0)
        {
            cout << "检测到蓝色障碍物 " << obstacle_count
                 << " 个，参考中心 x = " << obstacle_x << endl;
        }
    }

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

        // ---- 中线计算 ----
        // 避障阶段且检测到障碍物时：
        //   障碍物在右 → 中线 =（左线 + 障碍物）/ 2
        //   障碍物在左 → 中线 =（右线 + 障碍物）/ 2
        // 其余情况维持原来的左右线中点。
        double mid;
        if (avoid && obstacle_count > 0)
        {
            if (obstacle_x > frame.cols / 2)
                mid = (l + obstacle_x) / 2.0;
            else
                mid = (r + obstacle_x) / 2.0;
        }
        else
        {
            mid = (l + r) / 2;
        }
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

// ==================== 斑马线检测 ====================
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
    const int BanMaMinWidth = 6;  // 单个斑马块最小宽度（像素）
    const int BanMaMaxWidth = 40; // 单个斑马块最大宽度（像素）
    const int BanMaNums     = 3;  // 一行最少斑马块数
    const int NeedRows      = 3;  // 需要连续合格的行数
    const int RowStep       = 2;  // 每隔几行扫一次
    const int ColMargin     = 10; // 左右边界忽略的像素数

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
        int blockCount = 0;

        // ★ 内层循环：逐列扫描当前行
        //   j = 当前扫描的列号（x 坐标）
        //   从 ColMargin（=10）开始 → 忽略最左边 10 个像素（防止边界噪声）
        //   到 cols - ColMargin 结束 → 忽略最右边 10 个像素
        for (int j = ColMargin; j < cols - ColMargin;)
        {
            // ★ 当前像素是黑色（0）：后面可能跟着一段白色（斑马块）
            if (src.at<uchar>(i, j) == 0)
            {
                j++;

                // ★ whiteLen：当前这段连续白色的长度
                int whiteLen = 0;

                while (j < cols - ColMargin && src.at<uchar>(i, j) == 255)
                {
                    j++;
                    whiteLen++;
                }

                // ★ 合格斑马块：6 <= whiteLen < 40
                if (whiteLen >= BanMaMinWidth && whiteLen < BanMaMaxWidth)
                {
                    blockCount++;
                }
            }
            else // ★ 当前像素是白色（255），跳过
            {
                j++;
            }
        }

        // ★ 单行 + 连续行判断
        if (blockCount >= BanMaNums) // 当前行合格（≥3 个斑马块）
        {
            consecutiveRows++;
            if (consecutiveRows >= NeedRows)
            {
                hasBanma = 1; // ★ 连续 3 行都合格 → 判定检测到斑马线
                break;
            }
        }
        else // ★ 当前行不合格
        {
            consecutiveRows = 0; // 断开连续计数
        }
    }

    // ---- 6. 可视化 ----
    if (isVisual)
    {
        imshow("roi of banma:", src);
        imshow("mask", mask1);

        // cout << "src non-zero: " << countNonZero(src)
        //      << "  consecutiveRows: " << consecutiveRows << endl;
    }

    return hasBanma;
}

// ==================== 蓝色挡板检测（起跑用） ====================
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

// ==================== 蓝色障碍物检测（避障用） ====================
/**
 * 移植自 Python 版 find_blue_regions：
 *   1. HSV 提取蓝色 → 形态学开/闭运算去噪
 *   2. 找外轮廓，按面积筛选（minArea ~ maxArea）
 *   3. 返回合格障碍物数量，并通过 obs_x 输出参考障碍物（面积最大者）的中心横坐标
 *
 * img    输入图像（会在其上绘制检测框，用于可视化）
 * obs_x  输出：参考障碍物的中心 x 坐标（全图坐标）
 * 返回：检测到的合格蓝色障碍物个数
 */
int find_blue_obstacle(Mat &img, int &obs_x)
{
    obs_x = 0;
    if (img.empty())
        return 0;

    Mat hsv, mask;
    cvtColor(img, hsv, COLOR_BGR2HSV);

    // ---- 1. 蓝色范围（HSV），与 Python 版一致 ----
    Scalar lower_blue(100, 80, 80);
    Scalar upper_blue(130, 255, 255);
    inRange(hsv, lower_blue, upper_blue, mask);

    // ---- 2. 形态学去噪：开运算去小噪点，闭运算填补空洞 ----
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    // ---- 3. 找外轮廓 ----
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // ---- 4. 面积筛选（与 Python 版一致）----
    const double minArea = 30;
    const double maxArea = 320;

    int count = 0;
    double bestArea = 0;

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area < minArea || area > maxArea)
            continue;

        Rect box = boundingRect(contours[i]);
        int cx = box.x + box.width / 2;
        int cy = box.y + box.height / 2;

        // 取面积最大的障碍物作为避障参考
        if (area > bestArea)
        {
            bestArea = area;
            obs_x = cx;
        }
        count++;

        // ---- 5. 可视化：画框、中心点、坐标 ----
        rectangle(img, Point(box.x, box.y),
                  Point(box.x + box.width, box.y + box.height),
                  Scalar(0, 255, 0), 2);
        circle(img, Point(cx, cy), 5, Scalar(0, 0, 255), -1);

        char buf[64];
        snprintf(buf, sizeof(buf), "(%d,%d)", cx, cy);
        putText(img, buf, Point(box.x, box.y - 10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
    }

    // ---- 6. 可视化调试 ----
    if (isVisual)
    {
        imshow("obstacle mask", mask);
        if (count == 0)
            cout << "未检测到蓝色障碍物" << endl;
    }

    return count;
}
