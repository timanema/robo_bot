#include <opencv2/opencv.hpp>
#include <cstdio>
#include <chrono>

using namespace cv;
using namespace std;

#define VERT_DIF 25
#define VERT_CUT 2
#define BIAS_START 1
#define BIAS_ANGLE 10
#define BIAS_CUT 5
#define BOUND_CUT 0
#define BOUND_ANGLE 15

#define BIAS_LEN 0.1
#define BOUND_LEN 0.25
#define ANGLE_OVERRIDE 40

float perform(const Mat *i) {
    Mat input = *i;

    cv::Rect roi;
    roi.x = 0;
    roi.y = input.size().height * 0.5;
    roi.width = input.size().width;
    roi.height = input.size().height * 0.25;

    int leftBound = input.size().width * (0.5 - BOUND_LEN);
    int rightBound = input.size().width * (0.5 + BOUND_LEN);

    Mat blur;
    medianBlur(input(roi), blur, 7);
    medianBlur(input(roi), blur, 7);

    Mat gray;
    inRange(blur, Scalar(80,80,80), Scalar(255,255,255), gray);

    Mat edge;
    Canny(gray, edge, 50, 100);

    vector<Vec4i> lines;
    HoughLinesP(edge, lines, 1, CV_PI / 180, 50, 30, 5);
    float verSum = 0.f;
    float horSum = 0.f;
    int verCnt = 0;
    int horCnt = 0;
    int leftBoundCntVer = 0;
    int rightBoundCntVer = 0;
    int leftBoundCntHor = 0;
    int rightBoundCntHor = 0;
    int horBias = 0;
    int verBias = 0;
    float biasBorderMin = input.size().width * (0.5 - BIAS_LEN);
    float biasBorderMax = input.size().width * (0.5 + BIAS_LEN);

    for (auto l : lines) {
        // check if hor
        bool hor = abs(l[0] - l[2]) > VERT_DIF;
        bool outOfLeft = l[0] < leftBound || l[2] < leftBound;
        bool outOfRight = l[0] > rightBound || l[2] > rightBound;

        Point p1;
        Point p2;

        if (l[1] < l[3]) {
            p1 = Point(l[0], l[1]);
            p2 = Point(l[2], l[3]);
        } else {
            p2 = Point(l[0], l[1]);
            p1 = Point(l[2], l[3]);
        }

        float angle = -atan2(p1.y - p2.y, p1.x - p2.x);
        bool leftBias = l[0] < biasBorderMin || l[2] < biasBorderMin;
        bool rightBias = l[0] > biasBorderMax || l[2] > biasBorderMax;

        if (outOfLeft || outOfRight) {
            if (hor) {
                leftBoundCntHor += outOfLeft ? 1 : 0;
                rightBoundCntHor += outOfRight ? 1 : 0;
                horBias -= leftBias;
                horBias += rightBias;
            } else {
                leftBoundCntVer += outOfLeft ? 1 : 0;
                rightBoundCntVer += outOfRight ? 1 : 0;
                verBias -= leftBias;
                verBias += rightBias;
            }
        } else {
            if (hor) {
                horSum += angle;
                horCnt += 1;
                horBias += leftBias ? -1 : rightBias ? 1 : 0;
            } else {
                verSum += angle;
                verCnt += 1;
                verBias += leftBias ? -1 : rightBias ? 1 : 0;
            }
        }

        line(edge, p1, p2, Scalar(255, 255, 255), 3, LINE_AA);
//        printf("Found line: %5.2f, %d\n", angle * 180.f / (float) CV_PI, hor);
    }

    // draw bounds
    Point left_p1 = Point(input.size().width * (0.5 - BOUND_LEN), 0);
    Point left_p2 = Point(input.size().width * (0.5 - BOUND_LEN), input.size().height);
    Point right_p1 = Point(input.size().width * (0.5 + BOUND_LEN), 0);
    Point right_p2 = Point(input.size().width * (0.5 + BOUND_LEN), input.size().height);
    line(edge, left_p1, left_p2, Scalar(255, 255, 255), 5, LINE_AA);
    line(edge, right_p1, right_p2, Scalar(255, 255, 255), 5, LINE_AA);

    imshow("test", edge);
//    waitKey(0);

    bool includeHor = verCnt < VERT_CUT;
    int bias = verBias + (includeHor ? horBias : 0);
    float biasDegree = bias < -BIAS_START ? BIAS_ANGLE : bias > BIAS_START ? -BIAS_ANGLE : 0;
    float sum = verSum + (includeHor ? horSum : 0);
    int cnt = verCnt + (includeHor ? horCnt : 0);
    float angle = sum / cnt + (biasDegree * CV_PI / 180.f);
//    printf("Bias %d, hor %d, ", bias, includeHor);

    float  minAngle = 0;
    float  maxAngle = CV_PI;

    if (bias < -BIAS_CUT) {
        minAngle = 0.5 * CV_PI;
    }

    if (bias > BIAS_CUT) {
        maxAngle = 0.5 * CV_PI;
    }

    angle = min(max(minAngle, angle), maxAngle);

    float forceLeft = min(max(minAngle, (90.f + BOUND_ANGLE) * (float) CV_PI / 180.f), maxAngle);
    float forceRight = min(max(minAngle,  (90.f - BOUND_ANGLE) * (float) CV_PI / 180.f), maxAngle);
    float forceStraight = min(max(minAngle, (90.f + biasDegree) * (float) CV_PI / 180.f), maxAngle);

    if ((angle > forceLeft + ANGLE_OVERRIDE || angle < forceRight - ANGLE_OVERRIDE) && sum > 0) {
        return angle;
    }

    if (leftBoundCntVer > BOUND_CUT) {
        if (leftBoundCntVer >= verCnt) {
            return forceLeft;
        }

        return forceStraight;
    }

    if (rightBoundCntVer > BOUND_CUT) {
        if (leftBoundCntVer >= verCnt) {
            return forceRight;
        }

        return forceStraight;
    }

    if (leftBoundCntHor > BOUND_CUT && includeHor) {
        if (leftBoundCntHor >= horCnt) {
            return forceLeft;
        }

        return forceStraight;
    }

    if (rightBoundCntHor > BOUND_CUT && includeHor) {
        if (leftBoundCntVer >= horCnt) {
            return forceRight;
        }

        return forceStraight;
    }

    // signal no line found
    if (sum <= 0) {
        return -42;
    }

    return angle;
}

int main(int argc, const char **argv) {
//  Mat image = imread(samples::findFile("../dataset/notslot/image0.jpg"));

    VideoCapture cap("dataset/notslot/video2.mp4");

    if (!cap.isOpened()) {
        printf("Error opening video stream or file\n");
        return -1;
    }

    namedWindow("win", WINDOW_NORMAL);

    int frameCount = 0;
    cv::VideoWriter output("out.mp4", cap.get(CAP_PROP_FOURCC), cap.get(CAP_PROP_FPS),
                           cv::Size(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT)));

    float prev_angle = 0.5f * (float) CV_PI;

    while (true) {
        Mat frame;
        cap >> frame;
        frameCount++;

        if (frame.empty()) break;

        cv::Size size = frame.size();
        float halfWidth = size.width / 2;
        auto start = chrono::high_resolution_clock::now();
        float angle = perform(&frame);
        float dur = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start).count();
        float fps = 1000.f / dur;

        if (angle < -41) {
            angle = prev_angle;
//            printf("aaaah");
            //TODO: Set counter somewhere to indicate that the robot is lost
        }

        Point pt1 = Point(halfWidth, size.height);
        Point pt2 = Point(halfWidth + halfWidth * cos(angle), size.height - halfWidth * sin(angle));
        line(frame, pt1, pt2, Scalar(0, 255, 0), 10, LINE_AA);

        float degree = angle * 180.f / (float) CV_PI;
        float percentage = cap.get(CAP_PROP_FRAME_COUNT) / frameCount;
//        printf("angle %f, time %fms, fps %f\n", degree, dur, fps);
//        printf("%f degrees, %f%\n", degree, percentage);

        imshow("win", frame);
        output.write(frame);

        prev_angle = angle;
        if ((char) waitKey(25) == 27) break;
    }

    cap.release();
    destroyAllWindows();

    return 0;
}