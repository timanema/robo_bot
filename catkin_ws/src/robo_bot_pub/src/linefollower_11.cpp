#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>

#include <thread>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <chrono>

using namespace cv;
using namespace std;

// Dany vars
#define EROSION_SIZE 5
#define EDGE_THRESH 1
#define EDGE_THRESH_SCHARR 1

// Tim vars
#define HOR_DIF 35
#define VERT_CUT 2
#define BIAS_START 1
#define BIAS_ANGLE 45
#define BIAS_CUT 3
#define BOUND_CUT 0
#define BOUND_ANGLE 80

#define BIAS_LEN 0.1
#define BOUND_LEN 0.25
#define ANGLE_OVERRIDE 45

#define SELF_CORRECT true

#define EROSION_SIZE 5

#define ROI_Y_START 0.65
#define ROI_Y_SIZE 0.25
#define ROI_X_START 0
#define ROI_X_SIZE 1

// Algo select
#define TIM

#ifdef DANY
float perform(const Mat *input) {
  Mat gray;
  cvtColor((*input), gray, COLOR_RGB2GRAY);

  cv::Rect roi;
  roi.x = 0;
  roi.y = gray.size().height * 0.5;
  roi.width = gray.size().width;
  roi.height = gray.size().height * 0.2;

  Mat thresh;
  threshold(gray(roi), thresh, 100, 255, THRESH_BINARY_INV);

  Mat resized;
  resize(thresh, resized, Size(600, 600), INTER_LINEAR);

  Mat eroded;
  Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * EROSION_SIZE + 1, 2 * EROSION_SIZE + 1), Point(EROSION_SIZE, EROSION_SIZE));
  erode(resized, eroded, element);

  Mat blurImage;
  Mat edge1;
  blur(eroded, blurImage, Size(3,3));
  Canny(blurImage, edge1, EDGE_THRESH, EDGE_THRESH * 3, 3);

  Mat dx,dy;
  Mat edge2;
  Scharr(blurImage,dx,CV_16S,1,0);
  Scharr(blurImage,dy,CV_16S,0,1);
  Canny(dx, dy, edge2, EDGE_THRESH_SCHARR, EDGE_THRESH_SCHARR * 3);

  Mat rgb;
  cvtColor(blurImage, rgb, COLOR_GRAY2RGB);

  vector<Vec4i> lines;
  float sum = 0.f;
  HoughLinesP(blurImage, lines, 1, CV_PI /180, 200, 100, 50);

  for(auto l : lines){
    Point pt1 = Point(l[0], l[1]);
    Point pt2 = Point(l[2], l[3]);
    float angle = atan2(pt1.y - pt2.y, pt1.x - pt2.x);
    sum += abs(angle);

    line(rgb, pt1, pt2, Scalar(255, 0, 0), 3, LINE_AA);
  }

  return sum / lines.size();
}
#else
float perform(const Mat *i) {
    Mat input = *i;

    // Create ROI
    cv::Rect roi;
    roi.x = input.size().width * ROI_X_START;
    roi.y = input.size().height * ROI_Y_START;
    roi.width = input.size().width * ROI_X_SIZE;
    roi.height = input.size().height * ROI_Y_SIZE;

    // Get bounds
    int leftBound = input.size().width * (0.5 - BOUND_LEN);
    int rightBound = input.size().width * (0.5 + BOUND_LEN);

    // Do some image processing (blur -> threshold -> erode -> edge)
    Mat blur;
    medianBlur(input(roi), blur, 7);
    medianBlur(input(roi), blur, 7);

    Mat gray;
    inRange(blur, Scalar(80,80,80), Scalar(255,255,255), gray);

    Mat eroded;
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * EROSION_SIZE + 1, 2 * EROSION_SIZE + 1), Point(EROSION_SIZE, EROSION_SIZE));
    erode(gray, eroded, element);

    Mat edge;
    Canny(eroded, edge, 50, 100);

    // Retrieve the lines
    vector<Vec4i> lines;
    HoughLinesP(edge, lines, 1, CV_PI / 180, 50, 30, 5);
  
    // Define all counters used
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

    // For each detected line, we draw it and check its positition in order to increment the correct counters
    for (auto l : lines) {
        // check if hor
        bool hor = abs(l[0] - l[2]) > HOR_DIF;
        bool outOfLeft = l[0] < leftBound || l[2] < leftBound;
        bool outOfRight = l[0] > rightBound || l[2] > rightBound;

        // Get the 'highest' point
        Point p1;
        Point p2;

        if (l[1] < l[3]) {
            p1 = Point(l[0], l[1]);
            p2 = Point(l[2], l[3]);
        } else {
            p2 = Point(l[0], l[1]);
            p1 = Point(l[2], l[3]);
        }

        // Calculate the angle
        float angle = -atan2(p1.y - p2.y, p1.x - p2.x);
      
        // Check if this line is more one one side on the screen than the other
        bool leftBias = l[0] < biasBorderMin || l[2] < biasBorderMin;
        bool rightBias = l[0] > biasBorderMax || l[2] > biasBorderMax;

        // Increment correct counters, should be self-explanatory
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
    }

    // Draw bouds
    Point left_p1 = Point(input.size().width * (0.5 - BOUND_LEN), 0);
    Point left_p2 = Point(input.size().width * (0.5 - BOUND_LEN), input.size().height);
    Point right_p1 = Point(input.size().width * (0.5 + BOUND_LEN), 0);
    Point right_p2 = Point(input.size().width * (0.5 + BOUND_LEN), input.size().height);
    line(edge, left_p1, left_p2, Scalar(255, 255, 255), 5, LINE_AA);
    line(edge, right_p1, right_p2, Scalar(255, 255, 255), 5, LINE_AA);

    // Show the resulting image
    imshow("RoboVision™", edge);

    // Check if we should include horizontal lines, and if so include them
    bool includeHor = verCnt < VERT_CUT;
    int bias = verBias + (includeHor ? horBias : 0);
    float biasDegree = bias < -BIAS_START ? BIAS_ANGLE : bias > BIAS_START ? -BIAS_ANGLE : 0;
    float sum = verSum + (includeHor ? horSum : 0);
    int cnt = verCnt + (includeHor ? horCnt : 0);
    float angle = sum / cnt + (SELF_CORRECT ? (biasDegree * CV_PI / 180.f) : 0);

    float  minAngle = 0;
    float  maxAngle = CV_PI;

    // Check if the robot should correct its course (which it basically always should except for some tests)
    if (SELF_CORRECT) {
        // Check if we should cap the angle based on the line bias
        if (bias < -BIAS_CUT) {
            minAngle = 0.5 * CV_PI;
        }

        if (bias > BIAS_CUT) {
            maxAngle = 0.5 * CV_PI;
        }

        // Cap the angle
        angle = min(max(minAngle, angle), maxAngle);

        // Calculate the angles of the robot we return, if it should correct itself
        float forceLeft = min(max(minAngle, (90.f + BOUND_ANGLE) * (float) CV_PI / 180.f), maxAngle);
        float forceRight = min(max(minAngle,  (90.f - BOUND_ANGLE) * (float) CV_PI / 180.f), maxAngle);
        float forceStraight = min(max(minAngle, (90.f + biasDegree) * (float) CV_PI / 180.f), maxAngle);

        // If the angle of the line is higher than our correction + some margin, just use the angle
        if ((angle > forceLeft + ANGLE_OVERRIDE || angle < forceRight - ANGLE_OVERRIDE) && sum > 0) {
            return angle;
        }

        // Depending on how many lines are out of bounds and where we can force the robot to the left, right and straight
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
    }

    // If there was no line we simply return a magic number
    if (sum <= 0) {
        return -42;
    }

    // Nothing special, so just follow the line
    return angle;
}
#endif

float prev_angle = 0.5f * (float) CV_PI;
float degree = 90;
int lost_count = 0;

void ros_callback(const sensor_msgs::CompressedImageConstPtr& img) {
        Mat raw = imdecode(Mat(img->data), 1);
        Mat frame;

        // Rotate and crop the received image from (in our case) 2K to 480p
        rotate(raw, raw, ROTATE_90_CLOCKWISE);
        resize(raw, frame, Size(480, 640), INTER_LINEAR);

        cv::Size size = frame.size();
        float halfWidth = size.width / 2;
        float angle = perform(&frame);

        // If there were no lines, continue 
        if (angle < -41) {
            angle = prev_angle;
            lost_count += 1;
        }

        // Draw the line the robot will follow
        Point pt1 = Point(halfWidth, size.height);
        Point pt2 = Point(halfWidth + halfWidth * cos(angle), size.height - halfWidth * sin(angle));
        line(frame, pt1, pt2, Scalar(0, 255, 0), 10, LINE_AA);

        // Convert radian to degree
        degree = angle * 180.f / (float) CV_PI;
  
        // Convert to twist spec
        degree = -1.f + 2.f * (degree / 180.f);
  
        // Show result
        imshow("PlebVision™", frame);
}

int main(int argc, char **argv) {
   namedWindow("PlebVision™", WINDOW_NORMAL);
   resizeWindow("PlebVision™", 480, 640);
   
   // ros - start

   ros::init(argc, argv, "linefollower");

   ros::NodeHandle nh;
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   ros::Subscriber sub = nh.subscribe<sensor_msgs::CompressedImage>("camera/image/compressed", 1, ros_callback);
   geometry_msgs::Twist msg;

   // ros - end

   while (ros::ok()) {
        // ros - start
        msg.angular.z = degree;
        msg.linear.x = 0.3; // fixed speed of 0.3

        // If the robot hasn't seen a line for a while, set its speed to 0
        // Our robot works at about 40fps (last time we checked), which means the robot will stop after about 2s of no line
        if (lost_count > 100) {
          msg.linear.x = 0.0;
        }
     
        pub.publish(msg);
        ros::spinOnce();
        // ros - end

        if ((char) waitKey(25) == 27) break;
    }

//    cap.release();
    destroyAllWindows();

    return 0;
}
