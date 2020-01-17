#include <opencv2/opencv.hpp>
#include <cstdio>

#define EROSION_SIZE 5
#define EDGE_THRESH 1
#define EDGE_THRESH_SCHARR 1

using namespace cv;
using namespace std;

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

//  float angle = (sum / lines.size()) * 180.f / (float) CV_PI;

  return sum / lines.size();
}

int main(int argc, const char** argv) {
//  Mat image = imread(samples::findFile("../dataset/notslot/image0.jpg"));

  VideoCapture cap("dataset/notslot/video0.mp4");

  if(!cap.isOpened()){
    printf("Error opening video stream or file\n");
    return -1;
  }

  namedWindow("win", WINDOW_NORMAL);

  int frameCount = 0;
  cv::VideoWriter output("out.mp4", cap.get(CAP_PROP_FOURCC), cap.get(CAP_PROP_FPS), cv::Size(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT)));

  while(true){
    Mat frame;
    cap >> frame;
    frameCount++;

    if (frame.empty()) break;

    cv::Size size = frame.size();
    float halfWidth = size.width / 2;
    float angle = perform(&frame);
    Point pt1 = Point(halfWidth, size.height);
    Point pt2 = Point(halfWidth + halfWidth * cos(angle), size.height - halfWidth * sin(angle));
    line(frame, pt1, pt2, Scalar(0, 255, 0), 10, LINE_AA);

    float degree = angle * 180.f / (float) CV_PI;
    float percentage = cap.get(CAP_PROP_FRAME_COUNT) / frameCount;
    printf("%f degrees, %f%\n", degree, percentage);

    imshow("win", frame);
    output.write(frame);

    if((char) waitKey(25) == 27) break;
  }

  cap.release();
  destroyAllWindows();

  return 0;
}