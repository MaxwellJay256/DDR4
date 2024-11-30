#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
void colorSplitManual(const Mat &hsv_input, Mat &thresholded_output, const char* window_name);

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        Mat img_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
        imshow("Raw Image", img_raw);
        
        Mat img_hsv, img_hsv_split;
        cvtColor(img_raw, img_hsv, COLOR_BGR2HSV); // convert the image to HSV
        colorSplitManual(img_hsv, img_hsv_split, "HSV");

        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "color_split_tool");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/image", 1, imageCallback);

    ros::spin();
    return 0;
}

void colorSplitManual(const Mat &hsv_input, Mat &thresholded_output, const char* window_name) {
    static int hmin = 0, hmax = 255;
    static int smin = 0, smax = 255;
    static int vmin = 0, vmax = 255;
    createTrackbar("Hue Min", window_name, &hmin, 255);
    createTrackbar("Hue Max", window_name, &hmax, 255);
    createTrackbar("Sat Min", window_name, &smin, 255);
    createTrackbar("Sat Max", window_name, &smax, 255);
    createTrackbar("Val Min", window_name, &vmin, 255);
    createTrackbar("Val Max", window_name, &vmax, 255);

    inRange(hsv_input, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), thresholded_output);
    
    // Display the thresholded image
    // Restrict window size
    if (thresholded_output.cols > 640) {
        resize(thresholded_output, thresholded_output, Size(960, 540));
    }
    imshow(window_name, thresholded_output);
}