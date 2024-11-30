#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;

int hsv_pill_g_min[3] = {29, 3, 52};
int hsv_pill_g_max[3] = {103, 77, 155};
int hsv_pill_b_min[3] = {106, 38, 129};
int hsv_pill_b_max[3] = {148, 129, 227};

int img_width = 672, img_height = 376;
int roi_width = 300, roi_height = 100;
int roi_y_start = 90;
Rect roi(img_width / 2 - roi_width / 2, roi_y_start, roi_width, roi_height);

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        Mat img_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat img_show = img_raw.clone();

        GaussianBlur(img_raw, img_raw, Size(5, 5), 0, 0);
        
        Mat img_hsv, img_hsv_split;
        cvtColor(img_raw, img_hsv, COLOR_BGR2HSV); // convert the image to HSV
        
        Mat img_hsv_split_g, img_hsv_split_b;
        inRange(img_hsv, Scalar(hsv_pill_g_min[0], hsv_pill_g_min[1], hsv_pill_g_min[2]), Scalar(hsv_pill_g_max[0], hsv_pill_g_max[1], hsv_pill_g_max[2]), img_hsv_split_g);
        inRange(img_hsv, Scalar(hsv_pill_b_min[0], hsv_pill_b_min[1], hsv_pill_b_min[2]), Scalar(hsv_pill_b_max[0], hsv_pill_b_max[1], hsv_pill_b_max[2]), img_hsv_split_b);

        // open operation
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(img_hsv_split_g, img_hsv_split_g, MORPH_OPEN, element);
        // morphologyEx(img_hsv_split_b, img_hsv_split_b, MORPH_OPEN, element);
        imshow("Green spilt", img_hsv_split_g);
        imshow("Blue spilt", img_hsv_split_b);

        // find the contours within roi
        std::vector<std::vector<Point>> contours_g, contours_b;
        findContours(img_hsv_split_g(roi), contours_g, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        findContours(img_hsv_split_b(roi), contours_b, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        // draw the contours
        for (int i = 0; i < contours_g.size(); i++) {
            drawContours(img_show(roi), contours_g, i, Scalar(0, 255, 0), 2);
        }
        for (int i = 0; i < contours_b.size(); i++) {
            drawContours(img_show(roi), contours_b, i, Scalar(255, 0, 0), 2);
        }

        // draw the roi
        rectangle(img_show, roi, Scalar(0, 0, 255), 2);
        imshow("img_show", img_show);

        // count the number of contours
        printf("Green number: %ld, Blue number: %ld\n", contours_g.size(), contours_b.size());

        // caculate the average area of the contours
        float avg_area_g = 0, avg_area_b = 0;
        for (int i = 0; i < contours_g.size(); i++) {
            avg_area_g += contourArea(contours_g[i]);
        }
        for (int i = 0; i < contours_b.size(); i++) {
            avg_area_b += contourArea(contours_b[i]);
        }
        avg_area_g /= contours_g.size();
        avg_area_b /= contours_b.size();
        printf("Green area: %f, Blue area: %f\n", avg_area_g, avg_area_b);

        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pill_recognition_tool");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/image", 1, imageCallback);

    ros::spin();
    return 0;
}
