#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;

void colorSplit(const Mat &hsv_input, Mat &thresholded_output);
int hsv_min[3] = {0, 142, 88};
int hsv_max[3] = {183, 255, 255};

void findLane(const Mat &img, int lane_center[][3], int num_rows);

Mat img_raw;
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        img_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
        // Gaussian blur
        GaussianBlur(img_raw, img_raw, Size(5, 5), 0);

        // Color Split
        Mat img_hsv, img_hsv_split;
        cvtColor(img_raw, img_hsv, COLOR_BGR2HSV); // convert the image to HSV
        colorSplit(img_hsv, img_hsv_split);
        imshow("color split", img_hsv_split);

        int num_rows = 100;
        int lane_center[num_rows][3] = {0};
        findLane(img_hsv_split, lane_center, num_rows);

        Mat img_show = img_raw.clone();
        for (int i = img_show.rows - 1; i > img_show.rows - num_rows; i--) {
            circle(img_show, Point(lane_center[i][0], i), 1, Scalar(0, 0, 255), -1);
            circle(img_show, Point(lane_center[i][1], i), 1, Scalar(0, 0, 255), -1);
            circle(img_show, Point(lane_center[i][2], i), 1, Scalar(0, 255, 255), -1);
        }
        
        imshow("Image", img_show);
        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/image", 1, imageCallback);

    ros::spin();
    return 0;
}

void colorSplit(const Mat &hsv_input, Mat &thresholded_output) {
    inRange(hsv_input, Scalar(hsv_min[0], hsv_min[1], hsv_min[2]), Scalar(hsv_max[0], hsv_max[1], hsv_max[2]), thresholded_output);
}

void findLane(const Mat &img, int lane_center[][3], int num_rows) {
    // Split img into left side and right side
    Rect left_frame(0, 0, static_cast<int>(img.cols / 2), static_cast<int>(img.rows));
    Rect right_frame(static_cast<int>(img.cols / 2), 0, static_cast<int>(img.cols / 2), static_cast<int>(img.rows));
    Mat img_left = img(left_frame);
    Mat img_right = img(right_frame);

    // imshow("left", img_left);
    
    for (int i = img.rows - 1; i > img.rows - num_rows; i--)
    {
        
        // Left lane
        lane_center[i][0] = 0;
        
        for (int j = img_left.cols - 1; j > 0; j--) {
            if (img_left.at<uint8_t>(i, j) == 255) {
                lane_center[i][0] = j;
                break;
            }
        }

        // Right lane
        lane_center[i][1] = img.cols - 1;
        for (int j = 0; j < img_right.cols; j++) {
            if (img_right.at<uint8_t>(i, j) == 255) {
                lane_center[i][1] = img.cols / 2 + j;
                
                break;
            }
        }

        lane_center[i][2] = (lane_center[i][0] + lane_center[i][1]) / 2;
    }
    
}
