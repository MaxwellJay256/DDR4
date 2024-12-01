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
int hsv_cone_min[3] = {0, 142, 88};
int hsv_cone_max[3] = {183, 255, 255};
int hsv_pill_g_min[3] = {29, 3, 52};
int hsv_pill_g_max[3] = {103, 77, 155};
int hsv_pill_b_min[3] = {106, 38, 129};
int hsv_pill_b_max[3] = {148, 129, 227};

int img_width = 672, img_height = 376;
int roi_width = 300, roi_height = 100;
int roi_y_start = 90;

int roi_cone_width= 150,roi_cone_height = 100;
int roi_cone_y_start = 200;

int cone_pixel_thershold_in = 9000;
int cone_pixel_thershold_out = 5000;
int move_turn_error = 400;

Rect roi_pill(img_width / 2 - roi_width / 2, roi_y_start, roi_width, roi_height);
Rect roi_cone(img_width /2 - roi_cone_width / 2, roi_cone_y_start,roi_cone_width,roi_cone_height);

void findLane(const Mat &img, int lane_center[][3], int num_rows);
geometry_msgs::Twist patrolControl(float error);

Mat img_raw, img_show;
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        img_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (img_raw.empty()) {
            ROS_ERROR("Received empty image");
            return;
        }
        ROS_DEBUG("Receiving image from camera");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

geometry_msgs::Twist msg_patrol;

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/image", 1, imageCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    int phase = 0;

    while (ros::ok())
    {
        ros::spinOnce(); // check for incoming messages
        waitKey(100);
        if (img_raw.empty()) {
            ROS_WARN("Waiting for image...");
            continue;
        }

        imshow("raw", img_raw); // why?
        // Gaussian blur
        Mat img_blur;
        Mat img_show = img_raw.clone();
        GaussianBlur(img_raw, img_blur, Size(3, 3), 0, 0);
        // imshow("blur", img_blur);

        // Color Split
        Mat img_hsv, img_hsv_split;
        cvtColor(img_blur, img_hsv, COLOR_BGR2HSV); // convert the image to HSV
        colorSplit(img_hsv, img_hsv_split);
        // imshow("color split", img_hsv_split);

        Mat img_hsv_split_g, img_hsv_split_b;
        inRange(img_hsv, Scalar(hsv_pill_g_min[0], hsv_pill_g_min[1], hsv_pill_g_min[2]), Scalar(hsv_pill_g_max[0], hsv_pill_g_max[1], hsv_pill_g_max[2]), img_hsv_split_g);
        inRange(img_hsv, Scalar(hsv_pill_b_min[0], hsv_pill_b_min[1], hsv_pill_b_min[2]), Scalar(hsv_pill_b_max[0], hsv_pill_b_max[1], hsv_pill_b_max[2]), img_hsv_split_b);

        // open operation
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(img_hsv_split_g, img_hsv_split_g, MORPH_OPEN, element);
        morphologyEx(img_hsv_split_b, img_hsv_split_b, MORPH_OPEN, element);
        imshow("Green spilt", img_hsv_split_g);
        imshow("Blue spilt", img_hsv_split_b);

        // find the contours within roi_pill
        std::vector<std::vector<Point>> contours_g, contours_b;
        findContours(img_hsv_split_g(roi_pill), contours_g, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        findContours(img_hsv_split_b(roi_pill), contours_b, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        // draw the contours
        for (int i = 0; i < contours_g.size(); i++) {
            drawContours(img_show(roi_pill), contours_g, i, Scalar(0, 255, 0), 2);
        }
        for (int i = 0; i < contours_b.size(); i++) {
            drawContours(img_show(roi_pill), contours_b, i, Scalar(255, 0, 0), 2);
        }

        // draw roi_pill
        rectangle(img_show, roi_pill, Scalar(0, 0, 255), 2);
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
        
        float variance_g = 65535,variance_b = 65535;
        float area_square_sum_g = 0, area_square_sum_b = 0;
        if(contours_g.size() > 1)
        {
            for(int i =0; i < contours_g.size(); i++)
            {
                area_square_sum_g += (contourArea(contours_g[i]) - avg_area_g)*(contourArea(contours_g[i]) - avg_area_g);
            }
            variance_g = area_square_sum_g / contours_g.size();
        }
        if(contours_b.size() > 1)
        {
            for(int i = 0; i < contours_b.size(); i++)
            {
                area_square_sum_b += (contourArea(contours_b[i]) - avg_area_b)*(contourArea(contours_b[i]) - avg_area_b);
            }
            variance_b = area_square_sum_b / contours_b.size();
        }
        printf("Green area: %f, Blue area: %f\n", avg_area_g, avg_area_b);
        printf("Green area var: %f , Blue area var: % f \n", variance_g, variance_b);
        
        // Mat image_hsv_spilt_cone;
        // inRange(img_hsv,Scalar(hsv_cone_min[0],hsv_cone_min[1],hsv_cone_min[2]),Scalar(hsv_cone_max[0],hsv_cone_max[1],hsv_cone_max[2]),image_hsv_spilt_cone);
        // rectangle(img_show,roi_cone,Scalar(0,255,0),2);
        // imshow("img_show", img_show);
        // imshow("cone",image_hsv_spilt_cone);
        // Mat roiImage_cone = image_hsv_spilt_cone(roi_cone);
        // int cone_pixel_count = countNonZero(roiImage_cone);
        // imshow("cone_roi",roiImage_cone);
        // printf("cone_pixel_count: %d\n",cone_pixel_count);

        // int num_rows = 100;
        // int lane_center[num_rows][3] = {0};
        // findLane(img_hsv_split, lane_center, num_rows);

        float error = 0;
        // for (int i = img_show.rows - 1; i > img_show.rows - num_rows; i--) {
        //     circle(img_show, Point(lane_center[i][0], i), 1, Scalar(0, 0, 255), -1);
        //     circle(img_show, Point(lane_center[i][1], i), 1, Scalar(0, 0, 255), -1);
        //     circle(img_show, Point(lane_center[i][2], i), 1, Scalar(0, 255, 255), -1);

        //     error += (float)(lane_center[i][2] - img_show.cols / 2) / (float)num_rows;
        // }

        // // draw a line in the center
        // Point pt_mid_top(cvRound(img_show.cols / 2), img_show.rows - num_rows);
        // Point pt_mid_bottom(cvRound(img_show.cols / 2), cvRound(img_show.rows - 1));
        // line(img_show, pt_mid_bottom, pt_mid_top, Scalar(255, 0, 255), 2, LINE_AA);

        // printf("Error: %f\n", error);
        // imshow("Image", img_show);
        

        // panduan
        // if(cone_pixel_count > cone_pixel_thershold_in)
        // {
        //     phase = 1;
        // }
        // if(cone_pixel_count < cone_pixel_thershold_out)
        // {
        //     phase = 0;
        // }
        // if(phase == 1)
        // {
        //     if(variance_g > variance_b) 
        //     {
        //         error = move_turn_error;
        //     }
        //     else
        //     {
        //         error = -move_turn_error;
        //     }
        // }

        // msg_patrol = patrolControl(error);
        // printf("Linear: %f, Angular: %f\n", msg_patrol.linear.x, msg_patrol.angular.z);
        // vel_pub.publish(msg_patrol);
    }

    return 0;
}

void colorSplit(const Mat &hsv_input, Mat &thresholded_output) {
    inRange(hsv_input, Scalar(hsv_cone_min[0], hsv_cone_min[1], hsv_cone_min[2]), Scalar(hsv_cone_max[0], hsv_cone_max[1], hsv_cone_max[2]), thresholded_output);
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

geometry_msgs::Twist patrolControl(float error)
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.08;
    msg.angular.z = -error * 0.005;
    // msg.linear.x = 0;
    // msg.angular.z = 0;
    return msg;
}
