#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;

int hsv_pill_g_min[3] = {67, 4, 48};
int hsv_pill_g_max[3] = {117, 102, 164};
int hsv_pill_b_min[3] = {95, 34, 98};
int hsv_pill_b_max[3] = {126, 153, 190};
int hsv_cone_min[3] = {0, 142, 88};
int hsv_cone_max[3] = {183, 255, 255};

int img_width = 672, img_height = 376;
int roi_width = 300, roi_height = 100;
int roi_y_start = 90;

int roi_cone_width= 150,roi_cone_height = 100;
int roi_cone_y_start = 200;

int cone_pixel_thershold = 9000;

Rect roi(img_width / 2 - roi_width / 2, roi_y_start, roi_width, roi_height);
Rect roi_cone(img_width /2 - roi_cone_width / 2, roi_cone_y_start,roi_cone_width,roi_cone_height);

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
        morphologyEx(img_hsv_split_b, img_hsv_split_b, MORPH_OPEN, element);
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
        // imshow("img_show", img_show);

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

        if(contours_g.size() != 0)
        {
            for(int i =0; i < contours_g.size(); i++)
            {
                area_square_sum_g += (contourArea(contours_g[i]) - avg_area_g)*(contourArea(contours_g[i]) - avg_area_g);
            }
            variance_g = area_square_sum_g / contours_g.size();
        }
        if(contours_b.size() != 0)
        {
            for(int i =0; i < contours_b.size(); i++)
            {
                area_square_sum_b += (contourArea(contours_b[i]) - avg_area_b)*(contourArea(contours_b[i]) - avg_area_b);
            }
            variance_b = area_square_sum_b / contours_b.size();
        }
        printf("Green area: %f, Blue area: %f\n", avg_area_g, avg_area_b);
        printf("Green area var : %f , Blue area var : % f \n",variance_g,variance_b);
        
        
        Mat image_hsv_spilt_cone;
        inRange(img_hsv,Scalar(hsv_cone_min[0],hsv_cone_min[1],hsv_cone_min[2]),Scalar(hsv_cone_max[0],hsv_cone_max[1],hsv_cone_max[2]),image_hsv_spilt_cone);
        rectangle(img_show,roi_cone,Scalar(0,255,0),2);
        imshow("img_show", img_show);
        imshow("cone",image_hsv_spilt_cone);
        Mat roiImage_cone = image_hsv_spilt_cone(roi_cone);
        int cone_pixel_count = countNonZero(roiImage_cone);
        imshow("cone_roi",roiImage_cone);
        printf("cone_pixel_count: %d\n",cone_pixel_count);
        

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
