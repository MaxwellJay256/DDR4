#include <stdlib.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

enum CameraState {
    COMPUTER = 0,
    ZED,
    REALSENSE
};

CameraState cameraState = REALSENSE;
using namespace cv;

Mat frame_msg;
void rcvCameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    frame_msg = cv_ptr->image;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera");
    ros::NodeHandle n;
    ros::Subscriber camera_sub;
    ros::Publisher camera_pub = n.advertise<sensor_msgs::Image>("camera/image", 1);
    VideoCapture capture;
    
    switch (cameraState)
    {
    case COMPUTER:
        capture.open(0);
        if (!capture.isOpened()) {
            ROS_ERROR("Computer camera is not opened.");
            return -1;
        }
        break;
    
    case ZED:
        capture.open(2);
        if (!capture.isOpened()) {
            ROS_ERROR("ZED camera is not opened.");
            return -1;
        }
        break;

    case REALSENSE:
        camera_sub = n.subscribe("/camera/color/image_raw", 1, rcvCameraCallback);
        break;

    default:
        break;
    }

    Mat frIn;
    while (ros::ok())
    {
        switch (cameraState)
        {
        case COMPUTER:
            capture.read(frIn);
            if (frIn.empty()) {
                ROS_ERROR("No image from computer camera.");
                continue;
            }
            break;
        
        case ZED:
            capture.read(frIn);
            if (frIn.empty()) {
                ROS_ERROR("No image from ZED camera.");
                return -1;
            }
            frIn = frIn(Rect(0, 0, frIn.cols / 2, frIn.rows)); // Only use left camera
            break;

        case REALSENSE:
            if (frame_msg.cols == 0) {
                ROS_ERROR("No image from RealSense camera.");
                ros::spinOnce();
                return -1;
            }
            frIn = frame_msg;
            break;

        default:
            break;
        }

        if (!frIn.empty()) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frIn).toImageMsg();
            camera_pub.publish(msg);
        }

        waitKey(10);

        ros::spinOnce(); // Handle callback
    }

    return 0;
}
