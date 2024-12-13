# DDR4

HITSZ 2024 数字图像处理课程设计任务 1，代号 Drug Delivery Robot 2024，简称 DDR4。

## Basic Information

- Collaborator: [Maxwell Jay](https://github.com/MaxwellJay256)、[CY](https://github.com/CycleYerik)
- Language: `C++`
- Environment: Ubuntu 20.04, ROS Noetic, OpenCV, cv_bridge, dashgo_driver
- ZED camera image: width = 672, height = 376

## Run

Run the commands below:

```bash
catkin_make
source devel/setup.bash
roslaunch drug-delivery-robot drug-delivery-robot.launch
```

## Demo

![blue](demo/blue_record.gif)

![green](demo/green_record.gif)

## Features

- 运行连续流畅，车速适中，无停顿
- 完全基于视觉反馈，没有开环的控制环节——所以你明白你调的参数都是什么
- 算法简单易懂
- [`color_split_tool.cpp`](src/drug-delivery-robot/src/color_split_tool.cpp) 和 [`pill_recognition_tool.cpp`](src/drug-delivery-robot/src/pill_recognition_tool.cpp) 小工具助你调试

## Explanation

机器人需要做 2 件事情：
1. 沿密集路障筒表示的道路前进
2. 在 T 字路口识别两种药片的图片，如果是蓝色药片则左转，绿色药片则右转

### 1. 怎么用视觉避障？
用 HSV 阈值分割出橘色的路障筒，并以此算出车道的中心线，用一个 P 控制器（见 `patrolControl` 函数）调整机器人的 yaw 角速度，使其沿中心线前进。

### 2. 怎么判断进入和离开路口？
划定一个 `roi_cone`（Demo 中绿色矩形框），机器人越接近路口，则框中的路障筒越大，ROI 中的橘色像素点应当越多。

当橘色的像素个数超过阈值 `cone_pixel_thershold_in`，则进入转向状态（`phase = 1`）；如果转向完成，则 `roi_cone` 中橘色像素应当变少，故当 `cone_pixel_count` 小于 `cone_pixel_thershold_out` 时，回到巡线状态（`phase = 0`）。

### 3. 怎么识别药片并转向？
用 HSV 阈值分别分割出蓝色和绿色，用 `findContours` 函数找出药片轮廓，再数出药片个数。

考虑到背景中有很多噪点元素会被误识别，所以这里也划定了一个 ROI（`roi_pill`，Demo 中红色矩形框）。如果绿色轮廓比蓝色轮廓多，则给 1. 中的 error 加一个偏差，使机器人左转；反之则右转。
