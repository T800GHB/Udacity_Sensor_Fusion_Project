#include <iostream>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "structIO.hpp"

using namespace std;

void showLidarTopview()
{
    std::vector<LidarPoint> lidarPoints;
    readLidarPts("../dat/C51_LidarPts_0000.dat", lidarPoints);

    cv::Size worldSize(10.0, 20.0); // width and height of sensor field in m
    cv::Size imageSize(1000, 2000); // corresponding top view image in pixel

    float maxVal = worldSize.height;

    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    // plot Lidar points into image
    for (const auto& it : lidarPoints) {
        float xw = it.x; // world position in m with x facing forward from sensor
        float yw = it.y; // world position in m with y facing left from sensor

        int y = static_cast<int>(-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = static_cast<int>(-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

        // 1. Change the color of the Lidar points such that X=0.0m corresponds to red while X=20.0m is shown as green.
        // 2. Remove all Lidar points on the road surface while preserving measurements on the obstacles in the scene.
        double zw = it.z;
        double minZ = -1.42;
        if(zw > minZ) {
            double val = it.x;
            // Adjust color
            int red = min(255, static_cast<int>(255 * abs((val-maxVal) / maxVal)));
            int green = min(255, static_cast<int>(255 * (1 - abs((val - maxVal) / maxVal))));
            // Draw lidar points
            cv::circle(topviewImg, cv::Point(x,y), 5, cv::Scalar(0, green, red), -1);
        }
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i) {
        int y = static_cast<int>(-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::waitKey(0); // wait for key to be pressed
}

int main()
{
    showLidarTopview();
}
