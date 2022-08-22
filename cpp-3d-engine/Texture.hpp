#pragma once
#include "eigen3/Eigen/Dense"
#include "opencv2/opencv.hpp"
#include <string>

class Texture{
    public:
        Texture(cv::Mat image):
        image(image)
        {}
        Texture(std::string image_path):
        image(cv::imread(image_path))
        {}
        cv::Vec3b get_pixel(float u, float v){
            return image.at<cv::Vec3b>((image.rows-1)*u, (image.cols-1)*v);
        }
    private: 
        cv::Mat image;
};