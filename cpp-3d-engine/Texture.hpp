#pragma once
#include "eigen3/Eigen/Dense"
#include "opencv2/opencv.hpp"
#include <string>

class Texture{
    public:
        Texture(){
            this->image = cv::Mat::zeros(1,1,CV_8UC3);
            this->image.setTo(cv::Vec3b(255,255,255));
        }
        Texture(cv::Mat image):
        image(image)
        {}
        Texture(std::string image_path):
        image(cv::imread(image_path))
        {}
        cv::Vec3b get_pixel(float u, float v){
            return image.at<cv::Vec3b>((image.rows-1)*u, (image.cols-1)*v);
        }
        cv::Vec3b get_pixel(Eigen::Vector2f coords){
            return get_pixel(coords[0], coords[1]);
        }
    private: 
        cv::Mat image;
};