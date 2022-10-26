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
            return image.at<cv::Vec3b>((image.rows-1)*(1-v), (image.cols-1)*u);
        }
        cv::Vec3b get_pixel(Eigen::Vector2f coords){
            return get_pixel(coords[0], coords[1]);
        }
        cv::Mat get_image(){
            return image;
        }
        Texture copy(){
            Texture ret;
            ret.image = image.clone();
            return ret;
        }
    private: 
        cv::Mat image;
};

// template <typename _T, unsigned int _rows, unsigned int _cols, unsigned int _depth> 
// class Texture_{
//     public:
//         Texture_(){
//         }

//         template <typename T, unsigned int rows, unsigned int cols, unsigned int depth> 
//         static auto fill(T value)
//         {
//             Texture_ ret = Texture_<T, rows, cols, depth>();
//             ret.buffer.fill(value);
//             return ret;
//         }

//         static auto fromImage()

//         // T get_pixel(float u, float v){
//         //     return image.at<cv::Vec3b>((image.rows-1)*u, (image.cols-1)*v);
//         // }
//         // cv::Vec3b get_pixel(Eigen::Vector2f coords){
//         //     return get_pixel(coords[0], coords[1]);
//         // }
//     private: 
//         std::array<_T, _rows*_cols*_depth> buffer;
// };