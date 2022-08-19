#pragma once
#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"
#include <string>

class Triangle{
    public:
        Eigen::Vector3f v[3];
        cv::Scalar color;
        Triangle(cv::Scalar color=cv::Scalar(255,255,255)){
            v[0] = Eigen::Vector3f::Zero();
            v[1] = Eigen::Vector3f::Zero();
            v[2] = Eigen::Vector3f::Zero();
            this->color = color; 
        } 
        Triangle(Eigen::Vector3f (&vertices)[3], cv::Scalar color=cv::Scalar(255,255,255)){
            v[0] = vertices[0];
            v[1] = vertices[1];
            v[2] = vertices[2];
            this->color = color; 
        }
        Triangle(std::vector<Eigen::Vector3f> vertices, cv::Scalar color=cv::Scalar(255,255,255)){
            v[0] = vertices[0];
            v[1] = vertices[1];
            v[2] = vertices[2];
            this->color = color; 
        }
        Triangle(Eigen::Vector3f vertex0, Eigen::Vector3f vertex1, Eigen::Vector3f vertex2, cv::Scalar color=cv::Scalar(255,255,255)){
            v[0] = vertex0;
            v[1] = vertex1;
            v[2] = vertex2;
            this->color = color;
        }
        Eigen::Vector3f edge_0(){
            return this->v[1] - this->v[0];
        }
        Eigen::Vector3f edge_1(){
            return this->v[2] - this->v[1];
        }
        Eigen::Vector3f edge_2(){
            return this->v[0] - this->v[2];
        }

        Eigen::Vector3f normal(){
            return this->edge_0().cross(this->edge_1());
        }

        Eigen::Vector3f center(){
            return (this->v[0] + this->v[1] + this->v[2])/3;
        }
        void translate(Eigen::Vector3f v){
            this->v[0] += v;
            this->v[1] += v;
            this->v[2] += v; 
        } 

        void transform(Eigen::Matrix3f t){
            v[0] = t*v[0];
            v[1] = t*v[1];
            v[2] = t*v[2];
        }

        std::string to_string(){
            std::stringstream ret;
            ret << "v0 " << "v2 " << "v3\n";
            ret << v[0][0] << " " << v[1][0] << " " << v[2][0] << "\n";
            ret << v[0][1] << " " << v[1][1] << " " << v[2][1] << "\n";
            ret << v[0][2] << " " << v[1][2] << " " << v[2][2] << "\n";
            return ret.str();
        }
};