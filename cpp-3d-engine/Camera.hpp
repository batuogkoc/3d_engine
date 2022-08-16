#pragma once
#include "eigen3/Eigen/Dense"

class Camera{
    public:
        const unsigned int width;
        const unsigned int height;
        const float vertical_fov;
        const float horizontal_fov;
        const float perspective_coords_scale_factor;
        
        Eigen::Vector3f position;
        Eigen::Matrix3f orientation;
        Camera(unsigned int width, unsigned int height, float vertical_fov, Eigen::Vector3f position = Eigen::Vector3f::Zero(), Eigen::Matrix3f orientation=Eigen::Matrix3f::Identity()):
            width(width),
            height(height),
            vertical_fov(vertical_fov),
            horizontal_fov(2*atanf(((float)width)/((float)height)*tan(vertical_fov/2))),
            perspective_coords_scale_factor(((float)height)/2.0f/tanf(vertical_fov/2)),
            position(position),
            orientation(orientation)
        {}
};