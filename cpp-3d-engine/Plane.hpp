#pragma once 
#include "eigen3/Eigen/Dense"

class Plane{
    public:
        Eigen::Vector3f normal;
        Eigen::Vector3f point;
        Plane(Eigen::Vector3f normal, Eigen::Vector3f point):
        normal(normal),
        point(point)
        {}
};