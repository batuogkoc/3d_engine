#include <cmath>
#include "eigen3/Eigen/Dense"

class Camera{
    public:
        Eigen::Vector3f position;
        Eigen::Matrix3f orientation;
        Camera(unsigned int width, unsigned int height, float horizontal_fov){
            this->__height = height;
            this->__width = width;
            this->__optical_plane_distance = ((float)width/2)/sinf(((float)horizontal_fov)/2);
            this->position = Eigen::Vector3f::Zero();
            this->orientation = Eigen::Matrix3f::Identity();
        }
        add_to_scene()

    private:
        unsigned int __width;
        unsigned int __height;
        
        float __optical_plane_distance;
};