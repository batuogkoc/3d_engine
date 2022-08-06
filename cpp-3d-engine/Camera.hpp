#include <cmath>

class Camera{
    public:
        Camera(unsigned int width, unsigned int height, float horizontal_fov){
            this->__height = height;
            this->__width = width;
            this->__optical_plane_distance = ((float)width/2)/sinf(((float)horizontal_fov)/2);
        }

    private:
        unsigned int __width;
        unsigned int __height;
        float __optical_plane_distance;
};