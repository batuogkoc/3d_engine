#pragma once
#include "Texture.hpp"
#include "opencv2/opencv.hpp"

class Material{
    public:
        Texture map_Ka;
        std::array<float, 3> Ka;
        std::array<float, 3> Kd;
        std::array<float, 3> Ks;
        float Ns;      
        float d;
        std::array<float, 3> Tf;
        float Ni;  
};