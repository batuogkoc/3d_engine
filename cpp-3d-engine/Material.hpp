#pragma once
// #include "Texture.hpp"
#include "opencv2/opencv.hpp"
#include <filesystem>
#include <fstream>

class Material{
    public:
        //ambient color
        cv::Mat map_Ka;
        // std::array<float, 3> Ka;
        
        //diffuse color
        cv::Mat map_Kd;
        // std::array<float, 3> Kd;
        
        //specular color
        cv::Mat map_Ks;
        // std::array<float, 3> Ks;

        // //specular highlight
        // cv::Mat map_Ns;
        // float Ns;

        // //alpha
        // cv::Mat map_d; 
        // float d;
        
        // //bump map
        // cv::Mat map_bump;

        // //displacement map
        // cv::Mat map_disp;

        // float Ni;  

        static std::map<std::string, Material> fromMTL(std::filesystem::path mtl_path){
            std::map<std::string, Material> ret;
            std::fstream file;
            file.open(mtl_path);
            if(!file.is_open()){
                return ret;
            }
            std::string line;
            while(std::getline(file, line)){
                char *ptr
            }
        }
};