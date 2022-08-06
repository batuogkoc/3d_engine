#include <stdio.h>
#include <iostream>
#include "stl_reader/stl_reader.h"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv){
    std::vector<int> sizes = {300,300,3};
    cv::Mat image = cv::Mat::zeros(200, 200, CV_8UC1);
    cv::imshow("Aaa", image);
    while(true){
        static auto start = std::chrono::high_resolution_clock::now();
        auto end = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-9;
        start = end;

        int key = cv::waitKey(100);
        if(key == 'q'){
            break;
        }
        std::cout << "dt: " << dt << "\n";
    }
    cv::destroyAllWindows();
    return 0;
    // try {
    //     stl_reader::StlMesh <float, unsigned int> mesh ("../3d_files/Utah_teapot_(solid).stl");

    //     for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
    //         std::cout << "coordinates of triangle " << itri << ": ";
    //         for(size_t icorner = 0; icorner < 3; ++icorner) {
    //             // float* c = mesh.vrt_coords (mesh.tri_corner_ind (itri, icorner));
    //             // const float* c = mesh.tri_corner_coords (itri, icorner);
    //             // or alternatively:
    //             const float* c = mesh.vrt_coords (mesh.tri_corner_ind (itri, icorner));
    //             std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
    //         }
    //         std::cout << std::endl;
        
    //         const float* n = mesh.tri_normal (itri);
    //         std::cout   << "normal of triangle " << itri << ": "
    //                     << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
    //     }
    //     }
    //     catch (std::exception& e) {
    //     std::cout << e.what() << std::endl;
    // }
}