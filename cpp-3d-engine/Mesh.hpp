#include <vector>
#include "external/stl_reader/stl_reader.h"
#include "Triangle.hpp"
#include "eigen3/Eigen/Dense"
#include "opencv2/opencv.hpp"

class Mesh{
    public:
        std::vector<Triangle> triangles;
        Mesh(){};
        Mesh(stl_reader::StlMesh<float, unsigned int> mesh, bool random_color = false){
            for(size_t itri=0; itri<mesh.num_tris(); itri++){
                const float* c0 = mesh.tri_corner_coords(itri, 0);
                const float* c1 = mesh.tri_corner_coords(itri, 1);
                const float* c2 = mesh.tri_corner_coords(itri, 2);
                Eigen::Vector3f corner0 = {c0[0], c0[1], c0[2]};
                Eigen::Vector3f corner1 = {c1[0], c1[1], c1[2]};
                Eigen::Vector3f corner2 = {c2[0], c2[1], c2[2]};
                triangles.push_back(Triangle(corner0, corner1, corner2, random_color?cv::Scalar::randu(0,255):cv::Scalar::all(255)));
            }
        }
        void translate(Eigen::Vector3f v){
            for(size_t i = 0; i<this->triangles.size(); i++){
                this->triangles[i].translate(v);
            }
        }

        void transform(Eigen::Matrix3f t){
            for(size_t i = 0; i<this->triangles.size(); i++){
                this->triangles[i].transform(t);
            }
        }

        Mesh copy(){
            Mesh ret;
            for(auto triangle : this->triangles){
                ret.triangles.push_back(Triangle(triangle));
            }       
            return ret;
        }
};


