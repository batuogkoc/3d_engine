#include <vector>
#include "external/stl_reader/stl_reader.h"
#include "Triangle.hpp"
#include "Texture.hpp"
#include "eigen3/Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "string"
#include <fstream>

class Mesh{
    public:
        std::vector<Triangle> triangles;
        Texture texture;
        Mesh(){};
        Mesh(stl_reader::StlMesh<float, unsigned int> mesh):
        texture()
        {
            for(size_t itri=0; itri<mesh.num_tris(); itri++){
                const float* c0 = mesh.tri_corner_coords(itri, 0);
                const float* c1 = mesh.tri_corner_coords(itri, 1);
                const float* c2 = mesh.tri_corner_coords(itri, 2);
                Eigen::Vector3f corner0 = {c0[0], c0[1], c0[2]};
                Eigen::Vector3f corner1 = {c1[0], c1[1], c1[2]};
                Eigen::Vector3f corner2 = {c2[0], c2[1], c2[2]};
                triangles.push_back(Triangle(corner0, corner1, corner2, Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero()));
            }
        }
        Mesh(std::string obj_path){
            std::fstream file;
            file.open(obj_path);
            if(!file.is_open()){
                throw std::runtime_error("OBJ file could not be opened");
            }
            std::vector<Eigen::Vector3f> vertices;
            std::vector<Eigen::Vector2f> textures;
            std::string line;
            while(std::getline(file, line)){
                if(line[0] == '#')
                    continue;
                auto line_beginning = line.substr(0, line.find(' '));

                if(line_beginning == "v"){
                    float x,y,z;
                    if(sscanf(line.c_str(), "v %f %f %f", &x, &y, &z) != 3){
                        throw std::runtime_error("Invalid format");
                    }
                    Eigen::Vector3f vertex(x, y, z);
                    vertices.push_back(vertex);
                }

                if(line_beginning == "vt"){
                    float u,v;
                    if(sscanf(line.c_str(), "vt %f %f", &u, &v) != 2){
                        throw std::runtime_error("Invalid format");
                    }
                    Eigen::Vector2f texture(u, v);
                    textures.push_back(texture);
                }

                if(line_beginning == "f"){
                    int slash_count = std::count(line.begin(), line.end(), '/');
                    size_t v0,v1,v2;
                    size_t tex0,tex1,tex2;
                    size_t n0, n1, n2;
                    switch (slash_count)
                    {
                    case 0:
                        if(sscanf(line.c_str(), "f %u %u %u", &v0, &v1, &v2) != 3){
                            throw std::runtime_error("Invalid format");
                        }
                        this->triangles.push_back(Triangle(vertices[v0], vertices[v1], vertices[v2], Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero()));
                        break;
                    
                    case 3:
                        if(sscanf(line.c_str(), "f %u/%u %u/%u %u/%u", &v0, &tex0, &v1, &tex1, &v2, &tex2) != 6){
                            throw std::runtime_error("Invalid format");
                        }
                        this->triangles.push_back(Triangle(vertices[v0], vertices[v1], vertices[v2], textures[tex0], textures[tex1], textures[tex2]));
                        break;

                    case 6:
                        if(sscanf(line.c_str(), "f %u/%u/%u %u/%u/%u %u/%u/%u", &v0, &tex0, &n0, &v1, &tex1, &n1, &v2, &tex2, &n2) != 9){
                            if(sscanf(line.c_str(), "f %u//%u %u//%u %u//%u", &v0, &tex0, &v1, &tex1, &v2, &tex2) != 6){
                                throw std::runtime_error("Invalid format");
                            }
                        }
                        this->triangles.push_back(Triangle(vertices[v0], vertices[v1], vertices[v2], textures[tex0], textures[tex1], textures[tex2]));
                        break;

                    default:
                        throw std::runtime_error("Invalid format");
                        break;
                    }
                }

                line.clear();
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


