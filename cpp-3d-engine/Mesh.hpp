#include <vector>
#include "external/stl_reader/stl_reader.h"
#include "Triangle.hpp"
#include "Texture.hpp"
#include "eigen3/Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "string"
#include <fstream>
#include <filesystem>

class Mesh{
    public:
        std::vector<Triangle> triangles;
        Texture texture;
        Mesh():
        texture(),
        triangles()
        {};
        static std::optional<Mesh> fromSTL(std::filesystem::path stl_path){
            Mesh ret;
            try{
                stl_reader::StlMesh<float, unsigned int> mesh(stl_path);
                for(size_t itri=0; itri<mesh.num_tris(); itri++){
                    const float* c0 = mesh.tri_corner_coords(itri, 0);
                    const float* c1 = mesh.tri_corner_coords(itri, 1);
                    const float* c2 = mesh.tri_corner_coords(itri, 2);
                    Eigen::Vector3f corner0 = {c0[0], c0[1], c0[2]};
                    Eigen::Vector3f corner1 = {c1[0], c1[1], c1[2]};
                    Eigen::Vector3f corner2 = {c2[0], c2[1], c2[2]};
                    ret.triangles.push_back(Triangle(corner0, corner1, corner2, Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero()));
                }
                return ret;
            }
            catch(std::exception& e){
                return {};
            }
        }

        static std::optional<Mesh> fromOBJ(std::filesystem::path obj_path){
            Mesh ret;
            std::fstream file;
            file.open(obj_path);
            if(!file.is_open()){
                return {};
            }
            std::vector<Eigen::Vector3f> vertices;
            std::vector<Eigen::Vector2f> textures;
            std::string line;
            bool has_texture=false;
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
                    std::stringstream ss(line);
                    std::vector<size_t> values[3];
                    ss.get();
                    ss.get();
                    size_t current;
                    int idx = 0;
                    while(ss>>current){
                        char next;
                        ss.get(next);
                        values[idx].push_back(current);
                        if(next == '/'){
                            idx++;
                            if(ss.peek() == '/'){
                                idx++;
                            }
                                
                        }
                        else if(next == ' '){
                            idx = 0;
                        }
                        else{
                            break;
                        }
                    }
                    for(int i=1; i<values[0].size()-1; i++){
                        if(values[0].size() == values[1].size()){
                            has_texture = true;
                            ret.triangles.push_back(Triangle(vertices[values[0][0]-1], vertices[values[0][i]-1], vertices[values[0][i+1]-1], textures[values[1][0]-1], textures[values[1][i]-1], textures[values[1][i+1]-1]));
                        }
                        else{
                            ret.triangles.push_back(Triangle(vertices[values[0][0]-1], vertices[values[0][i]-1], vertices[values[0][i+1]-1], Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero()));
                        }
                    }
                }
                line.clear();
            }
            if(has_texture){
                ret.texture = Texture();
            }
            else{
                ret.texture = Texture();
            }

            return ret;
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
            ret.texture = texture.copy();
            return ret;
        }
};


