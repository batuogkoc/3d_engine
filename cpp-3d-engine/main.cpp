#include <stdio.h>
#include <iostream>
#include "external/stl_reader/stl_reader.h"
#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"
#include <algorithm>
class Triangle{
    public:
        Eigen::Vector3f v[3];
        cv::Scalar color;
        Triangle(cv::Scalar color=cv::Scalar(255,255,255)){
            v[0] = Eigen::Vector3f::Zero();
            v[1] = Eigen::Vector3f::Zero();
            v[2] = Eigen::Vector3f::Zero();
            this->color = color; 
        } 
        Triangle(Eigen::Vector3f (&vertices)[3], cv::Scalar color=cv::Scalar(255,255,255)){
            v[0] = vertices[0];
            v[1] = vertices[1];
            v[2] = vertices[2];
            this->color = color; 
        }
        Triangle(std::vector<Eigen::Vector3f> vertices, cv::Scalar color=cv::Scalar(255,255,255)){
            v[0] = vertices[0];
            v[1] = vertices[1];
            v[2] = vertices[2];
            this->color = color; 
        }
        Triangle(Eigen::Vector3f vertex0, Eigen::Vector3f vertex1, Eigen::Vector3f vertex2, cv::Scalar color=cv::Scalar(255,255,255)){
            v[0] = vertex0;
            v[1] = vertex1;
            v[2] = vertex2;
            this->color = color;
        }
        Eigen::Vector3f edge_0(){
            return this->v[1] - this->v[0];
        }
        Eigen::Vector3f edge_1(){
            return this->v[2] - this->v[1];
        }
        Eigen::Vector3f edge_2(){
            return this->v[0] - this->v[2];
        }

        Eigen::Vector3f normal(){
            return this->edge_0().cross(this->edge_1());
        }

        Eigen::Vector3f center(){
            return (this->v[0] + this->v[1] + this->v[2])/3;
        }
        void translate(Eigen::Vector3f v){
            this->v[0] += v;
            this->v[1] += v;
            this->v[2] += v; 
        } 

        void transform(Eigen::Matrix3f t){
            v[0] = t*v[0];
            v[1] = t*v[1];
            v[2] = t*v[2];
        }
};

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

class Camera{
    public:
        const unsigned int width;
        const unsigned int height;
        const float vertical_fov;
        const float perspective_coords_scale_factor;
        Eigen::Vector3f position;
        Eigen::Matrix3f orientation;
        Camera(unsigned int width, unsigned int height, float vertical_fov, Eigen::Vector3f position = Eigen::Vector3f::Zero(), Eigen::Matrix3f orientation=Eigen::Matrix3f::Identity()):
            width(width),
            height(height),
            vertical_fov(vertical_fov),
            perspective_coords_scale_factor(((float)height)/2.0f/tanf(vertical_fov/2)),
            position(position),
            orientation(orientation)
        {}
};

void render_mesh(cv::Mat image, Mesh mesh, Camera camera){
    Mesh mesh_copy = mesh.copy();
    image.setTo(cv::Scalar::zeros());

    Eigen::Matrix3f camera_orientation_inverse = camera.orientation.transpose();
    
    for(auto triangle : mesh_copy.triangles){
        triangle.v[0] = camera_orientation_inverse*(triangle.v[0]-camera.position);
        triangle.v[1] = camera_orientation_inverse*(triangle.v[1]-camera.position);
        triangle.v[2] = camera_orientation_inverse*(triangle.v[2]-camera.position);
        // if(triangle.center().dot(triangle.normal())>=0){
        //     continue;
        // }
        triangle.v[0][0] /= triangle.v[0][2];
        triangle.v[0][1] /= triangle.v[0][2]; 

        triangle.v[1][0] /= triangle.v[1][2];
        triangle.v[1][1] /= triangle.v[1][2];

        triangle.v[2][0] /= triangle.v[2][2];
        triangle.v[2][1] /= triangle.v[2][2];

        float scale_factor = camera.perspective_coords_scale_factor;

        triangle.v[0][0] = triangle.v[0][0]*scale_factor + camera.width/2;
        triangle.v[0][1] = triangle.v[0][1]*scale_factor + camera.height/2;
        
        triangle.v[1][0] = triangle.v[1][0]*scale_factor + camera.width/2;
        triangle.v[1][1] = triangle.v[1][1]*scale_factor + camera.height/2;

        triangle.v[2][0] = triangle.v[2][0]*scale_factor + camera.width/2;
        triangle.v[2][1] = triangle.v[2][1]*scale_factor + camera.height/2;
        cv::Point points[] = {cv::Point(triangle.v[0][0], triangle.v[0][1]),
                                cv::Point(triangle.v[1][0], triangle.v[1][1]),
                                cv::Point(triangle.v[2][0], triangle.v[2][1])};
        cv::fillConvexPoly(image, points, 3, triangle.color);
        // std::vector<cv::Point> points = {cv::Point(triangle.v[0][0], triangle.v[0][1]),
        //                            cv::Point(triangle.v[1][0], triangle.v[1][1]),
        //                            cv::Point(triangle.v[2][0], triangle.v[2][1])};
        // cv::fillPoly(image, points, triangle.color);
    }
}

int main(int argc, char** argv){
    auto program_start = std::chrono::high_resolution_clock::now();
    Mesh mesh_original = Mesh(stl_reader::StlMesh<float, unsigned int>("../3d_files/Utah_teapot_(solid).stl"), true);
    // std::vector<Triangle> triangles = {Triangle({0,0,0}, {1,0,1}, {0,0,1})};
    // Mesh mesh = Mesh();
    // mesh.triangles = triangles;
    Camera camera(1600, 900, 0.785398163f*2.0f);

    cv::Mat image = cv::Mat::zeros(camera.height, camera.width, CV_8UC3);
    while(true){
        Mesh mesh = mesh_original.copy();
        image.setTo(cv::Scalar::zeros());
        
        double time_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-program_start).count() * 1e-9;

        Eigen::AngleAxisf rotation(time_sec, Eigen::Vector3f::UnitZ());
        mesh.transform(rotation.matrix());

        Eigen::Vector3f camera_position = {0,-16,4};
        Eigen::Matrix3f camera_orientation;
        camera_orientation << 1, 0, 0,
                              0, 0, 1,
                              0, -1, 0;
        camera.position = camera_position;
        camera.orientation = camera_orientation;
        render_mesh(image, mesh, camera);
        // Eigen::Matrix3f camera_orientation_inverse = camera_orientation.transpose();
        
        // for(auto triangle_original : mesh.triangles){
        //     Triangle triangle(triangle_original);

        //     triangle.v[0] = camera_orientation_inverse*(triangle.v[0]-camera_position);
        //     triangle.v[1] = camera_orientation_inverse*(triangle.v[1]-camera_position);
        //     triangle.v[2] = camera_orientation_inverse*(triangle.v[2]-camera_position);
        //     // if(triangle.center().dot(triangle.normal())>=0){
        //     //     continue;
        //     // }
        //     triangle.v[0][0] /= triangle.v[0][2];
        //     triangle.v[0][1] /= triangle.v[0][2]; 

        //     triangle.v[1][0] /= triangle.v[1][2];
        //     triangle.v[1][1] /= triangle.v[1][2];

        //     triangle.v[2][0] /= triangle.v[2][2];
        //     triangle.v[2][1] /= triangle.v[2][2];

        //     float scale_factor = ((float)height)/2.0f/tanf(camera_fov_vertical/2);

        //     triangle.v[0][0] = triangle.v[0][0]*scale_factor + width/2;
        //     triangle.v[0][1] = triangle.v[0][1]*scale_factor + height/2;
            
        //     triangle.v[1][0] = triangle.v[1][0]*scale_factor + width/2;
        //     triangle.v[1][1] = triangle.v[1][1]*scale_factor + height/2;

        //     triangle.v[2][0] = triangle.v[2][0]*scale_factor + width/2;
        //     triangle.v[2][1] = triangle.v[2][1]*scale_factor + height/2;
        //     cv::Point points[] = {cv::Point(triangle.v[0][0], triangle.v[0][1]),
        //                           cv::Point(triangle.v[1][0], triangle.v[1][1]),
        //                           cv::Point(triangle.v[2][0], triangle.v[2][1])};
        //     cv::fillConvexPoly(image, points, 3, triangle.color);
        //     // std::vector<cv::Point> points = {cv::Point(triangle.v[0][0], triangle.v[0][1]),
        //     //                            cv::Point(triangle.v[1][0], triangle.v[1][1]),
        //     //                            cv::Point(triangle.v[2][0], triangle.v[2][1])};
        //     // cv::fillPoly(image, points, triangle.color);
        // }
        cv::imshow("Aaa", image);
        int key = cv::pollKey();
        if(key == 'q'){
            break;
        }

        static auto start = std::chrono::high_resolution_clock::now();
        auto end = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-9;
        start = end;
        std::cout << "------------------\n";
        std::cout << "dt: " << dt << "\n";
        std::cout << "FPS: " << 1/dt << "\n";
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