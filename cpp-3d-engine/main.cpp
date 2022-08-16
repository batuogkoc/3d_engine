#include <stdio.h>
#include <iostream>
#include "external/stl_reader/stl_reader.h"
#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"
#include <algorithm>
#include "Camera.hpp"
#include "Mesh.hpp"
#include "Triangle.hpp"
#include "math.h"
#include "Plane.hpp"

// void draw_triangle(cv::Mat image, Triangle& triangle){
//     Triangle tri(triangle);
//     std::sort(tri.v, tri.v+3, [](Eigen::Vector3f v1, Eigen::Vector3f v2){
//         return v1[1]>v2[1];
//     });

//     for(size_t i=(size_t)tri.v[0][1]; i<(size_t)tri.v[1][1]; i++){
//         for()
//     }
// }

float line_plane_intersection_t(Eigen::Vector3f line_v, Eigen::Vector3f line_p0, Eigen::Vector3f plane_n, Eigen::Vector3f plane_p0){
    float epsilon = 1e-6;
    float normal_vector_dot = line_v.dot(plane_n);
    float normal_p0_dif_dot = plane_n.dot(plane_p0-line_p0);
    if(std::abs(normal_vector_dot)<epsilon && std::abs(normal_p0_dif_dot)<epsilon){
        //all values of t satisfy the equation
        return std::numeric_limits<float>::quiet_NaN();
    }
    else if(std::abs(normal_vector_dot)<epsilon && std::abs(normal_p0_dif_dot)>epsilon){
        //the line and plane doesnt intersect
        return std::numeric_limits<float>::infinity();
    }
    else{
        return normal_p0_dif_dot/normal_vector_dot;
    }
}

std::vector<Triangle> clip_triangle(Triangle const &triangle, Plane const &plane){   
    float epsilon=1e-6;
    std::vector<Triangle> ret;
    int in_points = 0;
    bool in_points_idx[3];
    for(size_t i=0; i<3; i++){
        if(plane.normal.dot(triangle.v[i]-plane.point)>-epsilon){
            in_points++;
            in_points_idx[i] = true;
        }
        else{
            in_points_idx[i] = false;
        }
    }

    if(in_points==0){
        return ret;
    }
    else if(in_points == 3){
        ret.push_back(triangle);
        return ret;
    }

    float edge_intersection_ts[3];
    for(int i=0; i<3; i++){
        edge_intersection_ts[i] = line_plane_intersection_t(triangle.v[(i+1)%3]-triangle.v[i], triangle.v[i], plane.normal, plane.point);
        // printf("Edge %d t: %f in? %s\n", i, edge_intersection_ts[i], in_points_idx[i]?"true":"false");
    }
    
    if(in_points == 1){
        for(int i=0; i<3; i++){
            if(in_points_idx[i] == true){
                // ret.push_back(Triangle(triangle.v[i], triangle.v[i] + (triangle.v[(i+1)%3]-triangle.v[i])*edge_intersection_ts[i], triangle.v[(i+2)%3] + (triangle.v[(i+3)%3]-triangle.v[(i+2)%3])*edge_intersection_ts[(i+2)%3], triangle.color));
                ret.push_back(Triangle(triangle.v[i], triangle.v[i] + (triangle.v[(i+1)%3]-triangle.v[i])*edge_intersection_ts[i], triangle.v[(i+2)%3] + (triangle.v[(i+3)%3]-triangle.v[(i+2)%3])*edge_intersection_ts[(i+2)%3], cv::Scalar(0,255,0)));
                return ret;
            }
        }
    }
    else if(in_points==2){
        for(int i=0; i<3; i++){
            if(edge_intersection_ts[i] >= 0 && edge_intersection_ts[i] <= 1 && in_points_idx[i]){
                int i_minus_1 = (i-1)%3>=0?(i-1)%3:3+(i-1)%3;
                Eigen::Vector3f intersection_point_0 = (triangle.v[(i+1)%3]-triangle.v[i])*edge_intersection_ts[i] + triangle.v[i];
                Eigen::Vector3f intersection_point_1 = (triangle.v[(i+2)%3]-triangle.v[(i+1)%3])*edge_intersection_ts[(i+1)%3] + triangle.v[(i+1)%3];
                // ret.push_back(Triangle(triangle.v[i], intersection_point_0, triangle.v[i_minus_1], triangle.color));
                // ret.push_back(Triangle(triangle.v[i_minus_1], intersection_point_0, intersection_point_1, triangle.color));
                ret.push_back(Triangle(triangle.v[i], intersection_point_0, triangle.v[i_minus_1], cv::Scalar(255,0,0)));
                ret.push_back(Triangle(triangle.v[i_minus_1], intersection_point_0, intersection_point_1, cv::Scalar(0,0,255)));
                return ret;
            }
        }
    }
}

std::deque<Triangle> clip_triangle_on_planes(Triangle const &triangle, std::vector<Plane> const &planes){
    std::deque<Triangle> ret;
    ret.push_back(triangle);
    for(auto const &plane:planes){
        size_t num_to_operate_on = ret.size();
        for(size_t i=0; i<num_to_operate_on; i++){
            auto result = clip_triangle(ret[0], plane);
            ret.pop_front();
            ret.insert(ret.end(), result.begin(), result.end());
        }
    }
    return ret;
}

void render_mesh(cv::Mat& image, Mesh& mesh, Camera& camera, Eigen::Vector3f light_direction){
    light_direction.normalize();
    Mesh mesh_copy = mesh.copy();
    image.setTo(cv::Scalar::zeros());

    Eigen::Matrix3f camera_orientation_inverse = camera.orientation.transpose();
    
    Mesh to_draw = Mesh();
    for(int i=0; i<mesh_copy.triangles.size(); i++){
        Triangle &triangle = mesh_copy.triangles[i];
        if((triangle.center()-camera.position).dot(triangle.normal())>=0){
            continue;
        }

        float brightness = std::max(0.0f, -triangle.normal().dot(light_direction)/triangle.normal().norm());
        triangle.color *= brightness;

        
        triangle.v[0] = camera_orientation_inverse*(triangle.v[0]-camera.position);
        triangle.v[1] = camera_orientation_inverse*(triangle.v[1]-camera.position);
        triangle.v[2] = camera_orientation_inverse*(triangle.v[2]-camera.position);

        std::vector<Plane> planes;
        float near_clipping = 1;
        float far_clipping = 1000;
        planes.push_back(Plane({0,0,1}, {0,0,near_clipping}));
        planes.push_back(Plane({0,0,-1}, {0,0,far_clipping}));

        // planes.push_back(Plane({1,0,0}, {1,0,0}));
        // planes.push_back(Plane({-1,0,0}, {camera.width-1,0,0}));
        // planes.push_back(Plane({0,1,0}, {0,1,0}));
        // planes.push_back(Plane({0,-1,0}, {0,camera.height-1,0}));

        planes.push_back(Plane({0, tan(M_PI_2-camera.vertical_fov/2),1}, {0,0,0}));
        planes.push_back(Plane({0,-tan(M_PI_2-camera.vertical_fov/2),1}, {0,0,0}));
        planes.push_back(Plane({ tanf(M_PI_2-camera.horizontal_fov/2),0,1}, {0,0,0}));
        planes.push_back(Plane({-tanf(M_PI_2-camera.horizontal_fov/2),0,1}, {0,0,0}));

        auto result = clip_triangle_on_planes(triangle, planes);
        to_draw.triangles.insert(to_draw.triangles.end(), result.begin(), result.end());
        // to_draw.triangles.push_back(triangle);
    }

    std::sort(to_draw.triangles.begin(), to_draw.triangles.end(), [](Triangle t1, Triangle t2){
        return t1.center()[2]>t2.center()[2];
    });

    size_t i=0;
    for(auto triangle:to_draw.triangles){
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
        // std::cout<<i++<<":\n";
        // std::cout<<triangle.v[0][0]<<" "<<triangle.v[0][1]<<" "<<triangle.v[0][2]<<" "<<"\n";
        // std::cout<<triangle.v[1][0]<<" "<<triangle.v[1][1]<<" "<<triangle.v[1][2]<<" "<<"\n";
        // std::cout<<triangle.v[2][0]<<" "<<triangle.v[2][1]<<" "<<triangle.v[2][2]<<" "<<"\n";
        cv::fillConvexPoly(image, points, 3, triangle.color);
    }
}

int main(int argc, char** argv){
    auto program_start = std::chrono::high_resolution_clock::now();
    // Mesh mesh_original = Mesh();
    // mesh_original.triangles.push_back(Triangle({0,0,0},{1,0,1},{0,0,1}));
    // mesh_original.triangles.push_back(Triangle({0,0,0},{1,0,0},{1,0,1}));

    // Mesh mesh_original = Mesh(stl_reader::StlMesh<float, unsigned int>("../3d_files/Utah_teapot_(solid).stl"), false);
    Mesh mesh_original = Mesh(stl_reader::StlMesh<float, unsigned int>("../3d_files/cat.stl"), false);
    // std::vector<Triangle> triangles = {Triangle({0,0,0}, {1,0,1}, {0,0,1})};
    // Mesh mesh = Mesh();
    // mesh.triangles = triangles;
    Camera camera(1600, 900, 0.785398163f*2.0f);

    cv::Mat image = cv::Mat::zeros(camera.height, camera.width, CV_8UC3);
    camera.position = {0,-75,15};
    camera.orientation << 1, 0, 0,
                        0, 0, 1,
                        0, -1, 0;
    while(true){
        Mesh mesh = mesh_original.copy();
        image.setTo(cv::Scalar::zeros());
        
        double time_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-program_start).count() * 1e-9;

        Eigen::AngleAxisf rotation(time_sec, Eigen::Vector3f::UnitZ());
        mesh.transform(rotation.matrix());

        render_mesh(image, mesh, camera, {0,1,-1});
        cv::imshow("Aaa", image);
        int key = cv::pollKey();

        static auto start = std::chrono::high_resolution_clock::now();
        auto end = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-9;
        start = end;

        if(key == 'q'){
            break;
        }
        float speed = 15.0f;
        if(key == 'w'){
            camera.position += Eigen::Vector3f::UnitY()*speed*dt;
        }
        else if (key == 's'){
            camera.position -= Eigen::Vector3f::UnitY()*speed*dt;
        }
        else if (key == 'a'){
            camera.position -= Eigen::Vector3f::UnitX()*speed*dt;
        }
        else if (key == 'd'){
            camera.position += Eigen::Vector3f::UnitX()*speed*dt;
        }

                
        std::cout << "------------------\n";
        std::cout << "dt: " << dt << "\n";
        std::cout << "FPS: " << 1/dt << "\n";
        // auto t = Triangle({0,0,0},{0,0,3},{3,0,3});
        // std::vector<Plane> planes;
        // planes.push_back(Plane({0,0,1}, {0,0,1}));
        // planes.push_back(Plane({0,0,-1}, {0,0,2}));

        // auto result = clip_triangle_on_planes(t, planes);
        // for(int i=0; i<result.size(); i++){
        //     std::cout<< "Triangle number "<< i <<":\n";
        //     for(int j=0; j<3; j++){
        //         std::cout<<"vertex " <<j<< ":\n";
        //         std::cout<<result[i].v[j]<<"\n";
        //     }
        // }
    }
    cv::destroyAllWindows();
    return 0;
}