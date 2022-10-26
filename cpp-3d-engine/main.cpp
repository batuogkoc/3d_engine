#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
#include "external/stl_reader/stl_reader.h"
#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"
#include <algorithm>
#include "Camera.hpp"
#include "Mesh.hpp"
#include "Triangle.hpp"
#include "math.h"
#include "Plane.hpp"


void draw_triangle(cv::Mat &image, Texture &texture, Triangle triangle){
    Eigen::Matrix2f barycentric_t_matrix = Eigen::Matrix2f();
    barycentric_t_matrix << triangle.v[0][0] - triangle.v[2][0], triangle.v[1][0] - triangle.v[2][0],
                            triangle.v[0][1] - triangle.v[2][1], triangle.v[1][1] - triangle.v[2][1];
    Eigen::Matrix2f barycentric_t_matrix_inv = barycentric_t_matrix.inverse();
    Eigen::Vector2f vertex_2 = Eigen::Vector2f();
    vertex_2 << triangle.v[2][0], triangle.v[2][1];
    float z0 = triangle.v[0][2];
    float z1 = triangle.v[1][2];
    float z2 = triangle.v[2][2];
    


    std::sort(triangle.v, triangle.v+3, [](Eigen::Vector3f v1, Eigen::Vector3f v2){
        return v1[1]<v2[1];
    });
    Eigen::Vector3f edge_l = triangle.v[1]-triangle.v[0];
    Eigen::Vector3f edge_r = triangle.v[2]-triangle.v[0];
    

    edge_l /= edge_l[1];
    edge_r /= edge_r[1];

    if(edge_l[0]>edge_r[0]){
        auto temp = edge_r;
        edge_r = edge_l;
        edge_l = temp;
    }


    for(size_t i=std::max((size_t)0, (size_t)triangle.v[0][1]+1); i<=std::min((size_t)image.rows-1, (size_t)triangle.v[1][1]); i++){
        for(size_t j=std::max((size_t)0, (size_t)(triangle.v[0][0]+edge_l[0]*((float)i-triangle.v[0][1]))+1);j<=std::min((size_t)image.cols-1,(size_t)(triangle.v[0][0]+edge_r[0]*((float)i-triangle.v[0][1]))); j++){
            Eigen::Vector2f r = Eigen::Vector2f();
            r << (float)j,(float)i;
            Eigen::Vector2f v0_v1_barycentric = barycentric_t_matrix_inv*(r-vertex_2);
            float v0_w = v0_v1_barycentric[0];
            float v1_w = v0_v1_barycentric[1];
            float v2_w = 1-v0_w-v1_w;

            image.at<cv::Vec3b>(i,j) = texture.get_pixel(triangle.tex[0]*v0_w + triangle.tex[1]*v1_w + triangle.tex[2]*v2_w);
            // image.at<cv::Vec3b>(i,j)[0] = (uint8_t)255*v0_w;
            // image.at<cv::Vec3b>(i,j)[1] = (uint8_t)255*v1_w;
            // image.at<cv::Vec3b>(i,j)[2] = (uint8_t)255*v2_w;
        }
    }

    edge_l = triangle.v[0]-triangle.v[2];
    edge_r = triangle.v[1]-triangle.v[2];
    

    edge_l /= -edge_l[1];
    edge_r /= -edge_r[1];

    if(edge_l[0]>edge_r[0]){
        auto temp = edge_r;
        edge_r = edge_l;
        edge_l = temp;
    }

    for(size_t i=std::min((size_t)image.rows-1,(size_t)triangle.v[2][1]); i>std::max((size_t)0, (size_t)triangle.v[1][1]); i--){
        for(size_t j=std::max((size_t)0,(size_t)(triangle.v[2][0]+edge_l[0]*(triangle.v[2][1]-(float)i))+1);j<=std::min((size_t)image.cols-1,(size_t)(triangle.v[2][0]+edge_r[0]*(triangle.v[2][1]-(float)i))); j++){
            Eigen::Vector2f r = Eigen::Vector2f();
            r << (float)j,(float)i;
            Eigen::Vector2f v0_v1_barycentric = barycentric_t_matrix_inv*(r-vertex_2);
            float v0_w = v0_v1_barycentric[0];
            float v1_w = v0_v1_barycentric[1];
            float v2_w = 1-v0_w-v1_w;

            image.at<cv::Vec3b>(i,j) = texture.get_pixel(triangle.tex[0]*v0_w + triangle.tex[1]*v1_w + triangle.tex[2]*v2_w);
            // image.at<cv::Vec3b>(i,j)[0] = (uint8_t)255*v0_w;
            // image.at<cv::Vec3b>(i,j)[1] = (uint8_t)255*v1_w;
            // image.at<cv::Vec3b>(i,j)[2] = (uint8_t)255*v2_w;
        }
    }
}

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
    }
    
    if(in_points == 1){
        for(int i=0; i<3; i++){
            if(in_points_idx[i] == true){
                // ret.push_back(Triangle(triangle.v[i], triangle.v[i] + (triangle.v[(i+1)%3]-triangle.v[i])*edge_intersection_ts[i], triangle.v[(i+2)%3] + (triangle.v[(i+3)%3]-triangle.v[(i+2)%3])*edge_intersection_ts[(i+2)%3], triangle.color));
                float current_edge_t = edge_intersection_ts[i];
                float prev_edge_t = edge_intersection_ts[(i+2)%3];
                ret.push_back(Triangle(triangle.v[i], triangle.v[i] + (triangle.v[(i+1)%3]-triangle.v[i])*current_edge_t, triangle.v[(i+2)%3] + (triangle.v[(i+3)%3]-triangle.v[(i+2)%3])*prev_edge_t, triangle.tex[i], triangle.tex[i]*(1-current_edge_t) + triangle.tex[(i+1)%3]*(current_edge_t), triangle.tex[(i+2)%3]*(1-prev_edge_t) + triangle.tex[i]*prev_edge_t));
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

                Eigen::Vector2f intersection_point_0_tex = triangle.tex[i]*(1-edge_intersection_ts[i]) + triangle.tex[(i+1)%3]*(edge_intersection_ts[i]);
                Eigen::Vector2f intersection_point_1_tex = triangle.tex[(i+1)%3]*(1-edge_intersection_ts[(i+1)%3]) + triangle.tex[(i+2)%3]*(edge_intersection_ts[(i+1)%3]);
                // ret.push_back(Triangle(triangle.v[i], intersection_point_0, triangle.v[i_minus_1], triangle.color));
                // ret.push_back(Triangle(triangle.v[i_minus_1], intersection_point_0, intersection_point_1, triangle.color));
                ret.push_back(Triangle(triangle.v[i], intersection_point_0, triangle.v[i_minus_1], triangle.tex[i], intersection_point_0_tex, triangle.tex[i_minus_1]));
                ret.push_back(Triangle(triangle.v[i_minus_1], intersection_point_0, intersection_point_1, triangle.tex[i_minus_1], intersection_point_0_tex, intersection_point_1_tex));
                return ret;
            }
        }
    }
    return ret;
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
    std::vector<Plane> planes;
    float near_clipping = 0.1;
    float far_clipping = 1000;
    planes.push_back(Plane({0,0,1}, {0,0,near_clipping}));
    planes.push_back(Plane({0,0,-1}, {0,0,far_clipping}));

    planes.push_back(Plane({0, tanf(M_PI_2-camera.vertical_fov/2),1}, {0,0,0}));
    planes.push_back(Plane({0,-tanf(M_PI_2-camera.vertical_fov/2),1}, {0,0,0}));
    planes.push_back(Plane({ tanf(M_PI_2-camera.horizontal_fov/2),0,1}, {0,0,0}));
    planes.push_back(Plane({-tanf(M_PI_2-camera.horizontal_fov/2),0,1}, {0,0,0}));
    for(int i=0; i<mesh_copy.triangles.size(); i++){
        Triangle &triangle = mesh_copy.triangles[i];
        if((triangle.center()-camera.position).dot(triangle.normal())>=0){
            continue;
        }

        float brightness = std::max(0.0f, -triangle.normal().dot(light_direction)/triangle.normal().norm());
        // triangle.color *= brightness;

        
        triangle.v[0] = camera_orientation_inverse*(triangle.v[0]-camera.position);
        triangle.v[1] = camera_orientation_inverse*(triangle.v[1]-camera.position);
        triangle.v[2] = camera_orientation_inverse*(triangle.v[2]-camera.position);

        auto result = clip_triangle_on_planes(triangle, planes);
        to_draw.triangles.insert(to_draw.triangles.end(), result.begin(), result.end());
        // to_draw.triangles.push_back(triangle);
    }

    std::sort(to_draw.triangles.begin(), to_draw.triangles.end(), [](Triangle t1, Triangle t2){
        return t1.center()[2]>t2.center()[2];
    });

    size_t i=0;
    for(auto triangle:to_draw.triangles){
        //convert to projective coords
        triangle.v[0][0] /= triangle.v[0][2];
        triangle.v[0][1] /= triangle.v[0][2]; 

        triangle.v[1][0] /= triangle.v[1][2];
        triangle.v[1][1] /= triangle.v[1][2];

        triangle.v[2][0] /= triangle.v[2][2];
        triangle.v[2][1] /= triangle.v[2][2];

        //convert to frame coords
        float scale_factor = camera.perspective_coords_scale_factor;

        triangle.v[0][0] = triangle.v[0][0]*scale_factor + camera.width/2;
        triangle.v[0][1] = triangle.v[0][1]*scale_factor + camera.height/2;
        
        triangle.v[1][0] = triangle.v[1][0]*scale_factor + camera.width/2;
        triangle.v[1][1] = triangle.v[1][1]*scale_factor + camera.height/2;

        triangle.v[2][0] = triangle.v[2][0]*scale_factor + camera.width/2;
        triangle.v[2][1] = triangle.v[2][1]*scale_factor + camera.height/2;

        //draw pixels
        Eigen::Matrix2f barycentric_t_matrix = Eigen::Matrix2f();
        barycentric_t_matrix << triangle.v[0][0] - triangle.v[2][0], triangle.v[1][0] - triangle.v[2][0],
                                triangle.v[0][1] - triangle.v[2][1], triangle.v[1][1] - triangle.v[2][1];
        Eigen::Matrix2f barycentric_t_matrix_inv = barycentric_t_matrix.inverse();
        Eigen::Vector2f vertex_2 = Eigen::Vector2f();
        vertex_2 << triangle.v[2][0], triangle.v[2][1];
        float z0 = triangle.v[0][2];
        float z1 = triangle.v[1][2];
        float z2 = triangle.v[2][2];

        std::sort(triangle.v, triangle.v+3, [](Eigen::Vector3f v1, Eigen::Vector3f v2){
            return v1[1]<v2[1];
        });
        Eigen::Vector3f edge_l = triangle.v[1]-triangle.v[0];
        Eigen::Vector3f edge_r = triangle.v[2]-triangle.v[0];
        

        edge_l /= edge_l[1];
        edge_r /= edge_r[1];

        if(edge_l[0]>edge_r[0]){
            auto temp = edge_r;
            edge_r = edge_l;
            edge_l = temp;
        }


        for(size_t i=std::max((size_t)0, (size_t)triangle.v[0][1]+1); i<=std::min((size_t)image.rows-1, (size_t)triangle.v[1][1]); i++){
            for(size_t j=std::max((size_t)0, (size_t)(triangle.v[0][0]+edge_l[0]*((float)i-triangle.v[0][1]))+1);j<=std::min((size_t)image.cols-1,(size_t)(triangle.v[0][0]+edge_r[0]*((float)i-triangle.v[0][1]))); j++){
                Eigen::Vector2f r = Eigen::Vector2f();
                r << (float)j,(float)i;
                Eigen::Vector2f v0_v1_barycentric = barycentric_t_matrix_inv*(r-vertex_2);
                float v0_w = v0_v1_barycentric[0];
                float v1_w = v0_v1_barycentric[1];
                float v2_w = 1-v0_w-v1_w;
                
                // Eigen::Vector2f affine_mapped_coords = (triangle.tex[0]*v0_w + triangle.tex[1]*v1_w + triangle.tex[2]*v2_w);
                
                // triangle.tex[0] *=vertex_0_z_inv;
                // triangle.tex[1] *=vertex_1_z_inv;
                // triangle.tex[2] *=vertex_2_z_inv;

                float current_pixel_z_inv = 1/z0*v0_w + 1/z1*v1_w + 1/z2*v2_w;

                Eigen::Vector2f perspective_corrected_coords = (triangle.tex[0]*1/z0*v0_w + triangle.tex[1]*1/z1*v1_w + triangle.tex[2]*1/z2*v2_w)/current_pixel_z_inv;
                
                image.at<cv::Vec3b>(i,j) = mesh.texture.get_pixel(perspective_corrected_coords);
                // cv::Vec3b vertex_0_color = texture.at<cv::Vec3b>(triangle.tex[0][1], triangle.tex[0][0]);
                // cv::Vec3b vertex_1_color = texture.at<cv::Vec3b>(triangle.tex[1][1], triangle.tex[1][0]);
                // cv::Vec3b vertex_2_color = texture.at<cv::Vec3b>(triangle.tex[2][1], triangle.tex[2][0]);
                // image.at<cv::Vec3b>(i,j) = vertex_0_color*v0_w + vertex_1_color*v1_w + vertex_2_color*v2_w;
                // image.at<cv::Vec3b>(i,j)[0] = (uint8_t)255*v0_w;
                // image.at<cv::Vec3b>(i,j)[1] = (uint8_t)255*v1_w;
                // image.at<cv::Vec3b>(i,j)[2] = (uint8_t)255*v2_w;
            }
        }

        edge_l = triangle.v[0]-triangle.v[2];
        edge_r = triangle.v[1]-triangle.v[2];
        

        edge_l /= -edge_l[1];
        edge_r /= -edge_r[1];

        if(edge_l[0]>edge_r[0]){
            auto temp = edge_r;
            edge_r = edge_l;
            edge_l = temp;
        }

        for(size_t i=std::min((size_t)image.rows-1,(size_t)triangle.v[2][1]); i>std::max((size_t)0, (size_t)triangle.v[1][1]); i--){
            for(size_t j=std::max((size_t)0,(size_t)(triangle.v[2][0]+edge_l[0]*(triangle.v[2][1]-(float)i))+1);j<=std::min((size_t)image.cols-1,(size_t)(triangle.v[2][0]+edge_r[0]*(triangle.v[2][1]-(float)i))); j++){
                Eigen::Vector2f r = Eigen::Vector2f();
                r << (float)j,(float)i;
                Eigen::Vector2f v0_v1_barycentric = barycentric_t_matrix_inv*(r-vertex_2);
                float v0_w = v0_v1_barycentric[0];
                float v1_w = v0_v1_barycentric[1];
                float v2_w = 1-v0_w-v1_w;
                
                // Eigen::Vector2f affine_mapped_coords = (triangle.tex[0]*v0_w + triangle.tex[1]*v1_w + triangle.tex[2]*v2_w);
                
                // triangle.tex[0] *=vertex_0_z_inv;
                // triangle.tex[1] *=vertex_1_z_inv;
                // triangle.tex[2] *=vertex_2_z_inv;

                float current_pixel_z_inv = 1/z0*v0_w + 1/z1*v1_w + 1/z2*v2_w;

                Eigen::Vector2f perspective_corrected_coords = (triangle.tex[0]*1/z0*v0_w + triangle.tex[1]*1/z1*v1_w + triangle.tex[2]*1/z2*v2_w)/current_pixel_z_inv;
                
                image.at<cv::Vec3b>(i,j) = mesh.texture.get_pixel(perspective_corrected_coords);
                // cv::Vec3b vertex_0_color = texture.at<cv::Vec3b>(triangle.tex[0][1], triangle.tex[0][0]);
                // cv::Vec3b vertex_1_color = texture.at<cv::Vec3b>(triangle.tex[1][1], triangle.tex[1][0]);
                // cv::Vec3b vertex_2_color = texture.at<cv::Vec3b>(triangle.tex[2][1], triangle.tex[2][0]);
                // image.at<cv::Vec3b>(i,j) = vertex_0_color*v0_w + vertex_1_color*v1_w + vertex_2_color*v2_w;
                // image.at<cv::Vec3b>(i,j)[0] = (uint8_t)255*v0_w;
                // image.at<cv::Vec3b>(i,j)[1] = (uint8_t)255*v1_w;
                // image.at<cv::Vec3b>(i,j)[2] = (uint8_t)255*v2_w;
            }
        }
    }
}

int main(int argc, char** argv){
    auto program_start = std::chrono::high_resolution_clock::now();
    // Mesh mesh_original = Mesh();
    // mesh_original.triangles.push_back(Triangle({0,0,0},{1,0,1},{0,0,1}));
    // mesh_original.triangles.push_back(Triangle({0,0,0},{1,0,0},{1,0,1}));

    // Mesh mesh_original = Mesh(stl_reader::StlMesh<float, unsigned int>("../../../3d_files/Utah_teapot_(solid).stl"), false);
    // Mesh mesh_original = Mesh(stl_reader::StlMesh<float, unsigned int>("../../../3d_files/cat.stl"));
    Mesh mesh_original = Mesh::fromOBJ(std::filesystem::path("../../../3d_files/cat_obj/12221_Cat_v1_l3.obj")).value_or(Mesh());
    mesh_original.texture = Texture("../../../3d_files/cat_obj/Cat_diffuse.jpg");
    // Mesh mesh_original = Mesh::fromOBJ(std::filesystem::path("../../../3d_files/checkerboard_obj/checkerboard.obj")).value_or(Mesh());
    // mesh_original.texture = Texture("../../../3d_files/checkerboard_obj/checkerboard_texture.png");
    // mesh_original.texture = Texture("../../../3d_files/checkerboard_obj/mario.jpeg");

    // std::vector<Triangle> triangles = {Triangle({0,0,0}, {1,0,1}, {0,0,1})};
    // Mesh mesh = Mesh();
    // mesh.triangles = triangles;
    Camera camera(1600, 900, 0.785398163f*2.0f);

    cv::Mat image = cv::Mat::zeros(camera.height, camera.width, CV_8UC3);
    camera.position = {0,-75,15};
    // camera.position = {0,-1,0.5};
    camera.orientation << 1, 0, 0,
                        0, 0, 1,
                        0, -1, 0;
    while(true){
        Mesh mesh = mesh_original.copy();
        image.setTo(cv::Scalar::zeros());
        
        double time_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-program_start).count() * 1e-9;

        // Eigen::AngleAxisf rotation(time_sec, Eigen::Vector3f::UnitX());
        // mesh.transform(rotation.matrix());
        // rotation = Eigen::AngleAxisf(time_sec/3, Eigen::Vector3f::UnitY());
        // mesh.transform(rotation.matrix());
        Eigen::AngleAxisf rotation = Eigen::AngleAxisf(time_sec, Eigen::Vector3f::UnitZ());
        mesh.transform(rotation.matrix());

        render_mesh(image, mesh, camera, {0,1,-1});
        cv::imshow("Aaa", image);
        // cv::imshow("Texture", mesh.texture.get_image());
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
    }
    cv::destroyAllWindows();
    return 0;
}