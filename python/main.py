import cv2 
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from collections import deque 
import math as m
from stl import mesh
WIDTH = 1600
HEIGHT = 900
FPS = 30
delta_ts = deque(maxlen=30)
prev_time = time.perf_counter()
image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

# class Camera():
#     def __init__(self, height:int, width:int, horizontal_fov:float):
#         self.__height = height
#         self.__width = width
#         self.__horizontal_fov = horizontal_fov
#         self.position = np.zeros((3,1))
#         self.orientation = R.from_euler("XYZ", (-m.pi/2, 0, 0)).as_matrix()

# class Scene():
#     def __init__(self):
#         self.__objects = {}
#         self.__lights = {}
#         self.__cameras = {}
    
#     def get_camera(self, camera_id):
#         return self.__cameras[camera_id]
    
#     def add_camera(self, camera_id, camera:Camera):
#         self.__cameras[camera_id] = camera

#     def add_object(self, object_id, object:Mesh):
#         self.__objects
#     def render_scene(self, camera_id):


# class Mesh():
#     def __init__(self, triangles, colors):
#         triangles_shape = triangles.shape
#         colors_shape = colors.shape
#         assert triangles_shape[0]==colors_shape[0] & triangles_shape[1:] == (3,3) & colors_shape[1:] == (3,), f"Triangles and colors must be of shape (n,3,3) and (n,3) respectively, not {triangles_shape} and {colors_shape}"

#         self.__triangles = triangles
#         self.__colors = colors
#         self.__normals = self.calculate_normals()
#         self.__normals_norm = np.linalg.norm(self.__normals, axis=1)

#     def calculate_normals(self):
#         vertex_1s = self.__triangles[:,0]
#         vertex_2s = self.__triangles[:,1]
#         vertex_3s = self.__triangles[:,2]
#         edge_1s = vertex_1s - vertex_3s
#         edge_2s = vertex_2s - vertex_3s
#         return np.cross(edge_1s, edge_2s, axis=1)


def smooth_triangle_lighting_parallel(image, triangles, corner_colors):
    triangle_arrays = np.hstack([np.stack((triangles[:,0], triangles[:,1], triangles[:,2]), axis=2), np.ones((triangles.shape[0],1,3))]).astype(int)
    x_lefts, y_tops = np.moveaxis(np.min(triangle_arrays[:,0:2], axis=2), 1, 0)
    x_rights, y_bottoms = np.moveaxis(np.max(triangle_arrays[:,0:2], axis=2), 1, 0) 
    mask = (x_rights != x_lefts) & (y_tops != y_bottoms)
    triangle_arrays = triangle_arrays[mask]
    x_lefts = x_lefts[mask]
    x_rights = x_rights[mask]
    y_bottoms = y_bottoms[mask]
    y_tops = y_tops[mask]
    print(x_lefts)
    print(x_rights)
    def arange(start, stop):
        return np.arange(start, stop)

    paralel_arange = np.frompyfunc(arange, 2, 1)
    xs = paralel_arange(x_lefts, x_rights)
    print(xs)
def smooth_triangle_lighting(image, triangle, colors):
    # Specify (x,y) triangle verticesq
    a, b, c = triangle.astype(int)
    # Specify colors
    a_color, b_color, c_color = colors.reshape((3,3))

    # Make array of vertices
    # ax bx cx
    # ay by cy
    #  1  1  1
    triArr = np.asarray([a[0],b[0],c[0], a[1],b[1],c[1], 1,1,1]).reshape((3, 3))
    # Get bounding box of the triangle
    xleft = min(a[0], b[0], c[0])
    xright = max(a[0], b[0], c[0])
    ytop = min(a[1], b[1], c[1])
    ybottom = max(a[1], b[1], c[1])


    if xleft == xright or ytop == ybottom:
        return image
    # Build np arrays of coordinates of the bounding box
    xs = range(xleft, xright)
    ys = range(ytop, ybottom)
    xv, yv = np.meshgrid(xs, ys)
    xv = xv.flatten()
    yv = yv.flatten()
    
    # Compute all least-squares /
    p = np.array([xv, yv, [1] * len(xv)])
    alphas, betas, gammas = np.linalg.lstsq(triArr, p, rcond=-1)[0]
    
    # Apply mask for pixels within the triangle only
    mask = (alphas > 0) & (betas > 0) & (gammas > 0)
    alphas_m = alphas[mask]
    betas_m = betas[mask]
    gammas_m = gammas[mask]
    xv_m = xv[mask]
    yv_m = yv[mask]

    xv_m[xv_m>=WIDTH] = WIDTH-1
    xv_m[xv_m<0] = 0
    yv_m[yv_m>=HEIGHT] = HEIGHT-1
    yv_m[yv_m<0] = 0
    
    def mul(a, b) :
        # Multiply two vectors into a matrix
        return np.asmatrix(b).T @ np.asmatrix(a)
    
    # Compute and assign colors
    colors = mul(a_color, alphas_m) + mul(b_color, betas_m) + mul(c_color, gammas_m)
    image[yv_m, xv_m] = colors
    return image

def render_triangles(image, triangles, triangle_colors, camera_position, camera_rotation, light_position, global_lighting=False):
    if True:
        vertex_1s = triangles[:,0]
        vertex_2s = triangles[:,1]
        vertex_3s = triangles[:,2]
        edge_1s = vertex_1s - vertex_3s
        edge_2s = vertex_2s - vertex_3s
        normals = np.cross(edge_1s, edge_2s, axis=1)
        centers = np.sum(triangles, axis=1)/3
        distances_to_camera = centers-camera_position.T
        cos_angles_camera = -np.sum(normals * distances_to_camera, axis=1)
        mask = cos_angles_camera > 0

        vertex_1s = vertex_1s[mask]
        vertex_2s = vertex_2s[mask]
        vertex_3s = vertex_3s[mask]

        normals = normals[mask]

        triangles = triangles[mask]
        triangle_colors = triangle_colors[mask]

        normal_lengths = np.linalg.norm(normals, axis=1)

        v1s_distance_to_light = (vertex_1s-light_position.T)
        v2s_distance_to_light = (vertex_2s-light_position.T)
        v3s_distance_to_light = (vertex_3s-light_position.T)
        v1s_distance_to_light_norm = np.linalg.norm(v1s_distance_to_light, axis=1)
        v2s_distance_to_light_norm = np.linalg.norm(v2s_distance_to_light, axis=1)
        v3s_distance_to_light_norm = np.linalg.norm(v3s_distance_to_light, axis=1)
        v1s_cos_angle = -(np.sum(normals * v1s_distance_to_light, axis=1))/(normal_lengths*v1s_distance_to_light_norm)
        v2s_cos_angle = -(np.sum(normals * v2s_distance_to_light, axis=1))/(normal_lengths*v2s_distance_to_light_norm)
        v3s_cos_angle = -(np.sum(normals * v3s_distance_to_light, axis=1))/(normal_lengths*v3s_distance_to_light_norm)
        v1s_brightness = np.clip(v1s_cos_angle*(1/(v1s_distance_to_light_norm))*2, 0, 1)
        v2s_brightness = np.clip(v2s_cos_angle*(1/(v2s_distance_to_light_norm))*2, 0, 1)
        v3s_brightness = np.clip(v3s_cos_angle*(1/(v3s_distance_to_light_norm))*2, 0, 1)
        
        # triangles = triangles[mask]
        triangles = (np.linalg.inv(camera_rotation) @ (triangles.reshape(-1,3).T-camera_position)).T.reshape(-1,3,3)
        triangles[:,:,0] /= triangles[:,:,2]
        triangles[:,:,1] /= triangles[:,:,2]
        triangle_zs = np.max(triangles, axis=1)[:,2]

        # triangles = triangles[:,:,0:2]
        triangles *= HEIGHT//2
        triangles[:,:,0] += WIDTH//2
        triangles[:,:,1] += HEIGHT//2
        
        order = np.argsort(triangle_zs)[::-1]
        if global_lighting:
            corner_colors = np.stack((triangle_colors, triangle_colors, triangle_colors), axis=1)
        else:
            corner_colors = np.stack(((triangle_colors.T*v1s_brightness).T, (triangle_colors.T*v2s_brightness).T, (triangle_colors.T*v3s_brightness).T), axis=1)
        for colors, triangle in zip(corner_colors[order], triangles[:,:,0:2][order]):
            if not global_lighting:
                smooth_triangle_lighting(image, triangle.reshape((3,2)).astype(int), (colors).astype(np.uint8))
            else:
                image = cv2.fillPoly(image, (triangle.astype(np.int),), colors[0].tolist())
        # smooth_triangle_lighting_parallel(image, triangles[:,:,0:2][order], corner_colors[order])
    else:
        for triangle, color in zip(triangles, triangle_colors):
            vertex_1, vertex_2, vertex_3 = triangle.reshape(-1,3)
            vertex_1 = vertex_1.reshape(3,1)
            vertex_2 = vertex_2.reshape(3,1)
            vertex_3 = vertex_3.reshape(3,1)
            center = (vertex_1 + vertex_2 + vertex_3)/3
            edge_1 = vertex_1-vertex_3
            edge_2 = vertex_2-vertex_3
            normal = np.cross(edge_1, edge_2, axis=0)
            normal_len = np.linalg.norm(normal)
            v1_distance_to_light = (vertex_1-light_position)
            v2_distance_to_light = (vertex_2-light_position)
            v3_distance_to_light = (vertex_3-light_position)

            v1_distance_to_light_norm = np.linalg.norm(v1_distance_to_light)
            v2_distance_to_light_norm = np.linalg.norm(v2_distance_to_light)
            v3_distance_to_light_norm = np.linalg.norm(v3_distance_to_light)
            
            cos_angle_v1 = -(normal.T @ v1_distance_to_light)[0,0]/(normal_len*v1_distance_to_light_norm)
            cos_angle_v2 = -(normal.T @ v2_distance_to_light)[0,0]/(normal_len*v2_distance_to_light_norm)
            cos_angle_v3 = -(normal.T @ v3_distance_to_light)[0,0]/(normal_len*v3_distance_to_light_norm)
            v1_brightness = max(0, cos_angle_v1*(1/(v1_distance_to_light_norm**2))) 
            v2_brightness = max(0, cos_angle_v2*(1/(v2_distance_to_light_norm**2))) 
            v3_brightness = max(0, cos_angle_v3*(1/(v3_distance_to_light_norm**2))) 
            distance_to_camera = (center-camera_position)
            cos_angle_camera = -(normal.T @ distance_to_camera)
            triangle.reshape(-1,3,3)
            if cos_angle_camera > 0:
                triangle = (np.linalg.inv(camera_rotation) @ (triangle.reshape(-1,3).T - camera_position)).T.reshape(-1,3,3)
                triangle[:,:,0] /= triangle[:,:,2]
                triangle[:,:,1] /= triangle[:,:,2]
                
                triangle = triangle[:,:,0:2]
                triangle *= HEIGHT//2
                triangle[:,:,0] += WIDTH//2
                triangle[:,:,1] += HEIGHT//2

                if global_lighting:
                    colors = np.array([color, color, color])
                else:
                    colors = np.array([color*v1_brightness, color*v2_brightness, color*v3_brightness])

                smooth_triangle_lighting(image, triangle.reshape((3,2)).astype(int), (colors*15).astype(np.uint8))
                # image = cv2.fillPoly(image, (triangle.astype(np.int),), (255,255,255))
   
    return image

def rotate_triangles(triangles, rotation_matrix, center_of_rotation):
    triangles = triangles.reshape(-1,3).T - center_of_rotation
    triangles = rotation_matrix @ triangles
    triangles += center_of_rotation
    return (triangles.T).reshape(-1,3,3)

def translate_triangles(triangles, translation_vector):
    triangles = triangles.reshape(-1,3).T + translation_vector
    return (triangles.T).reshape(-1,3,3)

def main():
    global prev_time
    quad = np.array([[[0, 0, 1], [0, 0, 0], [1, 0, 0]], [[0,0,1],[1,0,0],[1,0,1]]], dtype=np.float32)
    quad_f = quad.copy()
    quad_b = translate_triangles(rotate_triangles(quad.copy(), R.from_euler("XYZ", (0,0,m.pi)).as_matrix(), np.array([[0.5,0,0]]).T), np.array([[0,1,0]]).T)
    quad_r = translate_triangles(rotate_triangles(quad.copy(), R.from_euler("XYZ", (0,0,m.pi/2)).as_matrix(), np.array([[0,0,0]]).T), np.array([[1,0,0]]).T)
    quad_l = translate_triangles(rotate_triangles(quad.copy(), R.from_euler("XYZ", (0,0,-m.pi/2)).as_matrix(), np.array([[1,0,0]]).T), np.array([[-1,0,0]]).T)
    quad_t = translate_triangles(rotate_triangles(quad.copy(), R.from_euler("XYZ", (-m.pi/2,0,0)).as_matrix(), np.array([[0,0,0]]).T), np.array([[0,0,1]]).T)
    quad_d = translate_triangles(rotate_triangles(quad.copy(), R.from_euler("XYZ", (m.pi/2,0,0)).as_matrix(), np.array([[0,0,1]]).T), np.array([[0,0,-1]]).T)

    cube = np.stack((quad_f, quad_b, quad_r, quad_l, quad_t, quad_d), axis=0).reshape((-1,3,3))

    teapot = mesh.Mesh.from_file("3d_files/Utah_teapot_(solid).stl")
    triangles_original = teapot.points.reshape((-1,3,3))

    colors = (np.random.rand(triangles_original.shape[0],3)*255).astype(np.uint8)
    # colors = (np.ones((triangles_original.shape[0],3))*255).astype(np.uint8)
    while True:
        start_time = time.perf_counter()

        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        triangles = triangles_original.copy()
        r = R.from_euler("XYZ", (0,0,time.perf_counter()/4))
        triangles = rotate_triangles(triangles, r.as_matrix(), np.array([[0.5,0.5,0.5]]).T)
        #triangles = translate_triangles(triangles, np.array([[0,m.sin(time.perf_counter()),0]]).T)
        camera_position = np.array([[0,-15,5]]).T
        camera_rotation = R.from_euler("XYZ", (-m.pi/2, 0, 0)).as_matrix()
        image = render_triangles(image, triangles, colors, camera_position, camera_rotation, np.array([[0,-10,5]]).T, global_lighting=True)
        # image = render_triangles(image, triangles, np.array([[255,0 ,0], [255,0 ,0], [0,0,255], [0,0,255], [0,255,0], [0,255,0], [255,0,255], [255,0,255], [0,255,255], [0,255,255], [255,255,0], [255,255,0]]), camera_position, camera_rotation, np.array([[0,-5,2]]).T, global_lighting=True)
        # image = cv2.putText(image, f"{1/delta_time:.2f}", (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0,255,0), thickness=1, lineType=cv2.LINE_AA)
        cv2.imshow("window", image)
        
        delta_time = time.perf_counter() - start_time
        delta_ts.append(delta_time)
        print(f"{1/(sum(delta_ts)/len(delta_ts)):.2f}")
        delay = int(max(1, (1/FPS-delta_time)*1000))
        inp = cv2.waitKey(delay)
        if inp == ord("q"):
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()