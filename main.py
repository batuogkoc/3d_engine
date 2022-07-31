import cv2 
import numpy as np
import time
from  scipy.spatial.transform import Rotation as R
import math as m
WIDTH = 1600
HEIGHT = 900
FPS = 30
prev_time = time.perf_counter()

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
    # if alphas == []:
    #     return image
    # Apply mask for pixels within the triangle only
    mask = (alphas > 0) & (betas > 0) & (gammas > 0)
    alphas_m = alphas[mask]
    betas_m = betas[mask]
    gammas_m = gammas[mask]
    xv_m = xv[mask]
    yv_m = yv[mask]

    xv_m[xv_m>=WIDTH] = WIDTH-1
    yv_m[yv_m>=HEIGHT] = HEIGHT-1
    
    def mul(a, b) :
        # Multiply two vectors into a matrix
        return np.asmatrix(b).T @ np.asmatrix(a)
    
    # Compute and assign colors
    colors = mul(a_color, alphas_m) + mul(b_color, betas_m) + mul(c_color, gammas_m)
    image[yv_m, xv_m] = colors
    return image

def render_triangles(image, triangles, triangle_colors, camera_position, camera_rotation, light_position):
    # print(triangles.shape)
    # centers = np.sum(triangles, axis=1)/3
    # edge1s = triangles[:,0]-triangles[:,2]
    # print(edge1s)
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
        v1_brightness = max(0, cos_angle_v1*(1/v1_distance_to_light_norm**2)) 
        v2_brightness = max(0, cos_angle_v2*(1/v2_distance_to_light_norm**2)) 
        v3_brightness = max(0, cos_angle_v3*(1/v3_distance_to_light_norm**2)) 
        
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

            colors = np.array([color*v1_brightness, color*v2_brightness, color*v3_brightness])
            image = smooth_triangle_lighting(image, triangle.reshape((3,2)).astype(int), (colors*15).astype(np.uint8))
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
    while True:
        start_time = time.perf_counter()

        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        quad = np.array([[[0, 0, 1], [0, 0, 0], [1, 0, 0]], [[0,0,1],[1,0,0],[1,0,1]]], dtype=np.float32)
        quad_f = quad.copy()
        quad_b = translate_triangles(rotate_triangles(quad.copy(), R.from_euler("XYZ", (0,0,m.pi)).as_matrix(), np.array([[0.5,0,0]]).T), np.array([[0,1,0]]).T)
        quad_r = translate_triangles(rotate_triangles(quad.copy(), R.from_euler("XYZ", (0,0,m.pi/2)).as_matrix(), np.array([[0,0,0]]).T), np.array([[1,0,0]]).T)
        quad_l = translate_triangles(rotate_triangles(quad.copy(), R.from_euler("XYZ", (0,0,-m.pi/2)).as_matrix(), np.array([[1,0,0]]).T), np.array([[-1,0,0]]).T)
        quad_t = translate_triangles(rotate_triangles(quad.copy(), R.from_euler("XYZ", (-m.pi/2,0,0)).as_matrix(), np.array([[0,0,0]]).T), np.array([[0,0,1]]).T)
        quad_d = translate_triangles(rotate_triangles(quad.copy(), R.from_euler("XYZ", (m.pi/2,0,0)).as_matrix(), np.array([[0,0,1]]).T), np.array([[0,0,-1]]).T)

        triangles = np.stack((quad_f, quad_b, quad_r, quad_l, quad_t, quad_d), axis=0)
        r = R.from_euler("XYZ", (0,0,time.perf_counter()))
        triangles = rotate_triangles(triangles, r.as_matrix(), np.array([[0.5,0.5,0.5]]).T)
        #triangles = translate_triangles(triangles, np.array([[0,m.sin(time.perf_counter()),0]]).T)
        camera_position = np.array([[0,-5,5]]).T
        camera_rotation = R.from_euler("XYZ", (-m.pi/2, 0, 0)).as_matrix()
        image = render_triangles(image, triangles, np.array([[255,0 ,0], [255,0 ,0], [0,0,255], [0,0,255], [0,255,0], [0,255,0], [255,0,255], [255,0,255], [0,255,255], [0,255,255], [255,255,0], [255,255,0]]), camera_position, camera_rotation, np.array([[0,-5,2]]).T)
        # image = cv2.putText(image, f"{1/delta_time:.2f}", (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0,255,0), thickness=1, lineType=cv2.LINE_AA)
        cv2.imshow("window", image)
        
        delta_time = time.perf_counter() - start_time
        print(f"{1/delta_time:.2f}")
        delay = int(max(1, (1/FPS-delta_time)*1000))
        inp = cv2.waitKey(delay)
        if inp == ord("q"):
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()