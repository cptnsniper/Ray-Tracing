import numpy as np
from PIL import Image
import math

x = 160
y = 90
ray_max = 100
aspect = x / y

def dist(vec1, vec2):
    return math.sqrt(math.pow(vec2[0] - vec1[0], 2) + math.pow(vec2[1] - vec1[1], 2) + math.pow(vec2[2] - vec1[2], 2))

def mag(vec):
    return dist(np.array([0, 0, 0]), vec)

def dot(vec1, vec2):
    return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]

def angle(vec1, vec2):
    return math.acos(dot(vec1, vec2) / (mag(vec1) * mag(vec2)))

def map(value, min_in, max_in, min_out, max_out):
    return min_out + ((max_out - min_out) / (max_in - min_in) * (value - min_in))

def clamp(value, min1, max1):
    return max(min(value, max1), min1)

def normal(vec):
    return vec / mag(vec)

def move_along_vector(vec, dir, dist):
    return vec + normal(dir) * dist

def dist_sphere(point, pos, size):
    return dist(point, pos) - size

def dist_plane(point, pos, size):
    return point[1] - pos[1]

def closest_obj(point, objects):
    obj = None
    obj_dist = math.inf
    for o in objects:
        cur_dist = o.dist(point)
        if (cur_dist < obj_dist):
            obj = o
            obj_dist = cur_dist
    return (obj, obj_dist)

class obj:
    def __init__(self, position, size, color, dist_func):
        self.position = np.array(position)
        self.size = size
        self.color = color
        self.func = dist_func
    def dist(self, point):
        return self.func(point, self.position, self.size)

def draw(): 
    data = np.zeros((y, x, 3), dtype = np.uint8)
    for i in range (x):
        for j in range(y):
            u = i / x * 2 - 1
            v = j / y * 2 - 1
            u *= aspect
            # u *= -1
            v *= -1

            sphere1 = obj([-4, 7, 10], 5, [255, 100, 100], dist_sphere)
            sphere2 = obj([4, 6, 14], 7, [100, 100, 255], dist_sphere)
            ground = obj([0, 0, 0], 0, [100, 100, 100], dist_plane)
            objs = [sphere1, sphere2, ground]

            light = np.array([-8, 12, 2])
            light_pow = 100
            world = np.array([20, 20, 40])
            camera = np.array([0, 5, -2])
            orthographic = False
            focal_length = 1
            camera_dist = 100

            ray = camera
            ray_dir = np.array([ray[0] + u * focal_length, ray[1] + v * focal_length, ray[2] + 1])
            if (orthographic):
                ray = ray_dir
                ray[2] = camera[2]
                ray_dir = np.array([0, 0, 1 - camera[2]])
            hit = False
            for ii in range(ray_max):
                ray = move_along_vector(ray, ray_dir - camera, closest_obj(ray, objs)[1])
                dist_to_surface = closest_obj(ray, objs)[1]
                if (dist_to_surface < 0.001):
                    hit = True
                    break
                elif (dist_to_surface > camera_dist):
                    break
            
            shadow = False
            if (hit):
                shadow_ray = ray
                prev_dist = 0
                for ii in range(ray_max):
                    shadow_ray = move_along_vector(shadow_ray, light - ray, closest_obj(shadow_ray, objs)[1])
                    dist_to_surface = closest_obj(shadow_ray, objs)[1]
                    if (dist_to_surface < 0.0001 and prev_dist > dist_to_surface):
                        shadow = True
                        break
                    elif (dist_to_surface > camera_dist):
                        break
                    prev_dist = dist_to_surface

            darkness = 0
            cur_obj = closest_obj(ray, objs)[0]
            if (not False):
                light_pow = light_pow / pow(dist(ray, light), 2)
                light_angle = angle(light - cur_obj.position, ray - cur_obj.position)
                light_angle = clamp(0.5 - (light_angle / math.pi), 0, 1)
                darkness = light_pow * (light_angle) * 2
                # darkness = (0.5 - light_angle) * 2

            if (hit):
                    data[j, i] = [clamp(cur_obj.color[0] * darkness + world[0] / 255, 0, 255), clamp(cur_obj.color[1] * darkness + world[1] / 255, 0, 255), clamp(cur_obj.color[2] * darkness + world[2] / 255, 0, 255)]
            else:
                data[j, i] = world

    return data

img = Image.fromarray(draw())
img.show()