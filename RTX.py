import numpy as np
from PIL import Image
import math

x = 160 * 2
y = 90 * 2
ray_max = 10000
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

def dist_sphere(point, sphere, radius):
    return dist(point, sphere) - radius

def closest_obj(piont, objects):
    obj = None
    obj_dist = math.inf
    for o in objects:
        if (dist(piont, o) < obj_dist):
            obj = o
            obj_dist = dist(piont, o)
    return obj

def draw(): 
    data = np.zeros((y, x, 3), dtype = np.uint8)
    for i in range (x):
        for j in range(y):
            u = i / x * 2 - 1
            v = j / y * 2 - 1
            u *= aspect
            # u *= -1
            v *= -1

            sphere_origin = np.array([-4, 2, 10])
            radius = 5
            sphere_origin1 = np.array([4, -2, 15])
            radius1 = 7

            light = np.array([-5, 10, 0])

            ray = np.array([u * 10, v * 10, 0])
            hit = False
            for ii in range(ray_max):
                ray = move_along_vector(ray, np.array([0, 0, 1]), min(dist_sphere(ray, sphere_origin, radius), dist_sphere(ray, sphere_origin1, radius1)))
                dist_to_surface = min(dist_sphere(ray, sphere_origin, radius), dist_sphere(ray, sphere_origin1, radius1))
                if (dist_to_surface < 0.001):
                    hit = True
                    break
                elif (dist_to_surface > 100):
                    break
            
            shadow = False
            if (hit):
                shadow_ray = ray
                prev_dist = 0
                for ii in range(ray_max):
                    shadow_ray = move_along_vector(shadow_ray, light - ray, min(dist_sphere(shadow_ray, sphere_origin, radius), dist_sphere(shadow_ray, sphere_origin1, radius1)))
                    dist_to_surface = min(dist_sphere(shadow_ray, sphere_origin, radius), dist_sphere(shadow_ray, sphere_origin1, radius1))
                    if (dist_to_surface < 0.0001 and prev_dist > dist_to_surface):
                        shadow = True
                        break
                    elif (dist_to_surface > 100):
                        break
                    prev_dist = dist_to_surface

            darkness = 0
            obj = closest_obj(ray, [sphere_origin, sphere_origin1])
            if (not shadow):
                light_pow = 255 * 40 / pow(dist(ray, light), 2)
                light_angle = angle(light - obj, ray - obj)
                light_angle = clamp(light_angle / math.pi * 2, 0, 1)
                darkness = light_pow * (1 - light_angle)

            if (hit):
                if (obj[0] == sphere_origin[0]):
                    data[j, i] = [clamp(darkness * 2, 20, 255), clamp(darkness, 20, 255), clamp(darkness, 20, 255)]
                else:
                    data[j, i] = [clamp(darkness, 20, 255), clamp(darkness * 2, 20, 255), clamp(darkness, 20, 255)]
                # data[j, i] = [map(angle(light, ray - sphere_origin), 0, math.pi, 255, 0), map(angle(light, ray - sphere_origin), 0, math.pi, 255, 0), map(angle(light, ray - sphere_origin), 0, math.pi, 255, 0)]
            else:
                data[j, i] = [20, 20, 40]

    return data

img = Image.fromarray(draw())
img.show()