import numpy as np
from PIL import Image
import math

x = 160
y = 90
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

def draw(): 
    data = np.zeros((y, x, 3), dtype = np.uint8)
    for i in range (x):
        for j in range(y):
            u = i / x * 2 - 1
            v = j / y * 2 - 1
            u *= aspect

            sphere_origin = np.array([-4, -2, 5])
            radius = 5
            sphere_origin1 = np.array([4, 2, 5])
            radius1 = 5

            light = np.array([-10, -5, 2])

            ray = np.array([u * 10, v * 10, 0])
            hit = False
            for ii in range(100):
                ray = move_along_vector(ray, np.array([0, 0, 1]), min(dist_sphere(ray, sphere_origin, radius), dist_sphere(ray, sphere_origin1, radius1)))
                if (min(dist_sphere(ray, sphere_origin, radius), dist_sphere(ray, sphere_origin1, radius1)) < 0.01):
                    hit = True
                    break
            
            shadow = False
            if (hit):
                shadow_ray = ray
                for ii in range(100):
                    shadow_ray = move_along_vector(shadow_ray, light, min(dist_sphere(shadow_ray, sphere_origin, radius), dist_sphere(shadow_ray, sphere_origin1, radius1)))
                    if (min(dist_sphere(shadow_ray, sphere_origin, radius), dist_sphere(shadow_ray, sphere_origin1, radius1)) < 0.001):
                        shadow = True
                        break
            
            # light_angle = angle(light, ray - sphere_origin)
            # darkness = map(light_angle, 0, math.pi, 0, 255)
            # darkness -= 255 / 2

            if (shadow):
                darkness = 20
            else:
                darkness = 255

            darkness = clamp(darkness, 20, 255)

            if (hit):
                data[j, i] = [darkness * 2, darkness, darkness]
            else:
                data[j, i] = [20, 20, 40]

    return data

img = Image.fromarray(draw())
img.show()