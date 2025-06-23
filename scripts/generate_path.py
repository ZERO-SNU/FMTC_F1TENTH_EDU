'''
Smooth path logged with particle filter using B-Spline fitting
INPUT: path to .csv (x,y,yaw)
OUTPUT: .csv containing smoothed path
'''

import numpy as np
import matplotlib.pyplot as plt
import yaml
from PIL import Image
import os
import pandas as pd
from scipy.interpolate import splprep, splev

home_path = os.path.expanduser('~')

map_img_path = home_path+"/f1tenth_ws/src/particle_filter/maps/track_0604.pgm"
map_yaml_path = home_path+"/f1tenth_ws/src/particle_filter/maps/track_0604.yaml"
input_path = home_path+"/wp-2025-06-04-09-54-02.csv"
output_path = home_path+"/wp-2025-06-04-09-54-02_clean.csv"

map_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM))
map_img = map_img.astype(np.float64)

with open(map_yaml_path, 'r') as yaml_stream:
    try:
        map_metadata = yaml.safe_load(yaml_stream)
        map_resolution = map_metadata['resolution']
        origin = map_metadata['origin']
    except yaml.YAMLError as ex:
        print(ex)

# calculate map parameters
cut = 500
orig_x = origin[0]
orig_y = origin[1]
# ??? Should be 0
orig_s = np.sin(origin[2])
orig_c = np.cos(origin[2])

## load and visualize input path 
raw_data = pd.read_csv(input_path)
x = raw_data["# x_m"].values
x = x[:cut]
y = raw_data["y_m"].values
y = y[:cut]
yaw = raw_data["yaw_rad"].values

x_vis = x - orig_x
y_vis = y - orig_y

x_vis /= map_resolution
y_vis /= map_resolution

plt.figure(figsize=(10, 10))
plt.imshow(map_img, cmap="gray", origin="lower")
plt.plot(x_vis,y_vis)
plt.title("Inferred Path")
plt.show()

## interpolate
tck, u = splprep([x, y], s=0.5)
u_fine = np.linspace(0, 1, 2000)
new_points = splev(u_fine, tck)
x_smooth = new_points[0]
y_smooth = new_points[1]

x_smooth_vis = x_smooth - orig_x
y_smooth_vis = y_smooth - orig_y

x_smooth_vis /= map_resolution
y_smooth_vis /= map_resolution

plt.figure(figsize=(10, 10))
plt.imshow(map_img, cmap="gray", origin="lower")
plt.plot(x_smooth_vis, y_smooth_vis)
plt.title("Smoothed Raceline")
plt.show()

## write csv
df =  pd.DataFrame({'# x_m': x_smooth, 'y_m': y_smooth})
df.to_csv(output_path,index=False)
print("new path written")
