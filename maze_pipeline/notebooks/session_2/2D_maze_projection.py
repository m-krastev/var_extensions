import open3d as o3d
import numpy as np
# import matplotlib.pyplot as plt

# Step 1: Load the PLY file
point_cloud = o3d.io.read_point_cloud("../session_1/vsfm_matches_full/reconstruct_dense_full.0.ply") #week 1 maze
print('succeeded')

point_cloud_in_numpy = np.asarray(point_cloud.points) 
print(point_cloud_in_numpy.shape)

#o3d.visualization.draw_geometries([point_cloud]) 