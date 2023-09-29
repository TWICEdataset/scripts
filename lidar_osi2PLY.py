with open('TWICE_path.txt', 'r') as f:
    path_twice = f.read()
import google.protobuf.message
import google.protobuf.json_format
import numpy as np
import sys
import os
import struct
import open3d as o3d
import math as m
sys.path.insert(1,os.path.join(path_twice,"TWICE","open-simulation-interface"))
from osi3.osi_sensordata_pb2 import SensorData


def polar2cart(r, azimuth, elevation):
    x = r * m.cos(elevation) * m.cos(azimuth)
    y = r * m.cos(elevation) * m.sin(azimuth)
    z = r * m.sin(elevation)
    position = [x,y,z]

    return(position)

def osi2ply(path_osi_file,path_ply_files):
    count = 0
    lidar_obj = SensorData()
    with open(path_osi_file,'rb') as f:
        while 1:
            message_size_bytes = f.read(struct.calcsize("<L"))
            if len(message_size_bytes) == 0:
                break
            message_size = struct.unpack("<L",message_size_bytes)[0]
            message_bytes = f.read(message_size)
            lidar_obj.ParseFromString(message_bytes)
            
            
            # Create an open3d point cloud object
            X = np.array([])
            Y = np.array([])
            Z = np.array([])
            # intensity_array = np.array([])
            # reflectivity_array = np.array([])
            for ind in range(len(lidar_obj.feature_data.lidar_sensor[0].detection)):
                r = lidar_obj.feature_data.lidar_sensor[0].detection[ind].position.distance
                azimuth = lidar_obj.feature_data.lidar_sensor[0].detection[ind].position.azimuth
                elevation = lidar_obj.feature_data.lidar_sensor[0].detection[ind].position.elevation
                time_stamp = lidar_obj.feature_data.lidar_sensor[0].header.measurement_time.seconds + ((lidar_obj.feature_data.lidar_sensor[0].header.measurement_time.nanos)/1000000000)
                # reflectivity = lidar_obj.feature_data.lidar_sensor[0].detection[ind].reflectivity
                # intensity = lidar_obj.feature_data.lidar_sensor[0].detection[ind].intensity

                #converting to cartesians
                cartesian = polar2cart(r,azimuth,elevation)

                #append values to arrays
                X = np.append(X,cartesian[0])
                Y = np.append(Y,cartesian[1])
                Z = np.append(Z,cartesian[2])
            #     reflectivity_array = np.append(reflectivity_array,reflectivity)
            #     intensity_array = np.append(intensity_array,intensity)
            # reflectivity_scaled = reflectivity_array * (1/np.amax(reflectivity_array)) 
            o3d_point_cloud = o3d.geometry.PointCloud()

            # Set the point cloud data
            o3d_point_cloud.points = o3d.utility.Vector3dVector(np.column_stack((X, Y, Z)))
            # colors = np.zeros((len(lidar_obj.feature_data.lidar_sensor[0].detection), 3))
            # colors[:, 0] = reflectivity_scaled  # set red channel to reflectivity
            # colors[:, 1] = reflectivity_scaled
            # colors[:, 2] = reflectivity_scaled    # set green channel to intensity
            # o3d_point_cloud.colors = o3d.utility.Vector3dVector(colors)

            # Save the point cloud to a PLY file
            o3d.io.write_point_cloud(os.path.join(path_ply_files,str(time_stamp)+".ply"), o3d_point_cloud, write_ascii=True)

            count +=1
