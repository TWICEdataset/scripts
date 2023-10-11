# with open('TWICE_path.txt', 'r') as f:
#     path_twice = f.read()
import math as m
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Ellipse
from matplotlib.collections import PatchCollection
from matplotlib.transforms import Affine2D
import cv2
import json
import ast
import google.protobuf.message
import google.protobuf.json_format
import sys
import os
import pathlib
import pandas as pd
from shapely.geometry import Polygon, Point
# sys.path.insert(1,os.path.join(path_twice,"TWICE","open-simulation-interface"))
from osi3.osi_sensordata_pb2 import SensorData
from osi3.osi_datarecording_pb2 import SensorDataSeries



def polar2cart(r, azimuth, elevation):
    x = r * m.cos(elevation) * m.cos(azimuth)
    y = r * m.cos(elevation) * m.sin(azimuth)
    z = r * m.sin(elevation)
    position = [x,y]

    return(position)

def center_rect(x,y,height,width, angle):
    # Define the vertices of the rectangle
    vertices = np.array([[-width/2, -height/2],
                         [width/2, -height/2],
                         [width/2, height/2],
                         [-width/2, height/2]])
    center = (x,y)
    # Rotate the rectangle
    rotation = Affine2D().rotate_deg(m.degrees(angle))
    vertices = rotation.transform(vertices)

    # Translate the rectangle
    vertices = vertices + center

    #return the bottom left vertice of the rectangle
    return (vertices[0][0],vertices[0][1])

def create_rect_poly(x1,y1,width,length,angle):

    x2, y2 = x1 + width*np.cos(angle+(m.pi/2)), y1 + width*np.sin(angle+(m.pi/2))
    x3, y3 = x2 + length*np.sin(angle+(m.pi/2)), y2 - length*np.cos(angle+(m.pi/2))
    x4, y4 = x3 - width*np.cos(angle+(m.pi/2)), y3 - width*np.sin(angle+(m.pi/2))

    return(Polygon([(x1,y1),(x2,y2),(x3,y3),(x4,y4)]))

def open_label_read(path_openlabel,frame,CCRs):
    f=open(path_openlabel)
    data_openlabel=json.load(f)
    last_frame = len(data_openlabel['openlabel']['frames'])
    if CCRs:
        index_obj = 0
    if frame < last_frame:
        #getting ego index relative to radar      
        index_ego=data_openlabel['openlabel']['frames'][frame]['frame_properties']['Streams']['IMU_ego']['stream_properties']['sync']['frame_stream']
        if CCRs == False:
            #getting obj index relative to radar
            index_obj=data_openlabel['openlabel']['frames'][frame]['frame_properties']['Streams']['IMU_obj']['stream_properties']['sync']['frame_stream']
    if frame >= last_frame:
        #getting ego index relative to radar
        index_ego=data_openlabel['openlabel']['frames'][last_frame-1]['frame_properties']['Streams']['IMU_ego']['stream_properties']['sync']['frame_stream']
        if CCRs == False:
            #getting obj index relative to radar
            index_obj=data_openlabel['openlabel']['frames'][last_frame-1]['frame_properties']['Streams']['IMU_obj']['stream_properties']['sync']['frame_stream']
    #creating list with indexes
    index_list = [frame,index_ego,index_obj]

    return index_list

def radar_cluster_plot(path_radar_osi_file,path_file,save_video,save_frame,frame_n,count_points,covariance,segmentation_data):
    #path to open label file
    path_openlabel = os.path.join(os.path.dirname(path_radar_osi_file) , "open_label_radar.json")

    #name of the video and image file
    path_name = pathlib.PurePath(path_radar_osi_file)
    list_elements = list(path_name.parts)
    name_file = "radar_"+list_elements[-7]+"_"+list_elements[-6]+"_"+list_elements[-5]+"_"+list_elements[-4]+"_"+list_elements[-3]
    name_file = str(name_file)
    title_plot = list_elements[-7]+" "+list_elements[-6]+" "+list_elements[-5]+" "+list_elements[-3]
    #ego osi file path
    path_ego = os.path.join(os.path.dirname(os.path.dirname(path_radar_osi_file)),"IMU_ego","ego_sv_350_300.osi")

    # obj osi file path
    path_obj = os.path.join(os.path.dirname(os.path.dirname(path_radar_osi_file)), "IMU_obj", "obj_sv_350_300.osi")

    CCRs = False
    if "CCRs" in list_elements:
        CCRs = True

    #Open osi files 
    radar_osi = SensorDataSeries()
    with open(path_radar_osi_file,'rb') as f:
        radar_osi.ParseFromString(f.read())
    ego = SensorDataSeries()
    with open(path_ego,'rb') as f:
        ego.ParseFromString(f.read())

    # getting indexes info
    list_indexes=[]
    for frame in range(len(radar_osi.sensor_data)):
        list_indexes.append(open_label_read(path_openlabel,frame,CCRs))

    #video parameters
    if save_video:
        size = (1280,960)
        out = cv2.VideoWriter(path_file+name_file+".avi",cv2.VideoWriter_fourcc(*'DIVX'), 13, size)


    #dataframe for points analisys
    df = pd.DataFrame(columns=['timestamp','id','x','y',"prob_exist","RCS"])


    for frame in range(len(radar_osi.sensor_data)):
        #getting ego grond truth from osi file
        ind_ego = list_indexes[frame][1]
        ego_angle = ego.sensor_data[ind_ego].sensor_view[0].host_vehicle_data.vehicle_motion.orientation.yaw + (m.pi/2)
        ego_x = ego.sensor_data[ind_ego].sensor_view[0].host_vehicle_data.vehicle_motion.position.x
        ego_y = ego.sensor_data[ind_ego].sensor_view[0].host_vehicle_data.vehicle_motion.position.y

        #getting object ground truth from osi file
        if "CCRs" in list_elements:
            object_x = -37.8
            object_y = -36.04
            object_width = 1.8
            object_length = 4.060
            object_angle = ego_angle
        else:
            #open osi obj
            obj_osi = SensorDataSeries()
            with open(path_obj,'rb') as f:
                obj_osi.ParseFromString(f.read())

            ind_obj = list_indexes[frame][2]
            object_x = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[0].base.position.x
            object_y = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[0].base.position.y
            object_width = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[0].base.dimension.width
            object_length = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[0].base.dimension.length
            object_angle = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[0].base.orientation.yaw + (m.pi/2)
            R_obj = np.array([[m.cos(object_angle), (-m.sin(object_angle))],[m.sin(ego_angle), m.cos(object_angle)]])
            if object_angle == m.pi/2: #ground truth data from SP80 don't have orientation data, only position
                object_angle = ego_angle

            if "with_car" in list_elements:
                #correct the bike position
                correction = np.matmul(R_obj,np.array([4,0]))
                object_2_x = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[1].base.position.x 
                object_2_y = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[1].base.position.y
                object_2_width = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[1].base.dimension.width
                object_2_length = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[1].base.dimension.length
                object_2_angle = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[1].base.orientation.yaw + (m.pi/2)   
                object_2_x_lb, object_2_y_lb = center_rect(object_2_x,object_2_y,object_2_width,object_2_length,object_2_angle)
                rectangle_2_obj = create_rect_poly(object_2_x_lb,object_2_y_lb,object_2_width,object_2_length,object_2_angle)
                
        #getting the left bottom cornor of the objects to plot the rectangles
        ego_x_lb, ego_y_lb = center_rect(ego_x,ego_y,1.559,2.695,ego_angle)
        object_x_lb, object_y_lb = center_rect(object_x,object_y,object_width,object_length,object_angle)
        rectangle_obj = create_rect_poly(object_x_lb,object_y_lb,object_width,object_length,object_angle)
        #Rotation matrix (ego) 
        R = np.array([[m.cos(ego_angle), (-m.sin(ego_angle))],[m.sin(ego_angle), m.cos(ego_angle)]])

        #Reading multiple detections
        X=np.array([])
        Y=np.array([])
        X_00=np.array([])
        Y_00=np.array([])
        X_25=np.array([])
        Y_25=np.array([])
        X_50=np.array([])
        Y_50=np.array([])
        X_75=np.array([])
        Y_75=np.array([])
        X_90=np.array([])
        Y_90=np.array([])
        X_99=np.array([])
        Y_99=np.array([])
        timestamp = (radar_osi.sensor_data[frame].feature_data.radar_sensor[0].header.measurement_time.seconds)+(radar_osi.sensor_data[frame].feature_data.radar_sensor[0].header.measurement_time.nanos/1000000000)
        #Plotting 
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')
        count = 0
        for ind in range(len(radar_osi.sensor_data[frame].feature_data.radar_sensor[0].detection)):
            r = radar_osi.sensor_data[frame].feature_data.radar_sensor[0].detection[ind].position.distance
            azimuth = (radar_osi.sensor_data[frame].feature_data.radar_sensor[0].detection[ind].position.azimuth)
            r_rmse = radar_osi.sensor_data[frame].feature_data.radar_sensor[0].detection[ind].position_rmse.distance
            azimuth_rmse = (radar_osi.sensor_data[frame].feature_data.radar_sensor[0].detection[ind].position_rmse.azimuth)
            prob_exist = radar_osi.sensor_data[frame].feature_data.radar_sensor[0].detection[ind].existence_probability
            id = radar_osi.sensor_data[frame].feature_data.radar_sensor[0].detection[ind].object_id.value
            rcs = radar_osi.sensor_data[frame].feature_data.radar_sensor[0].detection[ind].rcs
            #converting to cartesian coordinate 
            cartesian = polar2cart(r,azimuth,0)
            cartesian_rmse = polar2cart(r_rmse,azimuth_rmse,0)
            point = np.array([cartesian[0]+ 2.77,cartesian[1]+0.04])#2.77x and 0.04y is the radar mouting position
            point_rmse = np.array([cartesian_rmse[0],cartesian_rmse[1]])
            #putting cluster points in the same frame system of the ego
            rotated_point = np.matmul(R,point)
            rotated_point_rmse = np.matmul(R,point_rmse)
            x_point = rotated_point[0] + ego_x
            y_point = rotated_point[1] + ego_y
            #assign the cluster to the existence probability group
            X = np.append(X,x_point) 
            Y = np.append(Y,y_point)
            if prob_exist == 0:
                X_00 = np.append(X_00,x_point) 
                Y_00 = np.append(Y_00,y_point) 
                ellipse = Ellipse(xy =(x_point,y_point),width=2*rotated_point_rmse[0],height=2*rotated_point_rmse[1],angle=m.degrees(ego_angle),facecolor="black",alpha=0.5,edgecolor="none")
            if prob_exist == 0.25:
                X_25 = np.append(X_25,x_point) 
                Y_25 = np.append(Y_25,y_point)
                ellipse = Ellipse(xy =(x_point,y_point),width=2*rotated_point_rmse[0],height=2*rotated_point_rmse[1],angle=m.degrees(ego_angle),facecolor="grey",alpha=0.5,edgecolor="none")
            if prob_exist == 0.50:
                X_50 = np.append(X_50,x_point) 
                Y_50 = np.append(Y_50,y_point)
                ellipse = Ellipse(xy =(x_point,y_point),width=2*rotated_point_rmse[0],height=2*rotated_point_rmse[1],angle=m.degrees(ego_angle),facecolor="red",alpha=0.5,edgecolor="none")
            if prob_exist == 0.75:
                X_75 = np.append(X_75,x_point) 
                Y_75 = np.append(Y_75,y_point)
                ellipse = Ellipse(xy =(x_point,y_point),width=2*rotated_point_rmse[0],height=2*rotated_point_rmse[1],angle=m.degrees(ego_angle),facecolor="orange",alpha=0.5,edgecolor="none")
            if prob_exist == 0.90:
                X_90 = np.append(X_90,x_point) 
                Y_90 = np.append(Y_90,y_point) 
                ellipse = Ellipse(xy =(x_point,y_point),width=2*rotated_point_rmse[0],height=2*rotated_point_rmse[1],angle=m.degrees(ego_angle),facecolor="yellow",alpha=0.5,edgecolor="none")
            if prob_exist == 0.999:
                X_99 = np.append(X_99,x_point) 
                Y_99 = np.append(Y_99,y_point)   
                ellipse = Ellipse(xy =(x_point,y_point),width=2*rotated_point_rmse[0],height=2*rotated_point_rmse[1],angle=m.degrees(ego_angle),facecolor="green",alpha=0.5,edgecolor="none")
            #counting the number of points inside the object bbox
            ellipse_poly = Polygon(ellipse.get_verts()) 
            if rectangle_obj.intersects(ellipse_poly) or rectangle_obj.contains(Point(x_point,y_point)):
                    count += 1
                    new_row = pd.DataFrame({'timestamp':timestamp,'id':id,'x':x_point,'y':y_point,"prob_exist":prob_exist,"RCS":rcs},index=[df.shape[0]+1])
                    df = pd.concat([df,new_row], ignore_index=True)
            if "with_car" in list_elements:
                if rectangle_2_obj.intersects(ellipse_poly) or rectangle_2_obj.contains(Point(x_point,y_point)):
                    count += 1
                    new_row = pd.DataFrame({'timestamp':timestamp,'id':id,'x':x_point,'y':y_point,"prob_exist":prob_exist,"RCS":rcs},index=[df.shape[0]+1])
                    df = pd.concat([df,new_row], ignore_index=True)
            if covariance == True:
                plt.gca().add_patch(ellipse)
        plt.gca().add_patch(Rectangle((ego_x_lb,ego_y_lb),2.695,1.559,angle=m.degrees(ego_angle),edgecolor='red',facecolor='none',lw=1,label="ground truth"))
        plt.gca().add_patch(Rectangle((object_x_lb,object_y_lb),object_length,object_width,angle=m.degrees(object_angle),edgecolor='red',facecolor='none',lw=1))
        if "with_car" in list_elements:
            plt.gca().add_patch(Rectangle((object_2_x_lb,object_2_y_lb),object_2_length,object_2_width,angle=m.degrees(object_2_angle),edgecolor='red',facecolor='none',lw=1))
        P_00 = plt.scatter(X_00,Y_00,s=0.1,color="black")
        P_25 = plt.scatter(X_25,Y_25,s=0.1,color="grey")
        P_50 = plt.scatter(X_50,Y_50,s=0.1,color="red")
        P_75 = plt.scatter(X_75,Y_75,s=0.1,color="orange")
        P_90 = plt.scatter(X_90,Y_90,s=0.1,color="yellow")
        P_99 = plt.scatter(X_99,Y_99,s=0.1,color="green")
        #getting limits of the plot
        if frame == 0: # first position of the ego to delimit the plot
            x_min = ego_x -10
            y_min = ego_y -10
        plt.xlabel('X_world')
        plt.ylabel('Y_world')
        plt.title("TWICE radar plot: "+title_plot)
        plt.xlim([x_min,10])
        plt.ylim([y_min,-10])
        plt.figtext(0.05, 0.05, str(round(timestamp,3)), ha='left', va='bottom', fontsize=12)
        ax.legend((P_00,P_25,P_50,P_75,P_90,P_99),("0%","25%","50%","75%","90%","99%"),loc="upper center", title="Existence Probability",bbox_to_anchor=(0.5, 1.25), ncol=7,fontsize="small",labelspacing=0.2,markerscale=5)

        if count_points == True:
            count_text = "#clusters obj:" + str(count)
            plt.figtext(0.95, 0.05, count_text, ha='right', va='bottom', fontsize=12)

        if save_video:
            plt.savefig(path_file+"temporary_twice.png",dpi=200)
            im = cv2.imread(path_file+"temporary_twice.png")
            out.write(im)
        if save_frame and frame == frame_n:
            plt.savefig(path_file+name_file+"_"+str(frame_n)+".png",dpi=200)

        plt.cla()
        plt.close()
    if segmentation_data == True: 
        df.to_csv(path_file+name_file+".csv", index=False)
    os.remove(path_file+"temporary_twice.png")
    out.release()





#################################object list#######################






def radar_objlist_plot(path_radar_osi_file,path_file,save_video,save_frame,frame_n,IoU,segmentation_data):
    #path to open label file
    path_openlabel = os.path.join(os.path.dirname(path_radar_osi_file) , "open_label_radar.json")

    #name of the video and image file
    path_name = pathlib.PurePath(path_radar_osi_file)
    list_elements = list(path_name.parts)
    name_file = "radar_"+list_elements[-7]+"_"+list_elements[-6]+"_"+list_elements[-5]+"_"+list_elements[-4]+"_"+list_elements[-3]
    name_file = str(name_file)
    title_plot = list_elements[-7]+" "+list_elements[-6]+" "+list_elements[-5]+" "+list_elements[-3]
    #ego osi file path
    path_ego = os.path.join(os.path.dirname(os.path.dirname(path_radar_osi_file)),"IMU_ego","ego_sv_350_300.osi")
    
    path_obj = os.path.join(os.path.dirname(os.path.dirname(path_radar_osi_file)), "IMU_obj", "obj_sv_350_300.osi")

    CCRs = False
    if "CCRs" in list_elements:
        CCRs = True
    #Open osi files 
    radar_osi = SensorDataSeries()
    with open(path_radar_osi_file,'rb') as f:
        radar_osi.ParseFromString(f.read())
    ego = SensorDataSeries()
    with open(path_ego,'rb') as f:
        ego.ParseFromString(f.read())

    # getting indexes info
    list_indexes=[]
    for frame in range(len(radar_osi.sensor_data)):
        list_indexes.append(open_label_read(path_openlabel,frame,CCRs))
    
    if save_video:
        #video parameters
        size = (1280,960)
        out = cv2.VideoWriter(path_file+name_file+".avi",cv2.VideoWriter_fourcc(*'DIVX'), 13, size)

    df = pd.DataFrame(columns=['timestamp','id','x','y',"prob_exist","RCS","IoU"])

    for frame in range(len(radar_osi.sensor_data)):
        #timestamp
        timestamp = radar_osi.sensor_data[frame].timestamp.seconds + (radar_osi.sensor_data[frame].timestamp.nanos/1000000000)
        #getting ego grond truth from osi file
        ind_ego = list_indexes[frame][1]
        ego_angle = ego.sensor_data[ind_ego].sensor_view[0].host_vehicle_data.vehicle_motion.orientation.yaw + (m.pi/2)
        ego_x = ego.sensor_data[ind_ego].sensor_view[0].host_vehicle_data.vehicle_motion.position.x
        ego_y = ego.sensor_data[ind_ego].sensor_view[0].host_vehicle_data.vehicle_motion.position.y
        #Rotation matrix (ego) 
        R = np.array([[m.cos(ego_angle), (-m.sin(ego_angle))],[m.sin(ego_angle), m.cos(ego_angle)]])
        if "CCRs" in list_elements:
            object_x = -37.8
            object_y = -36.04
            object_width = 1.8
            object_length = 4.060
            object_angle = ego_angle
        else:
            #open osi obj
            obj_osi = SensorDataSeries()
            with open(path_obj,'rb') as f:
                obj_osi.ParseFromString(f.read())

            #getting object ground truth from osi file
            ind_obj = list_indexes[frame][2]
            object_x = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[0].base.position.x
            object_y = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[0].base.position.y
            object_width = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[0].base.dimension.width
            object_length = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[0].base.dimension.length
            object_angle = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[0].base.orientation.yaw + (m.pi/2)
            if object_angle == m.pi/2: #ground truth data from SP80 don't have orientation data, only position
                object_angle = ego_angle
            #CBLA jave two objects
            #Rotation matrix (ego) 
            if "with_car" in list_elements:
                R_obj = np.array([[m.cos(object_angle), (-m.sin(object_angle))],[m.sin(ego_angle), m.cos(object_angle)]])
                #correct the bike position
                correction = np.matmul(R_obj,np.array([4,0]))
                object_2_x = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[1].base.position.x 
                object_2_y = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[1].base.position.y
                object_2_width = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[1].base.dimension.width
                object_2_length = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[1].base.dimension.length
                object_2_angle = obj_osi.sensor_data[ind_obj].sensor_view[0].global_ground_truth.moving_object[1].base.orientation.yaw + (m.pi/2)   
                object_2_x_lb, object_2_y_lb = center_rect(object_2_x,object_2_y,object_2_width,object_2_length,object_2_angle)
                rectangle_2_obj = create_rect_poly(object_2_x_lb,object_2_y_lb,object_2_width,object_2_length,object_2_angle)
        #getting the left bottom cornor of the objects to plot the rectangles
        ego_x_lb, ego_y_lb = center_rect(ego_x,ego_y,1.559,2.695,ego_angle)
        object_x_lb, object_y_lb = center_rect(object_x,object_y,object_width,object_length,object_angle)

        rectangle_obj = create_rect_poly(object_x_lb,object_y_lb,object_width,object_length,object_angle)
        #creating plot 
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')
        #ego and object of interest
        plt.gca().add_patch(Rectangle((ego_x_lb,ego_y_lb),2.695,1.559,angle=m.degrees(ego_angle),edgecolor='red',facecolor='none',lw=1,label="ground truth"))
        plt.gca().add_patch(Rectangle((object_x_lb,object_y_lb),object_length,object_width,angle=m.degrees(object_angle),edgecolor='red',facecolor='none',lw=1))
        if "with_car" in list_elements:
            plt.gca().add_patch(Rectangle((object_2_x,object_2_y),object_2_length,object_2_width,angle=m.degrees(object_2_angle),edgecolor='red',facecolor='none',lw=1))
        #add radar detected objects to plot
        IoU_list = []
        for ind in range(len(radar_osi.sensor_data[frame].moving_object)):
            #getting object parameters from osi file
            prob_exist = radar_osi.sensor_data[frame].moving_object[ind].candidate[0].probability
            obj_det_length = radar_osi.sensor_data[frame].moving_object[ind].base.dimension.length
            obj_det_width = radar_osi.sensor_data[frame].moving_object[ind].base.dimension.width
            obj_det_angle = radar_osi.sensor_data[frame].moving_object[ind].base.orientation.yaw
            id = radar_osi.sensor_data[frame].moving_object[ind].header.tracking_id.value
            rcs = radar_osi.sensor_data[frame].moving_object[ind].radar_specifics.rcs
            cartesian = [radar_osi.sensor_data[frame].moving_object[ind].base.position.x,radar_osi.sensor_data[frame].moving_object[ind].base.position.y,0]
            point = np.array([cartesian[0]+2.77,cartesian[1]+0.04])#2.77x and 0.04y is the radar mouting position
            #putting object points in the same frame system of the ego
            rotated_point = np.matmul(R,point)
            x_point = rotated_point[0] + ego_x
            y_point = rotated_point[1] + ego_y
            #adding a object rectangle to the plot
            if prob_exist == 0.75:
                rect_colour = "orange"
            if prob_exist == 0.90:
                rect_colour = "yellow"    
            if prob_exist == 0.99:
                rect_colour = (0,1,0,0.4)           
            if prob_exist == 0.999:
                rect_colour = (0,1,0,1)
            if prob_exist == 1:
                rect_colour = "green"

            #plot object list detection
            obj_x_lb, obj_y_lb = center_rect(rotated_point[0]+ego_x,rotated_point[1]+ego_y,obj_det_width,obj_det_length,ego_angle)
            plt.gca().add_patch(Rectangle((obj_x_lb,obj_y_lb),obj_det_length,obj_det_width,angle=m.degrees(ego_angle+obj_det_angle),edgecolor=rect_colour,facecolor='none',lw=1))
            if IoU == True:
                rectangle_detect = create_rect_poly(obj_x_lb,obj_y_lb,obj_det_width,obj_det_length,ego_angle)
                if "with_car" in list_elements:
                    # Calculate the intersection of the rectangles
                    intersection = rectangle_detect.intersection(rectangle_obj)

                    # Calculate the union of the rectangles
                    union = rectangle_detect.union(rectangle_obj)

                    #same for object 2
                    intersection_2 = rectangle_detect.intersection(rectangle_2_obj)

                    # Calculate the union of the rectangles
                    union_2 = rectangle_detect.union(rectangle_2_obj)

                    # Calculate the IoU
                    if intersection_2 != 0 and intersection_2 != 0:
                        iou = ((intersection.area / union.area) + (intersection_2.area / union_2.area))/2
                    elif intersection != 0 and intersection_2 == 0:
                        iou = (intersection.area / union.area)
                    elif intersection == 0 and intersection_2 != 0:
                        iou = (intersection_2.area / union_2.area) 
                else:
                    # Calculate the intersection of the rectangles
                    intersection = rectangle_detect.intersection(rectangle_obj)

                    # Calculate the union of the rectangles
                    union = rectangle_detect.union(rectangle_obj)

                    # Calculate the IoU
                    iou = intersection.area / union.area

                if iou != 0:
                    IoU_list.append(iou)
                    new_row = pd.DataFrame({'timestamp':timestamp,'id':id,'x':x_point,'y':y_point,"prob_exist":prob_exist,"RCS":rcs,"IoU":iou},index=[df.shape[0]+1])
                    df = pd.concat([df,new_row], ignore_index=True)

        #fake rectangles outside of the map, to make the existence_prob legend
        plt.gca().add_patch(Rectangle((-250,-250),0.1,0.1,angle=m.degrees(ego_angle),edgecolor='green',facecolor='none',lw=1,label="100%"))
        plt.gca().add_patch(Rectangle((-250,-250),0.1,0.1,angle=m.degrees(ego_angle),edgecolor=(0,1,0,1),facecolor='none',lw=1,label="99.9%"))
        plt.gca().add_patch(Rectangle((-250,-250),0.1,0.1,angle=m.degrees(ego_angle),edgecolor=(0,1,0,0.4),facecolor='none',lw=1,label="99%"))
        plt.gca().add_patch(Rectangle((-250,-250),0.1,0.1,angle=m.degrees(ego_angle),edgecolor="yellow",facecolor='none',lw=1,label="90%"))
        plt.gca().add_patch(Rectangle((-250,-250),0.1,0.1,angle=m.degrees(ego_angle),edgecolor="orange",facecolor='none',lw=1,label="75%"))
        plt.xlabel('X_world')
        plt.ylabel('Y_world')
        plt.title("TWICE radar plot: "+title_plot)
        if frame == 0: # first position of the ego to delimit the plot
            x_min = ego_x -10
            y_min = ego_y -10
        plt.xlim([x_min,10])
        plt.ylim([y_min,-10])
        ax.legend(loc="upper center", title="Existence Probability",  bbox_to_anchor=(0.5, 1.25), ncol=6,fontsize="small",labelspacing=0.2)
        if len(IoU_list) != 0:
            text_iou = "IoU:"+str(round(sum(IoU_list),2))
            plt.figtext(0.95, 0.05, text_iou, ha='right', va='bottom', fontsize=12)
        plt.figtext(0.05, 0.05, str(round(timestamp,3)), ha='left', va='bottom', fontsize=12)


        #writing frame on video

        if save_video:
            plt.savefig(path_file+"temporary_twice.png",dpi=200)
            im = cv2.imread(path_file+"temporary_twice.png")
            out.write(im)
        if save_frame and frame == frame_n:
            plt.savefig(path_file+name_file+"_"+str(frame_n)+".png",dpi=200)

        plt.cla()
        plt.close()

    if segmentation_data == True: 
        df.to_csv(path_file+name_file+".csv", index=False)
    os.remove(path_file+"temporary_twice.png")
    out.release()

