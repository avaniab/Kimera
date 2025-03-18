#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Author: Ekta Samani 

TOPS Descriptor computation with ROS Wrapper 

"""

import rospy
import open3d as o3d
import numpy as np
import csv
import sensor_msgs.msg 
from sensor_msgs.point_cloud2 import read_points
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from sensor_msgs.msg import Image 
from persim import PersistenceImager
import cv2
import copy
import threading
from collections import defaultdict
import os
import ros_numpy
from collections import deque
import json
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray

def print_logo():
    logo = """
    
     S M A R T S  
----------------------
        L A B    
----------------------

    """
    print(logo)

print_logo()

current_object_id = None
object

#rgb_to_obj_id_mapping = {}
segmentation_img = None
depth_img = None
#after numpify segmentatin_img
image_segmentation = None


pimgr = PersistenceImager()
pimgr.birth_range = (0,0.75)
pimgr.pers_range = (0,0.75)
pimgr.kernel_params = {'sigma': 0.00025}
pimgr.pixel_size = 0.025


points_by_object_id = {}  
object_ids_to_process = deque()  
processing_lock = threading.Lock()  
label = np.zeros((480, 720), dtype=int) 

def process_object(object_id):
    global points_by_object_id, object_ids_to_process, image_segmentation, depth_img, image_segmentation, current_object_id
    
    # Lock to ensure only one process at a time
    with processing_lock:
        if object_id in points_by_object_id:
            rospy.loginfo(f"Processing object id {object_id} started.")
            
            points = points_by_object_id[object_id]
            
            # 500 for now 

            if len(points) < 500:
                rospy.logwarn(f"Object {object_id} has {len(points)} points, skipping is insufficient.")
                return
            
            rospy.loginfo(f"Object {object_id} has {len(points)} points")

            np_points = np.array(points)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np_points)


            cam1 = np.expand_dims(np.asarray([0, 0, -0.1]), axis=0)
            cam2 = np.expand_dims(np.asarray([0, 0, 0]), axis=0)

            obj_id_in_segmentation = object_id

            tops_descriptor= main_compute_TOPS(pcd, cam1, cam2, depth_img, obj_id_in_segmentation, maxLayers=7)

            rospy.loginfo(f"Computed TOPS descriptor for object {object_id}: {tops_descriptor}")
            
            current_object_id = object_id
            # Save the descriptor to CSV
            save_descriptor_to_csv(tops_descriptor)
            tops_publisher(tops_descriptor)


            if object_ids_to_process:
                next_object_id = object_ids_to_process.popleft()  
                current_object_id = next_object_id
                process_object(next_object_id)


def point_cloud_callback(msg):
    global points_by_object_id, object_ids_to_process


    points = pc2.read_points(msg, field_names=("x", "y", "z", "object_id"), skip_nans=True)


    for point in points:
        x, y, z, object_id = point
        
        if object_id not in points_by_object_id:
            points_by_object_id[object_id] = []
        points_by_object_id[object_id].append([x, y, z])


    rospy.loginfo(f"Point cloud callback has given {len(points_by_object_id)} object IDs with points.")


    if object_id not in object_ids_to_process:
        object_ids_to_process.append(object_id)
        rospy.logwarn(f" OBJECT IDS TO PROCESS {object_ids_to_process}")


    if len(object_ids_to_process) == 1:  
        process_object(object_ids_to_process.popleft())


"""
    
def rgb_to_object_id_mapping_callback(msg):
    global rgb_to_obj_id_mapping
    try:
        # parse  to dictionary 
        mapping = json.loads(msg.data)
        
        for rgb_str, object_id in mapping.items():
            # Convert the RGB value to packed hex value
            rgb_value_int = int(rgb_str)  
            rgb_hex_str = hex(rgb_value_int)  # convert to hexadecimal string
            
            # Extract RGB components from hex
            r = (rgb_value_int >> 16) & 0xFF
            g = (rgb_value_int >> 8) & 0xFF
            b = rgb_value_int & 0xFF

            rgb_value_tuple = (r, g, b)
            rgb_to_obj_id_mapping[rgb_value_tuple] = object_id

            rospy.loginfo_once(f"Added RGB value {rgb_value_tuple} (hex: {rgb_hex_str}) with Object ID {object_id}")
        
        rospy.loginfo_once(f"Updated rgb_to_obj_id_mapping: {rgb_to_obj_id_mapping}")

    except (json.JSONDecodeError, ValueError) as e:
        rospy.logwarn(f"Invalid RGB data received: {msg.data} (Error: {e})")

"""

def label_matrix_callback(msg):
    global label
    rospy.loginfo("in label matrix callback ")
    label = ros_numpy.numpify(msg)
"""
    plt.figure(figsize=(10, 10))
    plt.imshow(label, cmap='jet', interpolation='nearest')
    plt.colorbar(label='Object ID')
    plt.title("Label Matrix Visualization")
    plt.show()
"""
def depth_img_callback(msg):
    global depth_img
    
    depth_img = ros_numpy.numpify(msg)

def save_descriptor_to_csv(tops_descriptor):

# the output file path
    output_file = "/home/avaniab/catkin_ws/src/semantics_tops_integrator/src/logging_stuff/tops_log.csv"
    
    directory = os.path.dirname(output_file)
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
            rospy.loginfo(f"Created directory: {directory}")
        except Exception as e:
            rospy.logerr(f"Failed to create directory: {str(e)}")
            return  # Exit if the directory cannot be created

    rospy.logwarn(f"From tops_node: saved to {output_file}")
    
    try:
        with open(output_file, mode="w", newline="") as file:
            writer = csv.writer(file)

           
            if isinstance(tops_descriptor, dict):
                for key, value in tops_descriptor.items():
                    writer.writerow([key, value])
            else:
                writer.writerow(tops_descriptor)  

        rospy.loginfo(f"Saved output to {output_file}")
    except Exception as e:
        rospy.logerr(f"Failed to save descriptor to CSV: {str(e)}")



def listener():
    rospy.init_node('tops_node', anonymous=True)

    # Subscriber setup
    rospy.Subscriber("/objects_of_interest", sensor_msgs.msg.PointCloud2, point_cloud_callback)
    rospy.Subscriber('/label_matrix', Image, label_matrix_callback)
    rospy.Subscriber('/tesse/depth/image_raw', Image, depth_img_callback)
    rospy.spin()

    


def tops_publisher(tops_descriptor):
        # Publisher setup
    tops_descriptor_pub = rospy.Publisher('/tops_descriptor', Float64MultiArray, queue_size=10)
        # loop
    while not rospy.is_shutdown():
        flattened_array = tops_descriptor.flatten()
        rospy.loginfo_once(f"tops descriptor shape: {tops_descriptor.shape}")
        
        msg = Float64MultiArray()
        msg.data = flattened_array.tolist()
        
        tops_descriptor_pub.publish(msg)
        rospy.loginfo_once(f"Published tops_descriptor of length {len(flattened_array)}")
        object_id_processed(current_object_id)
        rospy.sleep(0.1)

def object_id_processed(current_object_id):
    object_id_processed_pub = rospy.Publisher('/object_id_processed', String, queue_size=10)
    while not rospy.is_shutdown():
        current_object_id = str(current_object_id)
        object_id_processed_pub.publish(current_object_id)
        rospy.loginfo_once(f"processed object id {current_object_id} and published")

def rotateToFlatForLayering(pcd):
    try:
        pcdpts = np.asarray(pcd.points)[:-2,:]
        bbox = o3d.geometry.OrientedBoundingBox()
        bboxresult = bbox.create_from_points(o3d.utility.Vector3dVector(pcdpts))#,robust=True)
        Rnew = np.transpose(bboxresult.R)
        pcd.rotate(Rnew)

        #the angle is wrt to point with highest y. cw rotation of x axis until it meets an edge of bb
        w2,h2,angle2 = get2dboundingboxXYEfficient(np.asarray(pcd.points)[:-2,:])

        if h2 < w2:
            angles = [0,0, (angle2*np.pi)/180]
            R2dxy = o3d.geometry.get_rotation_matrix_from_xyz(angles)
            pcd.rotate(R2dxy)
        else:
            angle2 = 90-angle2
            angles = [0,0,-(angle2*np.pi)/180]
            R2dxy = o3d.geometry.get_rotation_matrix_from_xyz(angles)
            pcd.rotate(R2dxy)    

        w1,h1,angle1 = get2dboundingboxYZEfficient(np.asarray(pcd.points)[:-2,:])

        if h1 < w1:
            angles = [(angle1*np.pi)/180,0,0]
            R2dyz = o3d.geometry.get_rotation_matrix_from_xyz(angles)
            pcd.rotate(R2dyz)
        else:
            angle1 = 90-angle1
            angles = [-(angle1*np.pi)/180,0,0]
            R2dyz = o3d.geometry.get_rotation_matrix_from_xyz(angles)
            pcd.rotate(R2dyz)

        campos = np.asarray(pcd.points)[-1,:]

        if campos[2] > 0:
            Rtemp = o3d.geometry.get_rotation_matrix_from_xyz([np.pi,0,0])
            pcd.rotate(Rtemp)

        pts = np.asarray(pcd.points)[:-2,:]
        bbox = o3d.geometry.AxisAlignedBoundingBox()
        bboxresult = bbox.create_from_points(o3d.utility.Vector3dVector(pts))#,robust=True)
        extent = bboxresult.get_extent()  
        rospy.loginfo("function rotateToFlatForLayering successful")
        return pcd,extent  #,pts
    except Exception as e:
        rospy.logerr(f"Error in rotateToFlatForLayering: {str(e)}")


def get2dboundingboxXYEfficient(points):
    try: 
        final = my_scatter_plot_xy(points)
        final = final.astype(np.uint8) 
        kernel = np.ones((11, 11), np.uint8)
        final = cv2.erode(final, kernel, cv2.BORDER_REFLECT) 

        _,thresh = cv2.threshold(final,127,255,cv2.THRESH_BINARY_INV)

        cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cnt = sorted(cnts, key=cv2.contourArea)

        rect = cv2.minAreaRect(cnt[-1]) 
        (x,y),(w,h), a = rect # a - angle
        rospy.loginfo("function get2dboundingboxXYEfficient successful")

        return w,h,a
    except Exception as e:
        rospy.logerr(f"Error in get2dboundingboxXYEfficient: {str(e)}")

        


def get2dboundingboxYZEfficient(points):
    try:
        final = my_scatter_plot_yz(points)
        final = final.astype(np.uint8) 
        kernel = np.ones((11, 11), np.uint8)

        final = cv2.erode(final, kernel, cv2.BORDER_REFLECT) 

        _,thresh = cv2.threshold(final,127,255,cv2.THRESH_BINARY_INV)

        cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cnt = sorted(cnts, key=cv2.contourArea)

        rect = cv2.minAreaRect(cnt[-1]) 
        (x,y),(w,h), a = rect # a - angle

        rospy.loginfo("function get2dboundingboxYZEfficient successful")

        return w,h,a
    except Exception as e:
        rospy.logerr(f"Error in get2dboundingboxYZEfficient: {str(e)}")



def my_scatter_plot_xy(points):
    nx,ny = (224,224)
    xmin = np.min(points[:,0])
    ymin = np.min(points[:,1])
    img = 255*np.ones((nx,ny))
    
    x = np.linspace(xmin - 0.1,xmin+0.9,nx)
    y = np.linspace(ymin-0.1,ymin+0.9,ny)
    xbins = np.digitize(points[:,0],x)-1
    ybins = len(y) - np.digitize(points[:,1],y)
    
    for i in range(len(points)):

        img[ybins[i],xbins[i]] = 0
    rospy.loginfo("function my_scatter_plot_xy successful")
    return img


def my_scatter_plot_yz(points):
    ny,nz = (224,224)
    ymin = np.min(points[:,1])
    zmin = np.min(points[:,2])
    img = 255*np.ones((ny,nz))
    
    y = np.linspace(ymin - 0.1,ymin+0.9,ny)
    z = np.linspace(zmin-0.1,zmin+0.9,nz)
    ybins = np.digitize(points[:,1],y)-1
    zbins = len(z) - np.digitize(points[:,2],z)
    
    for i in range(len(points)):
        img[zbins[i],ybins[i]] = 0
    rospy.loginfo("function m_ scatterplot_yz successful")
    return img  

def roundinXYZ(pts):
    try: 
        pcdpts = copy.deepcopy(pts)
        pcdpts[:,0] = np.round(pcdpts[:,0],2)
        pcdpts[:,1] = np.round(pcdpts[:,1],2)
        pcdpts[:,2] = np.round(pcdpts[:,2],1)


        rospy.loginfo("function roundinXYZ successful")
        return pcdpts

    except Exception as e:
            # Log an error if something goes wrong
            rospy.logerr(f"Error in rotateToFlatForLayering: {str(e)}")


def trZMinusCam(pcd):
    try:
        pts = np.asarray(pcd.points)[:-2,:]
        pcd.translate([0,0,-np.min(pts[:,2])])
        rospy.loginfo("function trZMinusCam successful")
        return pcd
    except Exception as e:
        rospy.logerr(f"Erroro in trZMinusCam: {str(e)}")



def trYMinusCam(pcd):
    try:
        pts = np.asarray(pcd.points)[:-2,:]
        pcd.translate([0,-np.min(pts[:,1]),0])
        rospy.loginfo("function trYMinusCam successful")
        return pcd
    except Exception as e:
        rospy.logerr(f"Erroro in trYMinusCam: {str(e)}")   


def trXMinusCam(pcd):
    try: 
        pts = np.asarray(pcd.points)[:-2,:]
        pcd.translate([-np.min(pts[:,0]),0,0])
        rospy.loginfo("function trXMinusCam successful")

        return pcd
    
    except Exception as e:
        rospy.logerr(f"Error in trXMinusCam: {str(e)}")
"""
check need to flip minus cam
"""

def getZs(pcdpts):
    try:
        zlist = sorted(np.unique(pcdpts[:,2]))
        zs = {}
        for idx,num in enumerate(zlist):
            zs[idx] = num
        rospy.loginfo("function getZs successful")
        return zs
        
    except Exception as e:
        rospy.logerr(f"Error in getZs: {str(e)}")



def getLayer(pcdpts,zs,layeridx):
    try:
        rospy.loginfo("function getLayer successful")
        return pcdpts[np.where(pcdpts[:,2] == zs[layeridx])]
    except Exception as e:
        rospy.logerr(f"Error in getLayer: {str(e)}")

def computePDBinningNo2DTranslation(pcd):

    try: 
        #binning
        rospy.logwarn(f"computePDBinningNo2DTranslation input pcd : {pcd}")
        bins = np.arange(0,7.775,0.025) # will change later based on size of obj in the scene
        xes = copy.deepcopy(pcd[:,0])
        
        rospy.logwarn(f"xes values range from {np.min(xes)} to {np.max(xes)}") 
        pcd[:,0] = bins[np.digitize(xes,bins,right=True)]
        xesnew = np.unique(pcd[:,0])
        dgm = []
        for idx,x in enumerate(xesnew):
            ymax = np.max(pcd[np.where(pcd[:,0] == x)][:,1])
            ymin = np.min(pcd[np.where(pcd[:,0] == x)][:,1])

            dgm.append((x,x+ymax-ymin,0))
        rospy.loginfo("function computePDBinningNo2dTranslation successful")
        return np.asarray(dgm)
    
    except Exception as e:
        rospy.logerr(f"Error in computePDBinningNo2DTranslation: {str(e)}")


def getFeatureNewPad(pis,maxLayers):
    try:
        features = np.reshape(pis[0],(1,1024))
        rospy.loginfo(f"features shape: {features.shape}")
        for l in range(1,maxLayers):
            if l in pis:
                features = np.concatenate((features, np.reshape(pis[l],(1,1024))),axis=1)
                rospy.loginfo(f"features shape as index in pis: {features.shape}")
            else:
                features = np.concatenate((features, np.ones((1,1024))),axis=1)
                rospy.loginfo(f"features shape not in pis: {features.shape}")
        rospy.loginfo("function getFeatureNewPad successful")
        return features
        
    except Exception as e:
        rospy.logerr(f"Error in getFeatureNewPad: {str(e)}")

def mygetCamPosViewingDirection(pcd):
    try:
        points = np.asarray(pcd.points)
        cam2tr = points[-1,:]
        cam1tr = points[-2,:]
        direction = cam2tr-cam1tr
        unitvec = direction/np.linalg.norm(direction)
        rospy.loginfo("function mygetCamPosViewingDirection successful")
        return cam2tr, unitvec   
    except Exception as e:
        rospy.logerr(f"Error in mygetCamPosViewingDirection: {str(e)}")

def scaleObjectPCD(pcd,scaleFactor):
    try: 
        scaled = copy.deepcopy(pcd)
        scaled.scale(scaleFactor,center=[0,0,0])
        return scaled
    except Exception as e:
        rospy.logerr(f"Error in scaleObjectPCD: {str(e)}")

def orientCamBottom(pcd):
    try:
        campos = np.asarray(pcd.points)[-1,:] 
        if campos[2] > 0:
            Rtemp = o3d.geometry.get_rotation_matrix_from_xyz([np.pi,0,0])
            pcd.rotate(Rtemp)
        rospy.loginfo("function orientCamBottom successful")
        return pcd
    except Exception as e: 
        rospy.logerr(f"Error in orientCamBottom: {str(e)}")

def filterDgm(dgm,thresh):
    try:
        newdgm = []
        for i in range(len(dgm)):
            if dgm[i,1] - dgm[i,0] <= thresh:
                newdgm.append((dgm[i,0],dgm[i,1],0))
        rospy.loginfo("function filterDgm successful")
        
        return np.asarray(newdgm)
    except Exception as e:
        rospy.logerr(f"Error in filterDgm: {str(e)}")

# Function to log X and Y coordinates of contour points to a CSV file
def log_contour_to_csv(binimg, contours, file_path='contour_log.csv'):

    file_exists = os.path.isfile(file_path)
    
    with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        
        if not file_exists:
          
            writer.writerow(['Timestamp', 'Contour Index', 'Point Index', 'X', 'Y'])
        

        timestamp = rospy.get_time()  

        for contour_idx, contour in enumerate(contours):
            for point_idx, point in enumerate(contour):
                x, y = point[0]

                writer.writerow([timestamp, contour_idx, point_idx, x, y])
        
        rospy.loginfo(f"Logged contour points (X, Y) data to {file_path}")


def findContour(objidx,label, log_csv=False, file_path='/home/avaniab/contour_log.csv'):
    label = np.array(label)

    if label is not None:
        rospy.loginfo_once(f"Label shape:{np.shape(label)} ")

    else:
        rospy.logwarn("Label matrix is None.")
    rospy.logwarn(f"label is of type {type(label)}")

    binimg = np.expand_dims(255*np.where(label==objidx,1,0),axis=2).astype('uint8') #was axis 2 before; adds  new dimension at last axis along the depth of array
    
    if binimg is not None:
        rospy.loginfo(f"binary image has contents:{binimg.shape}, with pixles: {np.sum(binimg == 255)}")
    else:
        rospy.logwarn("binimg is empty") 


    contours,_ = cv2.findContours(binimg,mode=cv2.RETR_TREE,method=cv2.CHAIN_APPROX_NONE)
    rospy.logwarn(f"contours size: {len(contours)}")
    cnt = sorted(contours, key=cv2.contourArea)
    
        # added log contour to check working 
    if log_csv:
        log_contour_to_csv(binimg, contours, file_path)
        rospy.loginfo("function findContour successful")
    
    if not contours:
        rospy.logwarn("No contours found.")
        return None
    return np.squeeze(cnt[-1])[:,::-1] #because opencv flips rows and columns 




def checkOccludeeContour(contour,objidx,label,depth):
    boundary = np.zeros_like(label)
    rospy.logwarn(f"contour shape from check occludee: {contour.shape}")
    for i in range(np.shape(contour)[0]):
        x,y = contour[i,:]
        if x == np.shape(boundary)[0]-1:
            boundary[x,y] = 255 ## if object is cut
        elif y == np.shape(boundary)[1]-1:
            boundary[x,y] = 255
        else:
            if label[x-1,y] > 0 and label[x-1,y] != objidx:
                if depth[x-1,y] < depth[x,y]:
                    boundary[x,y] = 255
            if label[x+1,y] > 0 and label[x+1,y] != objidx:
                if depth[x+1,y] < depth[x,y]:
                    boundary[x,y] = 255
            if label[x,y-1] > 0 and label[x,y-1] != objidx:
                if depth[x,y-1] < depth[x,y]:
                    boundary[x,y] = 255
            if label[x,y+1] > 0 and label[x,y+1] != objidx:
                if depth[x,y+1] < depth[x,y]:
                    boundary[x,y] = 255

    return boundary

def visualize_object_pixels(label, color, objidx):

    mask = (label == objidx)  #bool array 

    print(f"Number of pixels for object {objidx}: {np.sum(mask)}")


    if np.sum(mask) == 0:
        print(f"No pixels found for object {objidx}.")

    object_color_image = np.zeros_like(color)  # start all  black

    object_color_image[mask] = color[mask]

    # Plot
    plt.figure(figsize=(20, 20))
    plt.imshow(object_color_image)
    plt.title(f"Visualization of Object {objidx} Pixels")
    plt.axis('off')
    plt.show()

"""
def export_colorpts_to_csv(colorpts, filename):
    rospy.loginfo_once(f"Exporting colorpts to {filename}")
    

    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        writer.writerow(['Red', 'Green', 'Blue'])
        
        for color in colorpts:
            writer.writerow(color)
    
    rospy.loginfo_once(f"colorpts successfully exported to {filename}")
"""


def getObjectOcclusionColors(objidx,label,color,export_colorpts_to_csv = False):
    try:
        rospy.logwarn(f"color: {color.shape} label: {label.shape}")
        label1d = np.reshape(label,(np.shape(label)[0]*np.shape(label)[1],))
        color1d = np.reshape(color,(np.shape(label)[0]*np.shape(label)[1],))/255
        idxes = np.where(label1d==objidx)
        colorpts = np.expand_dims(color1d[idxes],axis=1)
        objptscolor = np.concatenate((colorpts,np.zeros_like(colorpts),np.zeros_like(colorpts)),axis=1)
        rospy.logwarn(f"colorpts getObjectOcclusionColors: {colorpts.shape} objidx: {objidx}")
        
        # added log function to check its working 
        #if export_colorpts_to_csv: 
            #export_colorpts_to_csv(colorpts, filename="colorpts.csv")
        
        #added log to viz pixles in matplot 
        #visualize_object_pixels(label, color, objidx)

        return objptscolor
    except Exception as e:
        rospy.logerr(f"Error in filterDgm: {str(e)}")



def separateRedBlackPoints(pcdorg,objptcolors):
    try: 
        pts = np.asarray(pcdorg.points)
        return pts[np.where(objptcolors[:,0]== 1)], objptcolors[np.where(objptcolors[:,0]== 1)], pts[np.where(objptcolors[:,0]!= 1)], objptcolors[np.where(objptcolors[:,0]!= 1)] 
    except Exception as e:
        rospy.logerr(f"Error in separate RedBlackPoints: {str(e)}")
def checkNeedToFlipMinusCam(flatpcd):
    try: 
        flatpts = np.asarray(flatpcd.points)[:-2,:]
        xmin = np.min(flatpts[:,0])
        ymin = np.min(flatpts[:,1])
        zmin = np.min(flatpts[:,2])
        points = flatpts - [xmin,ymin,zmin]
        
        
        xmax = np.max(points[:,0])                
        threecolors = np.asarray(flatpcd.colors)[:-2,:]
        colors = threecolors[:,0]
        bins = np.arange(0,xmax+0.015,0.01)

        xes = copy.deepcopy(points[:,0])
        
        points[:,0] = bins[np.digitize(xes,bins,right=True)]
        rospy.loginfo(f"flatpts: {flatpts.shape}")
        rospy.logwarn(f"points: {points.shape}")
        rospy.logwarn(f"colors: {colors.shape}")
        rospy.logwarn(f"bins: {bins.shape}")
        redptsinlastbins = np.count_nonzero(colors[np.where(points[:,0]== bins[-1])])+np.count_nonzero(colors[np.where(points[:,0]== bins[-2])])
        redptsinfirstbins = np.count_nonzero(colors[np.where(points[:,0]== bins[0])])+np.count_nonzero(colors[np.where(points[:,0]== bins[1])])
        
        if redptsinfirstbins > redptsinlastbins:
            return True
        else:
            return False
    except Exception as e:
        rospy.logerr(f"Error in checkNeedToFlipMinusCam: {str(e)}")
    



# Main function for computing TOPS
def main_compute_TOPS(objpcd, cam1, cam2, depth_img, obj_id_in_segmentation, maxLayers, downsampling_and_outlier_removal=True):
    """
    Computes the TOPS descriptor for a given object in the scene.
    
    Parameters:
        objpcd (open3d.geometry.PointCloud): The object point cloud
        cam1 (np.array): The first camera position
        cam2 (np.array): The second camera position
        depth_img (np.array): The depth image (size 480x720)
        image_segmentation (sensor_msgs.Image): segmentation image from kimera; rgb colored
        obj_id_in_segmentation (int): The object ID to look for in segmentation 
        rgb_to_objid_mapping (dict): Mapping of RGB values to object IDs for segmentation 
        maxLayers (int): Maximum number of layers for the object.
        downsampling_and_outlier_removal (bool)
    
    """

    
    # Create label matrix from the segmentation image

    
    if label is None:
        rospy.logerr("Failed to create label matrix from segmentation image.")
        return None
    
    
    # Get the contour of the object of interest based on the object ID in the segmentation
    contour = findContour(obj_id_in_segmentation, label)

    
    
    # Check occlusion boundaries and update object point colors
    
    boundary = checkOccludeeContour(contour, obj_id_in_segmentation, label, depth_img)
    objptcolors = getObjectOcclusionColors(obj_id_in_segmentation, label, boundary)  # Set up colors for occlusion boundary


    if downsampling_and_outlier_removal:
        redpts, redcolors, blackpts, blackcolors = separateRedBlackPoints(objpcd, objptcolors)
                        
        pcdtodown = o3d.geometry.PointCloud()
        pcdtodown.points = o3d.utility.Vector3dVector(blackpts)
        downpcd = pcdtodown.voxel_down_sample(voxel_size=0.003)  ## Tune this based on use case
        
        # Add red points later, not here so they don't go away in outlier removal
        blackdownpcd, ind = downpcd.remove_radius_outlier(nb_points=220, radius=0.05)  ## Tune this based on use case
        
        downpcdpts = np.asarray(blackdownpcd.points)
        downpcdcolors = np.zeros_like(downpcdpts)
        downpcdptsplusred = np.concatenate((downpcdpts, redpts), axis=0)
        downpcdcolorsplusred = np.concatenate((downpcdcolors, redcolors), axis=0)
        objpcd = o3d.geometry.PointCloud()
        objpcd.points = o3d.utility.Vector3dVector(downpcdptsplusred)
        objpcd.colors = o3d.utility.Vector3dVector(downpcdcolorsplusred)

    
    #objpcd.colors = o3d.utility.Vector3dVector(objptcolors)

    rospy.logwarn(f"objpcd original size: {len(np.asarray(objpcd.points))}")
    rospy.logwarn(f"objpcd original color size : {len(np.asarray(objpcd.colors))}")

    pts = np.asarray(objpcd.points)
    pts = np.concatenate((pts, cam1, cam2), axis=0)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    
    
    colors = np.asarray(objpcd.colors)
    camcol = np.expand_dims(np.asarray([0, 0, 0]), axis=0)
    colors = np.concatenate((colors, camcol, camcol), axis=0)

    rospy.logwarn(f"points: {pts.shape} with colors: {colors.shape}")   

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(colors)



    rotatedpcd, extent = rotateToFlatForLayering(pcd)
    scaledpcd = scaleObjectPCD(rotatedpcd, 2.5)  # Scaling will scale cam positions appropriately   

    needtoflip = checkNeedToFlipMinusCam(scaledpcd)
    if needtoflip:
        Rflip = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, np.pi])
        scaledpcd.rotate(Rflip)
        rospy.loginfo("occluded")
    else:
        rospy.loginfo("skipped filp")
        donothing = 1
       
    trpcd = trXMinusCam(trYMinusCam(trZMinusCam(scaledpcd)))
    rotatedpcd = orientCamBottom(trpcd)
    
    R45 = o3d.geometry.get_rotation_matrix_from_xyz([0, -np.pi/4, 0])
    rotatedpcd.rotate(R45)    
    
    finalpcd = trXMinusCam(trYMinusCam(trZMinusCam(rotatedpcd)))
    
    pcdpts = np.asarray(finalpcd.points)[:-2, :]
    rounded = roundinXYZ(pcdpts)
    zs = getZs(rounded)
    
    pis = {}
    for key, value in zs.items():
        layer = getLayer(rounded, zs, key)
        rospy.loginfo(f"layer shape input to compute PDB: {layer.shape}")
        dgm = filterDgm(computePDBinningNo2DTranslation(layer), 0.75)
        #rospy.logwarn(f"dgm: {dgm}")
        
        img = pimgr.transform([dgm[:, 0:2]]) 
        #rospy.logwarn(f"img: {img}")
        pis[key] = img[0]
    #ospy.logwarn(f"{pis}")
    
    tops_descriptor = getFeatureNewPad(pis, maxLayers)

    rospy.logwarn(f"main_compute_TOPS this is the tops descirtpro shap: {tops_descriptor.shape}")
    
    return tops_descriptor

if __name__ == '__main__':
    listener() 