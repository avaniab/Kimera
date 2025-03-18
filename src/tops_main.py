#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: Ekta Samani
"""

import cv2,argparse
import numpy as np
import open3d as o3d
import fnmatch,os
import copy 
import pickle
from persim import PersistenceImager
from scipy.stats import iqr
import yaml
import shutil


pimgr = PersistenceImager()
pimgr.birth_range = (0,0.75)
pimgr.pers_range = (0,0.75)
pimgr.kernel_params = {'sigma': 0.00025}
pimgr.pixel_size = 0.025

def rotateToFlatForLayering(pcd):
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
    
    return pcd,extent

def get2dboundingboxXYEfficient(points):
    final = my_scatter_plot_xy(points)
    final = final.astype(np.uint8) 
    kernel = np.ones((11, 11), np.uint8)
    final = cv2.erode(final, kernel, cv2.BORDER_REFLECT) 

    
    _,thresh = cv2.threshold(final,127,255,cv2.THRESH_BINARY_INV)

    cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cnt = sorted(cnts, key=cv2.contourArea)

    rect = cv2.minAreaRect(cnt[-1]) 
    (x,y),(w,h), a = rect # a - angle
    
    return w,h,a

def get2dboundingboxYZEfficient(points):
    final = my_scatter_plot_yz(points)
    final = final.astype(np.uint8) 
    kernel = np.ones((11, 11), np.uint8)

    final = cv2.erode(final, kernel, cv2.BORDER_REFLECT) 
    
    
    _,thresh = cv2.threshold(final,127,255,cv2.THRESH_BINARY_INV)
    

    cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cnt = sorted(cnts, key=cv2.contourArea)

    rect = cv2.minAreaRect(cnt[-1]) 

    (x,y),(w,h), a = rect # a - angle

    return w,h,a

def my_scatter_plot_xy(points):
    nx,ny = (224,224)
    xmin = np.min(points[:,0])
    ymin = np.min(points[:,1])
    img = 255*np.ones((nx,ny))
    
    x = np.linspace(xmin - 0.1,xmin+0.9,nx)
    y = np.linspace(ymin-0.1,ymin+0.9,ny)
    xbins = np.digitize(points[:,0],x)
    ybins = len(y) - np.digitize(points[:,1],y)
    
    for i in range(len(points)):

        img[ybins[i],xbins[i]] = 0
    
    return img

def my_scatter_plot_yz(points):
    ny,nz = (224,224)
    ymin = np.min(points[:,1])
    zmin = np.min(points[:,2])
    img = 255*np.ones((ny,nz))
    
    y = np.linspace(ymin - 0.1,ymin+0.9,ny)
    z = np.linspace(zmin-0.1,zmin+0.9,nz)
    ybins = np.digitize(points[:,1],y)
    zbins = len(z) - np.digitize(points[:,2],z)
    
    for i in range(len(points)):
        img[zbins[i],ybins[i]] = 0
    
    return img   


def roundinXYZ(pts):
    pcdpts = copy.deepcopy(pts)
    pcdpts[:,0] = np.round(pcdpts[:,0],2)
    pcdpts[:,1] = np.round(pcdpts[:,1],2)
    pcdpts[:,2] = np.round(pcdpts[:,2],1)
    return pcdpts

def trZMinusCam(pcd):
    pts = np.asarray(pcd.points)[:-2,:]
    pcd.translate([0,0,-np.min(pts[:,2])])
    return pcd

def trYMinusCam(pcd):
    pts = np.asarray(pcd.points)[:-2,:]
    pcd.translate([0,-np.min(pts[:,1]),0])
    return pcd

def trXMinusCam(pcd):
    pts = np.asarray(pcd.points)[:-2,:]
    pcd.translate([-np.min(pts[:,0]),0,0])
    return pcd

def checkNeedToFlipMinusCam(flatpcd):
    ## TBD ##
    is_occluded_on_the_wrong_end  = False
    return is_occluded_on_the_wrong_end

def getZs(pcdpts):
    zlist = sorted(np.unique(pcdpts[:,2]))
    zs = {}
    for idx,num in enumerate(zlist):
        zs[idx] = num
    return zs

def getLayer(pcdpts,zs,layeridx):
    return pcdpts[np.where(pcdpts[:,2] == zs[layeridx])]

def computePDBinningNo2DTranslation(pcd):
    #binning
    bins = np.arange(0,0.775,0.025)
    xes = copy.deepcopy(pcd[:,0])
    pcd[:,0] = bins[np.digitize(xes,bins,right=True)]
    xesnew = np.unique(pcd[:,0])
    dgm = []
    for idx,x in enumerate(xesnew):
        ymax = np.max(pcd[np.where(pcd[:,0] == x)][:,1])
        ymin = np.min(pcd[np.where(pcd[:,0] == x)][:,1])

        dgm.append((x,x+ymax-ymin,0))

    return np.asarray(dgm)

def getFeatureNewPad(pis,maxLayers):
    features = np.reshape(pis[0],(1,1024))
    for l in range(1,maxLayers):
        if l in pis:
            features = np.concatenate((features, np.reshape(pis[l],(1,1024))),axis=1)
        else:
            features = np.concatenate((features, np.ones((1,1024))),axis=1)

    return features

def mygetCamPosViewingDirection(pcd):
    points = np.asarray(pcd.points)
    cam2tr = points[-1,:]
    cam1tr = points[-2,:]
    direction = cam2tr-cam1tr
    unitvec = direction/np.linalg.norm(direction)
    return cam2tr, unitvec       


def scaleObjectPCD(pcd,scaleFactor):
    scaled = copy.deepcopy(pcd)
    scaled.scale(scaleFactor,center=[0,0,0])
    return scaled


def orientCamBottom(pcd):
    campos = np.asarray(pcd.points)[-1,:] 
    if campos[2] > 0:
        Rtemp = o3d.geometry.get_rotation_matrix_from_xyz([np.pi,0,0])
        pcd.rotate(Rtemp)
    return pcd
        
def filterDgm(dgm,thresh):
    newdgm = []
    for i in range(len(dgm)):
        if dgm[i,1] - dgm[i,0] <= thresh:
            newdgm.append((dgm[i,0],dgm[i,1],0))
    return np.asarray(newdgm)

def findContour(objidx,label):
    binimg = np.expand_dims(255*np.where(label==objidx,1,0),axis=2).astype('uint8')
    contours,_ = cv2.findContours(binimg,mode=cv2.RETR_TREE,method=cv2.CHAIN_APPROX_NONE)
    cnt = sorted(contours, key=cv2.contourArea)
    return np.squeeze(cnt[-1])[:,::-1] #because opencv flips rows and columns

def checkOccludeeContour(contour,objidx,label,depth):
    boundary = np.zeros_like(label)
    for i in range(np.shape(contour)[0]):
        x,y = contour[i,:]
        if x == 719:
            boundary[x,y] = 255 ## if object is cut
        elif y == 1279:
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

def getObjectOcclusionColors(objidx,label,color):
    label1d = np.reshape(label,(np.shape(label)[0]*np.shape(label)[1],))
    color1d = np.reshape(color,(np.shape(label)[0]*np.shape(label)[1],))/255
    idxes = np.where(label1d==objidx)
    colorpts = np.expand_dims(color1d[idxes],axis=1)
    objptscolor = np.concatenate((colorpts,np.zeros_like(colorpts),np.zeros_like(colorpts)),axis=1)
    return objptscolor

def separateRedBlackPoints(pcdorg,objptcolors):
    pts = np.asarray(pcdorg.points)
    return pts[np.where(objptcolors[:,0]== 1)], objptcolors[np.where(objptcolors[:,0]== 1)], pts[np.where(objptcolors[:,0]!= 1)], objptcolors[np.where(objptcolors[:,0]!= 1)] 

def checkNeedToFlipMinusCam(flatpcd):
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

    redptsinlastbins = np.count_nonzero(colors[np.where(points[:,0]== bins[-1])])+np.count_nonzero(colors[np.where(points[:,0]== bins[-2])])
    redptsinfirstbins = np.count_nonzero(colors[np.where(points[:,0]== bins[0])])+np.count_nonzero(colors[np.where(points[:,0]== bins[1])])
    
    if redptsinfirstbins > redptsinlastbins:
        return True
    else:
        return False


def main_compute_TOPS(depth_image, image_segmentation, obj_id_in_segmentation, objpcd,cam1,cam2, maxLayers = 7, downsampling_and_outlier_removal = False):
    ### depth_image is the 480*720 sized depth image of the scene

    ### image_segmentation is the 480*720 sized segmentation map with IDs assigned only to objects of interest; everything else is background 

    ### objpcd is an open3d point cloud of the object
    
    ### cam2 is camera position 
    # e.g., cam2 = np.expand_dims(np.asarray([0,0,0]),axis=0)
    ### cam1 is a point selected such that cam2 - cam1 gives the viewing direction, 
    
    ### viewing direction is the direction pointing from camera toward the object/scene
    # e.g., cam1 = np.expand_dims(np.asarray([0,0,-0.1]),axis=0) 
    
    ### max layers is 7 for the objects we are considering

    contour = findContour(obj_id_in_segmentation, image_segmentation) #find the contour of the object of interest
    boundary = checkOccludeeContour(contour,idx,label,depthnp)
    objptcolors = getObjectOcclusionColors(badidxes,idx,label,boundary) #set up such that red color points in object pcd are occlusion boundary
    objpcd.colors = o3d.utility.Vector3dVector(objptcolors)

    if downsampling_and_outlier_removal:
        redpts, redcolors, blackpts,blackcolors = separateRedBlackPoints(objpcd, objptcolors)
                        
        pcdtodown = o3d.geometry.PointCloud()
        pcdtodown.points = o3d.utility.Vector3dVector(blackpts)
        downpcd = pcdtodown.voxel_down_sample(voxel_size=0.003)  ##tune this based on use case
        
        ##add red points later, not here so that they don't go away in outlier removal
        
        blackdownpcd,ind = downpcd.remove_radius_outlier(nb_points=220,radius=0.05) ##tune this based on use case
        
        downpcdpts = np.asarray(blackdownpcd.points)
        downpcdcolors = np.zeros_like(downpcdpts)
        downpcdptsplusred = np.concatenate((downpcdpts,redpts),axis=0)
        downpcdcolorsplusred = np.concatenate((downpcdcolors,redcolors),axis=0)
        objpcd = o3d.geometry.PointCloud()
        objpcd.points = o3d.utility.Vector3dVector(downpcdptsplusred)
        objpcd.colors = o3d.utility.Vector3dVector(downpcdcolorsplusred)   

    pts = np.asarray(objpcd.points)
    pts = np.concatenate((objpcd,cam1,cam2),axis=0)

    ##do this to maintain red color of occluded points
    colors = np.asarray(objpcd.colors)
    camcol = np.expand_dims(np.asarray([0,0,0]),axis=0)
    colors = np.concatenate((colors,camcol,camcol),axis=0) 
                        
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(colors)


    rotatedpcd,extent = rotateToFlatForLayering(pcd) 
    scaledpcd = scaleObjectPCD(rotatedpcd,2.5) #scaling will scale cam positions appropriately   
    
    needtoflip = checkNeedToFlipMinusCam(scaledpcd)
    if needtoflip:
        Rflip = o3d.geometry.get_rotation_matrix_from_xyz([0,0,np.pi])
        scaledpcd.rotate(Rflip)
    else:
        donothing=1
    
    trpcd = trXMinusCam(trYMinusCam(trZMinusCam(scaledpcd)))
    rotatedpcd = orientCamBottom(trpcd)
    
    R45 = o3d.geometry.get_rotation_matrix_from_xyz([0,-np.pi/4,0])
    rotatedpcd.rotate(R45)    
    
    
    finalpcd = trXMinusCam(trYMinusCam(trZMinusCam(rotatedpcd)))
    
    pcdpts = np.asarray(finalpcd.points)[:-2,:]
    rounded = roundinXYZ(pcdpts)
    zs = getZs(rounded)
    
    
    pis = {}
    for key,value in zs.items():
        layer = getLayer(rounded,zs,key)
        dgm = filterDgm(computePDBinningNo2DTranslation(layer),0.75)
        img = pimgr.transform([dgm[:,0:2]]) 
        pis[key] = img[0]
    
    
    tops_descriptor = getFeatureNewPad(pis,maxLayers)
    
    return tops_descriptor