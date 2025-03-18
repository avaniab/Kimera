#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: smartslab
"""

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray


from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, BatchNormalization
from std_msgs.msg import String
#from keras.layers.core import Dense, Dropout, Activation
#from keras.layers.normalization import BatchNormalization
# fails but should work

tops_descriptor = np.zeros((1, 7168), dtype=np.float64)
#descriptor_recieved = False
prediction_pub = None
descriptor_object_id = None

def tops_descriptor_callback(msg):
    global tops_descriptor
    rospy.loginfo_once(f"got tops descriptor{tops_descriptor}")
    expected_shape =  (1, 7168) #parameterize later works for 7 max layer slices
    tops_descriptor = np.array(msg.data, dtype=np.float64).reshape(expected_shape)
    #tops_descriptor = np.array(msg.data, dtype=np.float64)


def classifier_mlp_softmax(n_classes=10,objlayers= 7):
    classifier = Sequential()
    classifier.add(Dense(512, input_shape = (1024*objlayers,)))
    classifier.add(BatchNormalization())
    classifier.add(Activation('relu'))
    classifier.add(Dropout(0.2))

    classifier.add(Dense(256))
    classifier.add(BatchNormalization())
    classifier.add(Activation('relu'))
    classifier.add(Dropout(0.2))
    
    classifier.add(Dense(128))
    classifier.add(BatchNormalization())
    classifier.add(Activation('relu'))
    classifier.add(Dropout(0.2))

    classifier.add(Dense(64))
    classifier.add(BatchNormalization())
    classifier.add(Activation('relu'))
    classifier.add(Dropout(0.2))
    
    classifier.add(Dense(n_classes))
    classifier.add(BatchNormalization())
    classifier.add(Activation('softmax'))
    
    return classifier

def prediction(tops_descriptor, model, maxLayers = 7):
    #global descriptor_recieved
    if tops_descriptor.size == 0:
        rospy.logwarn("nothing recieved yet")
        #descriptor_recieved = False
        return None
    
    else:
        #descriptor_recieved = True
        rospy.loginfo(f"tops has content size {tops_descriptor.size}")
        feature= np.nan_to_num(tops_descriptor)
        rospy.loginfo(f"feature size{feature.shape}")
        rospy.loginfo(f"model{model}")
        #tensor flow v 2.6 got rid fo .predict classes 
        pred = model.predict(feature[:,:1024*(maxLayers+1)])[0]

        #rospy.loginfo('Predicted object is ', object_list[pred], ' and the corresponding class ID is ', pred)
        rospy.loginfo(f"pred geneeated: {pred}")
        predicted_class = object_list[np.argmax(pred)]

        rospy.loginfo(f"predicted class {predicted_class}")
        prediction_publisher(predicted_class)
        return pred



object_list = ['010_potted_meat_can', '025_mug', '005_tomato_soup_can', '006_mustard_bottle', '024_bowl',  '021_bleach_cleanser',  '019_pitcher_base', '029_plate', '009_gelatin_box_new']
 
model = classifier_mlp_softmax(9)
model.load_weights('/home/avaniab/catkin_ws/src/semantics_tops_integrator/tops_files/models_for_tops_alone_food_kitchen/2021/mlp_all_7layers.hdf5') ##path to the model file




def object_id_callback(msg):
    global descriptor_object_id
    descriptor_object_id = int(msg.data)
    rospy.loginfo_once(descriptor_object_id)

def listener():
    rospy.init_node('prediction', anonymous=True)
    rospy.Subscriber("/tops_descriptor", Float64MultiArray, tops_descriptor_callback)
    rospy.Subscriber("/object_id_processed", String, object_id_callback)
    while tops_descriptor.size == 0:
        rospy.loginfo("Waiting for tops descriptor...")
        rospy.sleep(0.5)
    ## use the main_compute_TOPS function to generate the top_descriptor used below.
    pred = prediction(tops_descriptor,model)
    
    rospy.spin()

def prediction_publisher(predicted_class):
    tops_descriptor_pub = rospy.Publisher('/prediction', String, queue_size=10)
    rate = rospy.Rate(10)
    # loop
    while not rospy.is_shutdown():
        msg = String()
        if descriptor_object_id:
            msg.data = f"{predicted_class}:{descriptor_object_id}"
            rospy.loginfo_once(f" published information {msg.data}")
        
        tops_descriptor_pub.publish(msg)
        
        rate.sleep()
if __name__ == '__main__':
    listener()