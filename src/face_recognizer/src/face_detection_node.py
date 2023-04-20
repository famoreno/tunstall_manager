#!/usr/bin/env python3
# import the necessary packages
from logging import shutdown
from tabnanny import verbose

from matplotlib import image
import rospy 
import numpy as np
# import argparse
# import time
import sys
import cv2
import dlib

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import imutils
from imutils.face_utils.helpers import FACIAL_LANDMARKS_IDXS
from imutils.face_utils.helpers import shape_to_np
from imutils.face_utils import FaceAligner

from face_recognizer.srv import command_face_detection

class face_detector_node:
    def __init__(self) -> None:
        # node parameters
        self.sub = rospy.Subscriber('/usb_cam/image_raw',Image,self.callback)
        self.pub_cara = rospy.Publisher('/face_detector/face', Image,queue_size=1)
        self.rate = rospy.Rate(3)
        self.bridge = CvBridge()
        self.confidence = 0.9
        
        # operation parameters
        self.new_image_ready = False
        self.active = False
        self.verbose = True

        # face detector
        self.protoPath = rospy.get_param('~protoPath')
        self.modelPath = rospy.get_param('~modelPath')
        self.detector = cv2.dnn.readNetFromCaffe(self.protoPath, self.modelPath)

        # face landmarks predictor 
        self.predictorPath = rospy.get_param('~shapePredictor')
        self.predictor = dlib.shape_predictor(self.predictorPath)
        self.fa = FaceAligner(self.predictor)

        #Service to start/stop face detection
        self.srv = rospy.Service('~command_face_detection', command_face_detection, self.handle_command_face_detection)

        while not rospy.is_shutdown():
            
            # Check if 'face detection' is active
            if self.active:

                if not self.new_image_ready:
                    continue

                # grab the frame from the threaded video stream and resize it
                # to have a maximum width of 800 pixels
                frame0 = self.new_image
                frame = imutils.resize(frame0, width=800)

                # grab the frame dimensions and convert it to a blob
                (h, w) = frame.shape[:2]
                blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,(300, 300), (104.0, 177.0, 123.0))

                # pass the blob through the network and obtain the detections and predictions
                self.detector.setInput(blob)
                detections = self.detector.forward()
                count = 0  

                for i in range(0, detections.shape[2]):
                    # extract the confidence (i.e., probability) associated with the prediction
                    confidence = detections[0, 0, i, 2]

                    # filter out weak detections by ensuring the `confidence` is
                    # greater than the minimum confidence
                    if verbose:
                        print(confidence * 100)
                    if confidence < self.confidence:
                        continue
                    count += 1

                    # compute the (x, y)-coordinates of the bounding box for the object
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    # recortar la imagen de la cara
                    crop_frame = frame[startY:endY,startX:endX]

                    # image = imutils.resize(frame0, width=800)
                    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    
                    # Eye alignment
                    # this_bb = dlib.rectangle(startX,startY,endX,endY)
                    # face_aligned = self.fa.align(image, gray, this_bb)

                    # Volver a cambiar el formato a imagen
                    (h,w) = crop_frame.shape[0:2]
                    if h > 0 and w > 0:
                        face_frame = self.bridge.cv2_to_imgmsg(crop_frame)
                        # face_frame = self.bridge.cv2_to_imgmsg(face_aligned)
                        self.pub_cara.publish(face_frame)

                # clear image ready flag to process more
                self.new_image_ready = False

            # sleep until frame rate is achieved
            self.rate.sleep()

    def callback(self,data):
        if self.new_image_ready:
            return
        try:
            self.new_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.new_image_ready = True
        except CvBridgeError as e:
            print(e)

    def handle_command_face_detection(self,req):
               
		# Check whether we start the node or we stop it
        if req.task_command == "on":
            self.active = True
            if self.verbose:
                rospy.loginfo("[face_detection_node] Received command: ON")
            
            return True
        elif req.task_command == "off":
            self.active = False
            self.new_image = None
            self.new_image_ready = False
            
            if self.verbose:
                rospy.loginfo("[face_detection_node] Received command: OFF")
                
            return True
        else:
            if self.verbose:
                    rospy.loginfo("[face_detection_node] Received unknown command:", req.task_command)
            return False

def main(args):
    rospy.init_node('face_detector_node', anonymous=True)
    node = face_detector_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

if __name__=='__main__':
    main(sys.argv)
