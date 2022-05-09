#!/usr/bin/env python3
# import the necessary packages
from logging import shutdown

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

class face_detector_node:
    def __init__(self) -> None:
        self.sub = rospy.Subscriber('/usb_cam/image_raw',Image,self.callback)
        self.pub_cara = rospy.Publisher('face', Image,queue_size=10)
        self.rate = rospy.Rate(1)
        self.bridge = CvBridge()
        self.confidence = 0.9
        self.new_image_ready = False

        # face detector
        self.protoPath = rospy.get_param('~protoPath')
        self.modelPath = rospy.get_param('~modelPath')
        self.detector = cv2.dnn.readNetFromCaffe(self.protoPath, self.modelPath)

        # face landmarks predictor 
        self.predictorPath = rospy.get_param('~shapePredictor')
        self.predictor = dlib.shape_predictor(self.predictorPath)
        self.fa = FaceAligner(self.predictor)

        while not rospy.is_shutdown():
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
                print(confidence * 100)
                if confidence < self.confidence:
                    continue
                count += 1

                # compute the (x, y)-coordinates of the bounding box for the object
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # recortar la imagen de la cara
                # crop_frame = frame[startY:endY,startX:endX]

                image = imutils.resize(frame0, width=800)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                
                # Eye alignment
                this_bb = dlib.rectangle(startX,startY,endX,endY)
                face_aligned = self.fa.align(image, gray, this_bb)

                # Volver a cambiar el formato a imagen
                # face_frame = self.bridge.cv2_to_imgmsg(crop_frame)
                face_frame = self.bridge.cv2_to_imgmsg(face_aligned)
                self.pub_cara.publish(face_frame)

            # clear image ready flag to process more
            self.new_image_ready = False

    def callback(self,data):
        if self.new_image_ready:
            return
        try:
            self.new_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.new_image_ready = True
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('face_detector_node', anonymous=True)
    node = face_detector_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

if __name__=='__main__':
    main(sys.argv)