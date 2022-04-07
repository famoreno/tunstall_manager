#!/usr/bin/env python3
# import the necessary packages
from logging import shutdown

from matplotlib import image
import rospy 
from imutils.video import VideoStream
import numpy as np
import argparse
import time 
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from .helpers import FACIAL_LANDMARKS_IDXS
from .helpers import shape_to_np




class face_detector_node:
        def __init__(self) -> None:
                self.sub = rospy.Subscriber('/usb_cam/image_raw',Image,self.callback)
                self.pub_cara = rospy.Publisher('cara', Image,queue_size=10)
                self.rate = rospy.Rate(1)
                self.bridge = CvBridge()
                self.confidence = 0.9

                while not rospy.is_shutdown():
                    #if self.new_image_ready: Es posible que no cea necesario este comando
                    # grab the frame from the threaded video stream and resize it
                    # to have a maximum width of 400 pixels
                    frame = self.new_image
                    frame = imutils.resize(frame, width=400)

                    # grab the frame dimensions and convert it to a blob
                    (h, w) = frame.shape[:2]
                    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,(300, 300), (104.0, 177.0, 123.0))

                    # pass the blob through the network and obtain the detections and
                     # predictions
                    net.setInput(blob)
                    detections = net.forward()
                    count = 0  

                    for i in range(0, detections.shape[2]):
                        # extract the confidence (i.e., probability) associated with the
                        # prediction
                        confidence = detections[0, 0, i, 2]
                        #print(confidence * 100)

 
                        # filter out weak detections by ensuring the `confidence` is
                         # greater than the minimum confidence
                        if confidence < self.confidence:
                            continue
                        count += 1

                        # compute the (x, y)-coordinates of the bounding box for the object
                        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                        (startX, startY, endX, endY) = box.astype("int")

                        

                        # draw the bounding box of the face along with the associated probability
                        #text = "{:.2f}%".format(confidence * 100) + ", Count " + str(count)
                        #y = startY - 10 if startY - 10 > 10 else startY + 10
                        #cv2.rectangle(frame, (startX, startY), (endX, endY),(0, 255, 0), 2)
                        #cv2.putText(frame, text, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

                        #recortar la imagen de la cara
                        crop_frame = frame[startY:endY,startX:endX]
                        
                        
                        # ALINEAR OJOS
                        
                        
                        #   
                        # Volver a cambiar el formato a imagen
                        face_frame = self.bridge.cv2_to_imgmsg(crop_frame)
                        self.pub_cara.publish(face_frame)

                    # show the output frame
                    cv2.imshow("Frame", frame)
                    key = cv2.waitKey(1) & 0xFF

                    # if the `q` key was pressed, break from the loop
                    # if key == ord("q"):
                    # break
                    # Estes liíneas para salir al precionar la q imagino que deberían ir fuera

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
