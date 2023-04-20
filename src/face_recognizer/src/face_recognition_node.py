#!/usr/bin/env python3
# nodo identificador

from logging import shutdown
#import string
from std_msgs.msg import String
#from matplotlib import image
from sensor_msgs.msg import Image
import numpy as np
import imutils
import pickle
import cv2
import sys
import rospy 
from cv_bridge import CvBridge, CvBridgeError

from face_recognizer.srv import command_face_recognition
from face_recognizer.srv import command_face_detection

class Face_recognition_node:
    def __init__(self) -> None:
        # node parameters
        self.sub = rospy.Subscriber('/face_detector/face',Image,self.face_detected_callback,queue_size=1)
        self.pub_nombre = rospy.Publisher('/face_recognizer/name',String,queue_size=10)
        self.pub_cara = rospy.Publisher('/face_recognizer/recognized_face', Image,queue_size=10)
        self.rate = rospy.Rate(3)
        self.bridge = CvBridge()

        # local operation parameters
        self.crop_face_ready = False
        self.detected_face_ready = False
        self.verbose = True
        self.active = True
        self.muestra = 10
        self.fidelidad = 7
        self.count = 0
        self.name_count1 =0
        self.name_count2 = 0
        self.name_count3 = 0
        self.name1 = "null"
        self.name2 = "null"
        self.name3 = "null"

        # load our serialized face embedding model from disk
        print("[INFO] loading face recognizer...")

        self.embbedModelPath = rospy.get_param('~embbedModelPath')
        #embbedModelPath = "../../data/openface.nn4.small2.v1.t7"
        embedder = cv2.dnn.readNetFromTorch(self.embbedModelPath) # This model is Torch-based and is responsible for extracting facial embeddings via deep learning feature extraction.

        # load the actual face recognition model along with the label encoder

        self.recognizerPath = rospy.get_param('~recognizerPath')
        #recognizerPath = "../../data/train_output/recognizer.pickle"
        recognizer = pickle.loads(open(self.recognizerPath, "rb").read())

        self.lePath = rospy.get_param('~lePath')
        #lePath = "../../data/train_output/le.pickle"
        le = pickle.loads(open(self.lePath, "rb").read())

        #Service to start/stop recognition mode
        #self.srv = rospy.Service('~command_face_recognition', command_face_recognition, self.handle_command_face_recognition)

        print("[INFO face recognizer] parameters loaded")


        while rospy is not shutdown():

            if self.detected_face_ready:

                # construct a blob for the face ROI, then pass the blob
                # through our face embedding model to obtain the 128-d
                # quantification of the face

                
                # la variable face coge el valor del topic recibido por el nodo anterior
                face = self.detected_face

                faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255, (96, 96),(0, 0, 0), swapRB=True, crop=False)
                embedder.setInput(faceBlob)
                vec = embedder.forward()
                # perform classification to recognize the face
                preds = recognizer.predict_proba(vec)[0]
                j = np.argmax(preds)
                proba = preds[j]
                name = le.classes_[j]
                # print(name)

                # draw the bounding box of the face along with the associated
                # probability
                text = "{}: {:.2f}%".format(name, proba * 100)
                #y = startY - 10 if startY - 10 > 10 else startY + 10
                #cv2.rectangle(image, (startX, startY), (endX, endY),(0, 0, 255), 2)
                cv2.putText(face, text, (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)


                ### contar 10 reconocimientos, si algún nombre supera el 70% lo publica
                
                if self.count < self.muestra:
                    
                    if self.name1 == "null":
                        self.name1 = name
                        self.name_count1 +=1
                        if self.verbose:
                            rospy.loginfo("[face recognizer node] name1 is: " + self.name1 + " and name_count1 is: " + str(self.name_count1))
                        self.count +=1
                        if self.verbose:
                           rospy.loginfo("[face recognizer node] number of recognitions is: " + str(self.count))
                    elif self.name1 != "null":
                        if name == self.name1:
                            self.name_count1 +=1
                            if self.verbose:
                                rospy.loginfo("[face recognizer node] name1 is: " + self.name1 + " and name_count1 is: " + str(self.name_count1))
                            self.count +=1
                            if self.verbose:
                                rospy.loginfo("[face recognizer node] number of recognitions is: " + str(self.count))
                        
                        elif self.name2 == "null":
                            self.name2 = name
                            self.name_count2 +=1
                            
                            if self.verbose:
                                rospy.loginfo("[face recognizer node] name2 is: " + self.name2 + " and name_count2 is: " + str(self.name_count2))
                            self.count +=1
                            if self.verbose:
                                rospy.loginfo("[face recognizer node] number of recognitions is: " + str(self.count))
                        
                        elif self.name2 != "null":
                            if name == self.name2:
                              self.name_count2 +=1  

                              if self.verbose:
                                rospy.loginfo("[face recognizer node] name2 is: " + self.name2 + "name_self.count2 is: " + str(self.name_count2))
                                self.count +=1

                                if self.verbose:
                                    rospy.loginfo("[face recognizer node] number of recognitions is: " + str(self.count))

                            elif self.name3 == "null":
                                self.name3 = name
                                self.name_count3 +=1
                                self.count +=1

                                if self.verbose:
                                    rospy.loginfo("[face recognizer node] self.name3 is: " + self.name3 + "and self.name_self.count3 is: " + str(self.name_count3))
                            
                            elif self.name3 != "null":

                                if name == self.name3:
                                    self.name_count3 +=1
                                    self.count +=1

                                    if self.verbose:
                                        rospy.loginfo("[face recognizer node] self.name3 is: " + self.name3 + "and self.name_self.count3 is: " + str(self.name_count3))

                elif self.count ==  self.muestra:
                    if self.name_count1 >= self.fidelidad:
                        if self.verbose:
                          rospy.loginfo("[face recognizer node]" + str(self.name_count1) + "is > " + str(self.fidelidad))  
                        self.publish_output(self.name1,face)
                        if self.verbose:
                            rospy.loginfo("[face recognizer node] Publicando cara...")
                    elif self.name_count2 >= self.fidelidad:
                        if self.verbose:
                          rospy.loginfo("[face recognizer node]" + str(self.name_count2) + "is > " + str(self.fidelidad))
                        self.publish_output(self.name2,face)
                        if self.verbose:
                            rospy.loginfo("[face recognizer node] Publicando cara...")
                    elif self.name_count2 >= self.fidelidad:
                        if self.verbose:
                          rospy.loginfo("[face recognizer node]" + str(self.name_count2) + "is > " + str(self.fidelidad))
                        self.publish_output(self.name3,face)
                        if self.verbose:
                            rospy.loginfo("[face recognizer node] Publicando cara...")
                        
                    self.count = 0
                    self.name_count1 = 0
                    self.name_count2 = 0
                    self.name_count3 = 0
                    self.name1 = "null"
                    self.name2 = "null"
                    self.name3 = "null"   



                
                
                    # publish the output image and the recognized face
                    #face_frame = self.bridge.cv2_to_imgmsg(face)
                    #self.pub_cara.publish(face_frame)
                    #self.pub_nombre.publish(String(name))

                    # reset flag and deactivate this node
                    # self.detected_face_ready = False
                    # self.active = False
                    
                
                    # deactivate face detector calling to its service
                    if self.verbose:
                        rospy.loginfo("[face_recognition_node] Deactivating face detector node")
                    
                    rospy.wait_for_service('/face_detector/command_face_detection')
                    try:
                        face_command = rospy.ServiceProxy('/face_detector/command_face_detection', command_face_detection)
                        face_command("off")
                        if self.verbose:
                            rospy.loginfo("[face_recognition_node] ... done")
                    except rospy.ServiceException as e:
                        print("Service call failed: %s"%e)
                

            # sleep until frame rate is achieved
            self.rate.sleep()

    # leer el topic de face_detection_node
    def face_detected_callback(self,data):
            if not self.active:
                return
            
            if self.detected_face_ready:
                return
            
            try:
                if self.verbose:
                    rospy.loginfo("[face_recognition_node] Received image from topic")
                
                self.detected_face = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
                self.detected_face_ready = True
            except CvBridgeError as e:
                if self.verbose:
                    print(e)

    def handle_command_face_recognition(self,req):
		# Check whether we start the node or we stop it
        if req.task_command == "on":
            self.active = True
            if self.verbose:
               rospy.loginfo("[face_recognition_node] Received command: ON")
            
            return True
        elif req.task_command == "off":
            self.active = False
            if self.verbose:
                rospy.loginfo("[face_recognition_node] Received command: OFF")
                
            return True
        else:
            if self.verbose:
                rospy.loginfo("[face_recognition_node] Received unknown command")
            
            return False

    def publish_output(self,name, face):
        # publish the output image and the recognized face
                face_frame = self.bridge.cv2_to_imgmsg(face)
                self.pub_cara.publish(face_frame)
                self.pub_nombre.publish(String(name))

                # reset flag and deactivate this node
                self.detected_face_ready = False
                # self.active = False



def main(args):
    rospy.init_node('face_recognizer_node', anonymous=True)
    node = Face_recognition_node ()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")


if __name__=='__main__':
    main(sys.argv)


