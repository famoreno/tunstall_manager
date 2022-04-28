# nodo identificador

from logging import shutdown
import string
from matplotlib import image
import numpy as np
import imutils
import pickle
import cv2
import rospy 
from cv_bridge import CvBridge, CvBridgeError

class face_recognizer_node:
    def __init__(self) -> None:
        self.sub = rospy.Subscriber('caras recortadas',image,self.callback)
        self.sub = rospy.Subscriber('lista de nombres',string,self.callback)
        self.pub_nombre = rospy.Publisher('nombre',string,queue_size=10)
        self.pub_cara = rospy.Publisher('face', image,queue_size=10)
        self.rate = rospy.Rate(1)
        self.bridge = CvBridge()


        while rospy is not shutdown():


        # construct a blob for the face ROI, then pass the blob
		# through our face embedding model to obtain the 128-d
		# quantification of the face

        # TODO #leer el topic de face_detection_node

        #face= 

        #
		faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255, (96, 96),
			(0, 0, 0), swapRB=True, crop=False)
		embedder.setInput(faceBlob)
		vec = embedder.forward()
		# perform classification to recognize the face
		preds = recognizer.predict_proba(vec)[0]
		j = np.argmax(preds)
		proba = preds[j]
		name = le.classes_[j]

        # draw the bounding box of the face along with the associated
		# probability
		text = "{}: {:.2f}%".format(name, proba * 100)
		y = startY - 10 if startY - 10 > 10 else startY + 10
		cv2.rectangle(image, (startX, startY), (endX, endY),
			(0, 0, 255), 2)
		cv2.putText(image, text, (startX, y),
			cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

        # show the output image
        cv2.imshow("Image", image)
        cv2.waitKey(0)

        # Pronunciar el nombre en voz alta


        #

def main(args):
    rospy.init_node('face_recognizer_node', anonymous=True)
    node =face_recognizer_node ()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")


if __name__=='__main__':
    main(sys.argv)


