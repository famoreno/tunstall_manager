from imutils.video import VideoStream
import numpy as np
import argparse
import time # En el tutorial no aparece esta librería
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()

#ap.add_argument("-i", "--dataset", required=True,
#help="path to inputdirectory of faces + images") # ruta a la entrada del set de datos de las imágenes de caras

#ap.add_argument("-e", "--embeddings", required=True,
#	help="path to output serialized db of facial embeddings") #ruta de la salida de ambeddings. El código computa estos embeddings faciales y se serializaran al disco

#ap.add_argument("-d", "--detector", required=True,
	#help="path to OpenCV's deep learning face detector")

#ap.add_argument("-m", "--embedding-model", required=True,
#	help="path to OpenCV's deep learning face embedding model")

# Aquí los argumentos varían un poco entre el tutorial y el código de git, de momento los he puesto todos y ya vamos viendo si alguno es redundante

ap.add_argument("-p", "--prototxt", required=True,
    help="path to Caffe 'deploy' prototxt file") # Averiguar función del argumento

ap.add_argument("-m", "--model", required=True,
    help="path to Caffe pre-trained model") # Averiguar función del argumento

ap.add_argument("-c", "--confidence", type=float, default=0.5,
    help="minimum probability to filter weak detections") # Umbral de filtrado 
args = vars(ap.parse_args())

# load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])

#print("[INFO] loading face detector...")
#protoPath = os.path.sep.join([args["detector"], "deploy.prototxt"]) # se usa un detector Caffe basado en DL para detectar las caras
#modelPath = os.path.sep.join([args["detector"],
#	"res10_300x300_ssd_iter_140000.caffemodel"])
#detector = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

# este fragmento de código parece que es redundante, hacer averiguaciones de las diferencias 

# load our serialized face embedding model from disk
#print("[INFO] loading face recognizer...")
#embedder = cv2.dnn.readNetFromTorch(args["embedding_model"]) # modelo basado en Torch y responsable de de extraer los embedidos faciales mediante aprendizaje

#fragmento de código del tutorial, posiblemente haya que descomentarlo, pero primero tengo que examinar todo el código
 
# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)

# loop over the frames from the video stream
while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = vs.read()
    frame = imutils.resize(frame, width=400)
 
    # grab the frame dimensions and convert it to a blob
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,(300, 300), (104.0, 177.0, 123.0))
 
    # pass the blob through the network and obtain the detections and
    # predictions
    net.setInput(blob)
    detections = net.forward()
    count = 0    
    
    # loop over the detections
    for i in range(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with the
        # prediction
        confidence = detections[0, 0, i, 2]
        #print(confidence * 100)

 
        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence
        if confidence < args["confidence"]:
            continue
        count += 1 
        # compute the (x, y)-coordinates of the bounding box for the
        # object
        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
        (startX, startY, endX, endY) = box.astype("int")
 
        # draw the bounding box of the face along with the associated
        # probability
        text = "{:.2f}%".format(confidence * 100) + ", Count " + str(count)
        y = startY - 10 if startY - 10 > 10 else startY + 10
        cv2.rectangle(frame, (startX, startY), (endX, endY),(0, 255, 0), 2)
        cv2.putText(frame, text, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
        
    # show the output frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
 
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
 
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()