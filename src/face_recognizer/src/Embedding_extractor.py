# import the necessary packages
from imutils import paths
import numpy as np
import argparse
import imutils
import pickle
import cv2
import os

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--dataset", required=True,
	help="path to input directory of faces + images") # ruta a la entrada del set de datos de las imágenes de caras
ap.add_argument("-e", "--embeddings", required=True,
	help="path to output serialized db of facial embeddings") #ruta de la salida de ambeddings. El código computa estos embeddings faciales y se serializaran al disco
ap.add_argument("-d", "--detector", required=True,
	help="path to OpenCV's deep learning face detector") #ruta al detector de caras basado en aprendizaje
ap.add_argument("-m", "--embedding-model", required=True,
	help="path to OpenCV's deep learning face embedding model") # Path to the OpenCV deep learning Torch embedding model. This model will allow us to extract a 128-D facial embedding vector.
ap.add_argument("-c", "--confidence", type=float, default=0.5,
	help="minimum probability to filter weak detections") #Optional threshold for filtering week face detections.
    args = vars(ap.parse_args())

# load our serialized face detector from disk
print("[INFO] loading face detector...")
protoPath = os.path.sep.join([args["detector"], "deploy.prototxt"]) #Loaded via Lines 26-29. We’re using a Caffe based DL face detector to localize faces in an image
modelPath = os.path.sep.join([args["detector"],
	"res10_300x300_ssd_iter_140000.caffemodel"])
detector = cv2.dnn.readNetFromCaffe(protoPath, modelPath)
# load our serialized face embedding model from disk
print("[INFO] loading face recognizer...")
embedder = cv2.dnn.readNetFromTorch(args["embedding_model"]) # This model is Torch-based and is responsible for extracting facial embeddings via deep learning feature extraction.


# grab the paths to the input images in our dataset
print("[INFO] quantifying faces...")
imagePaths = list(paths.list_images(args["dataset"])) # The imagePaths list, built on Line 37, contains the path to each image in the dataset. I’ve made this easy via my imutils function, paths.list_images.

# initialize our lists of extracted facial embeddings and  corresponding people names
knownEmbeddings = [] #Our embeddings and corresponding names will be held in two lists: knownEmbeddings and knownNames
knownNames = []

# initialize the total number of faces processed
total = 0 # We’ll also be keeping track of how many faces we’ve processed via a variable called total


# loop over the image paths
for (i, imagePath) in enumerate(imagePaths):
	# extract the person name from the image path
	print("[INFO] processing image {}/{}".format(i + 1,
		len(imagePaths)))
	name = imagePath.split(os.path.sep)[-2] # First, we extract the name of the person from the path

	# load the image, resize it to have a width of 600 pixels (while
	# maintaining the aspect ratio), and then grab the image
	# dimensions
	image = cv2.imread(imagePath)
	image = imutils.resize(image, width=600)
	(h, w) = image.shape[:2]

    # construct a blob from the image
    imageBlob = cv2.dnn.blobFromImage(
	cv2.resize(image, (300, 300)), 1.0, (300, 300),
	(104.0, 177.0, 123.0), swapRB=False, crop=False) # we construct a blob

	# apply OpenCV's deep learning-based face detector to localize
	 # faces in the input image
	 detector.setInput(imageBlob)        #From there we detect faces in the image by passing the imageBlobthrough the detector network
	 detections = detector.forward() 

    # ensure at least one face was found
	if len(detections) > 0:     # Assuming we have at least one detection, we’ll proceed into the body of the if-statement
	    # we're making the assumption that each image has only ONE
	    # face, so find the bounding box with the largest probability
	    i = np.argmax(detections[0, 0, :, 2])
	    confidence = detections[0, 0, i, 2]

	    # ensure that the detection with the largest probability also
	    # means our minimum probability test (thus helping filter out
	    # weak detections)
	    if confidence > args["confidence"]:
			# compute the (x, y)-coordinates of the bounding box for
			# the face
			box = detections[0, 0, i, 3:7] * np.array([w, h, w, h]) # Assuming we’ve met that threshold, we extract the face ROI and grab/check dimensions to make sure the face ROI is sufficiently large
			(startX, startY, endX, endY) = box.astype("int")

			# extract the face ROI and grab the ROI dimensions
			face = image[startY:endY, startX:endX]
			(fH, fW) = face.shape[:2]

			# ensure the face width and height are sufficiently large
			if fW < 20 or fH < 20:
				continue

            # construct a blob for the face ROI, then pass the blob
			# through our face embedding model to obtain the 128-d
			# quantification of the face
			faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255,
				(96, 96), (0, 0, 0), swapRB=True, crop=False) # We construct another blob, this time from the face ROI (not the whole image as we did before)
			embedder.setInput(faceBlob) #Subsequently, we pass the faceBlob through the embedder CNN This generates a 128-D vector (vec) which describes the face. We’ll leverage this data to recognize new faces via machine learning.
			vec = embedder.forward()

			# add the name of the person + corresponding face
			# embedding to their respective lists
			knownNames.append(name)                 # And then we simply add the name and embedding vec to knownNames and knownEmbeddings, respectively
			knownEmbeddings.append(vec.flatten())
			total += 1  # We also can’t forget about the variable we set to track the total number of faces either — we go ahead and increment the value
#We continue this process of looping over images, detecting faces, and extracting face embeddings for each and every image in our dataset.    
# dump the facial embeddings + names to disk
print("[INFO] serializing {} encodings...".format(total)) # We add the name and embedding data to a dictionary and then serialize the data in a pickle file
data = {"embeddings": knownEmbeddings, "names": knownNames}
f = open(args["embeddings"], "wb")
f.write(pickle.dumps(data))
f.close()      
#At this point we’re ready to extract embeddings by running our script