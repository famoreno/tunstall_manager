# import the necessary packages
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
import argparse
import pickle

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-e", "--embeddings", required=True,
	help="path to serialized db of facial embeddings") # The path to the serialized embeddings (we exported it by running the previous extract_embeddings.py script).
ap.add_argument("-r", "--recognizer", required=True,
	help="path to output model trained to recognize faces") # This will be our output model that recognizes faces. It is based on SVM. We’ll be saving it so we can use it in the next two recognition scripts.
ap.add_argument("-l", "--le", required=True,
	help="path to output label encoder") #Our label encoder output file path. We’ll serialize our label encoder to disk so that we can use it and the recognizer model in our image/video face recognition scripts.
args = vars(ap.parse_args())
# Each of these arguments is required. Let’s load our facial embeddings and encode our labels:
# load the face embeddings
print("[INFO] loading face embeddings...")
data = pickle.loads(open(args["embeddings"], "rb").read()) #We won’t be generating any embeddings in this model training script — we’ll use the embeddings previously generated and serialized.

# encode the labels
print("[INFO] encoding labels...")
le = LabelEncoder()							#Then we initialize our scikit-learn LabelEncoder and encode our name
labels = le.fit_transform(data["names"])

# train the model used to accept the 128-d embeddings of the face and
# then produce the actual face recognition
print("[INFO] training model...")
recognizer = SVC(C=1.0, kernel="linear", probability=True) #On Line 29 we initialize our SVM model, and on Line 30 we fit the model (also known as “training the model”).
recognizer.fit(data["embeddings"], labels) # Here we are using a Linear Support Vector Machine (SVM). After training the model we output the model and label encoder to disk as pickle files.

# write the actual face recognition model to disk
f = open(args["recognizer"], "wb")
f.write(pickle.dumps(recognizer))
f.close()
# write the label encoder to disk
f = open(args["le"], "wb")
f.write(pickle.dumps(le))
f.close()
# Here we are using a Linear Support Vector Machine (SVM).
# After training the model we output the model and label encoder to disk as pickle files.