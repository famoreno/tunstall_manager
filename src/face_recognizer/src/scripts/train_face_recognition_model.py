# import the necessary packages
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
import pickle

# Each of these arguments is required. Let’s load our facial embeddings and encode our labels:
# load the face embeddings
print("[INFO] loading face embeddings...")
embeddingsPath = "../../data/train_output/embeddings.pickle"
data = pickle.loads(open(embeddingsPath, "rb").read()) # We won’t be generating any embeddings in this model training script — we’ll use the embeddings previously generated and serialized.

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
recognizerPath = "../../data/train_output/recognizer.pickle"
f = open(recognizerPath, "wb")
f.write(pickle.dumps(recognizer))
f.close()

# write the label encoder to disk
lePath = "../../data/train_output/le.pickle"
f = open(lePath, "wb")
f.write(pickle.dumps(le))
f.close()
# Here we are using a Linear Support Vector Machine (SVM).
# After training the model we output the model and label encoder to disk as pickle files.