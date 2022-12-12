import cv2
import numpy as np

# Load the dataset of images
# The dataset should contain two classes: "fire" and "no fire"
X = np.load('dataset_images.npy')
y = np.load('dataset_labels.npy')

# Split the dataset into training and test sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.25)

# Convert the images to grayscale
X_train = np.array([cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for img in X_train])
X_test = np.array([cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for img in X_test])

# Flatten the images into 1D arrays
X_train = X_train.reshape(X_train.shape[0], -1)
X_test = X_test.reshape(X_test.shape[0], -1)

# Normalize the pixel values
X_train = X_train / 255.0
X_test = X_test / 255.0

# Use a support vector machine (SVM) as the model
model = cv2.ml.SVM_create()

# Train the model on the training set
model.train(X_train, cv2.ml.ROW_SAMPLE, y_train)

# Evaluate the model on the test set
_, accuracy = model.predict(X_test)

# Print the model's accuracy on the test set
print("Accuracy:", accuracy)

# Save the trained model
model.save('fire_detection_model.xml')