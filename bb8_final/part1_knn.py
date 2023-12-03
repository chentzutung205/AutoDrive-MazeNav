import cv2
import csv
import numpy as np
import os
import joblib

from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import confusion_matrix, accuracy_score


# Define the new crop function that uses color thresholding
def find_sign_by_color(img_hsv, lower_color, upper_color, image_path):
    mask = cv2.inRange(img_hsv, lower_color, upper_color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = 825

    if not contours or cv2.contourArea(max(contours, key=cv2.contourArea)) < min_area:
        height, width = img_hsv.shape[:2]
        size = min(height, width) // 2
        center = (height // 2, width // 2)
        return img_hsv[center[0] - size // 2:center[0] + size // 2,
                       center[1] - size // 2:center[1] + size // 2]

    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    cropped_img = img_hsv[y:y+h, x:x+w]

    return cropped_img


# Use scikit-learn's KNeighborsClassifier as the function of train_knn_classifier 
def train_knn_classifier(train_images, train_labels):
    # Flatten the images
    train_data = train_images.reshape(train_images.shape[0], -1)
    
    # Create and train the KNeighborsClassifier
    knn = KNeighborsClassifier(n_neighbors=5)
    knn.fit(train_data, train_labels)
    
    return knn


# Use scikit-learn's KNeighborsClassifier as the function of test_knn_classifier
def test_knn_classifier(knn, test_images, test_labels):
    # Flatten the images
    test_data = test_images.reshape(test_images.shape[0], -1)
    
    # Predict labels for test data
    predictions = knn.predict(test_data)
    
    # Calculate accuracy and confusion matrix
    accuracy = accuracy_score(test_labels, predictions)
    conf_matrix = confusion_matrix(test_labels, predictions)
    
    return accuracy, conf_matrix


# Modify the process_images function to use the color thresholding
def process_images(image_paths, image_directory, lower_color, upper_color, save_directory):
    if not os.path.exists(save_directory):
        os.makedirs(save_directory)

    processed_images = []
    for image_path in image_paths:
        img = cv2.imread(f"{image_directory}{image_path}.jpg", cv2.IMREAD_COLOR)
        if img is None:
            print(f"Image at {image_directory}{image_path}.jpg could not be read.")
            continue
        
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        cropped_img = find_sign_by_color(img_hsv, lower_color, upper_color, image_path)
        if cropped_img is not None and cropped_img.size > 0:
            resized_img = cv2.resize(cropped_img, (33, 25))
            processed_images.append(resized_img)
        else:
            print(f"Sign not found in image {image_path}.jpg")
            resized_img = np.zeros((33, 25, 3), dtype=np.uint8)  # Placeholder for images without a detected sign
            processed_images.append(resized_img)

        save_path = os.path.join(save_directory, f"processed_{image_path}.jpg")
        cv2.imwrite(save_path, resized_img)

    return np.array(processed_images)


# Path for the image directory and the label file
image_directory = './images/'
label_file = 'labels.txt'
save_directory = './processed_images/'


# Load images and labels
with open(f"{image_directory}{label_file}", 'r') as f:
    reader = csv.reader(f)
    data_lines = list(reader)

data_image_paths = [line[0] for line in data_lines]
data_labels = np.array([int(line[1]) for line in data_lines])


# Split the data and labels into training and test sets with a 3:1 ratio
train_data_path, test_data_path, train_labels, test_labels = train_test_split(data_image_paths, data_labels, test_size=0.25, random_state=47)


# Adjust these values based on sign colors
lower_color = np.array([0, 100, 40])
upper_color = np.array([180, 255, 191])


# Process training images
train_images = process_images(train_data_path, image_directory, lower_color, upper_color, save_directory)

accus = []
k = 5
# Train the KNN classifier using scikit-learn
knn = train_knn_classifier(train_images, train_labels)


# Process test images
test_images = process_images(test_data_path, image_directory, lower_color, upper_color, save_directory)


# Test the KNN classifier using scikit-learn
accuracy, confusion_matrix = test_knn_classifier(knn, test_images, test_labels)
# Print the results
print(f"Best Accuracy: {accuracy}")
print(f"Best Confusion matrix: \n{confusion_matrix}")


# save the model to disk using joblib
filename = 'knn_model_1200.sav'
joblib.dump(knn, filename, compress=3)


# load the model from disk
loaded_model = joblib.load(filename)


# Process test images for prediction
test_images_processed = process_images(test_data_path, image_directory, lower_color, upper_color, save_directory)
# Flatten the processed test images
test_data = test_images_processed.reshape(test_images_processed.shape[0], -1)


# Predict labels using the loaded model
loaded_predictions = loaded_model.predict(test_data)

# Calculate accuracy and confusion matrix
loaded_accuracy = accuracy_score(test_labels, loaded_predictions)
loaded_confusion_matrix = confusion_matrix[(test_labels, loaded_predictions)]

# Print the results
print(f"Loaded Model Accuracy: {loaded_accuracy}")
print(f"Loaded Model Confusion Matrix:\n{loaded_confusion_matrix}")