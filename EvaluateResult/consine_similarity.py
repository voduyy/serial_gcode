# # # Import necessary libraries
import cv2
import numpy as np
from tensorflow.keras.applications.vgg16 import VGG16, preprocess_input
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import Model
from scipy.spatial.distance import cosine
# Function to extract features using VGG16 and calculate cosine similarity
def calculate_feature_similarity(image_path1, image_path2):
    # Load pre-trained VGG16 model
    base_model = VGG16(weights='imagenet', include_top=False)
    model = Model(inputs=base_model.input, outputs=base_model.get_layer('block5_pool').output)

    # Function to preprocess image for VGG16
    def preprocess_image(image_path):
        img = image.load_img(image_path, target_size=(224, 224))  # Resize the image to 224x224 for VGG16
        img_data = image.img_to_array(img)
        img_data = np.expand_dims(img_data, axis=0)
        img_data = preprocess_input(img_data)  # Preprocessing step for VGG16
        return img_data

    # Preprocess the two images
    img1 = preprocess_image(image_path1)
    img2 = preprocess_image(image_path2)

    # Extract features using VGG16 model
    features_img1 = model.predict(img1)
    features_img2 = model.predict(img2)

    # Flatten features and calculate cosine similarity
    features_img1 = features_img1.flatten()
    features_img2 = features_img2.flatten()
    similarity = 1 - cosine(features_img1, features_img2)

    return similarity


# # Load the original image
# original_image_path = 'C:\D\image2Gcode\output_insideFrame/1_gcode_100.nc.jpg'  # Replace with your original image path
#
# # List of images to compare with the original image (e.g., reduced detail images)
# comparison_image_paths = [
#     'C:\D\image2Gcode\output_insideFrame/1_gcode_25.nc.jpg',
#     'C:\D\image2Gcode\output_insideFrame/1_gcode_50.nc.jpg',  # Replace with your image paths
#     'C:\D\image2Gcode\output_insideFrame/1_gcode_72.nc.jpg',
#     'C:\D\image2Gcode\output_insideFrame/1_gcode_75.nc.jpg',
#     'C:\D\image2Gcode\output_insideFrame/1_gcode_100.nc.jpg',
#     # Add more images here
# ]
#
# # Compare feature-based similarity (Cosine similarity) for each image with the original image
# for image_path in comparison_image_paths:
#     # Calculate feature-based similarity using VGG16
#     feature_similarity = calculate_feature_similarity(original_image_path, image_path)
#     print(f"Feature-based similarity (Cosine similarity) with {image_path}: {feature_similarity}")
#
