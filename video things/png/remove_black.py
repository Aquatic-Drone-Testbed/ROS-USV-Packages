import cv2
import numpy as np
import os

def make_black_transparent(image_path, output_path):
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    if image is None:
        print(f"Error: Unable to read image {image_path}")
        return
    
    # Convert grayscale to BGRA
    image_bgra = cv2.cvtColor(image, cv2.COLOR_GRAY2BGRA)
    
    # Make black pixels transparent
    black_pixels = np.all(image_bgra[:, :, :3] == [0, 0, 0], axis=-1)
    image_bgra[black_pixels, 3] = 0

    cv2.imwrite(output_path, image_bgra)

# Directory containing the radar images
input_directory = './original_images'  # Current directory
output_directory = './transparent_images'

# Ensure the output directory exists
os.makedirs(output_directory, exist_ok=True)

# Process each PNG image
for filename in os.listdir(input_directory):
    if filename.startswith('radar_image') and filename.endswith('.png'):
        input_path = os.path.join(input_directory, filename)
        output_path = os.path.join(output_directory, filename)
        make_black_transparent(input_path, output_path)

print("Processing complete. Transparent images are saved in:", output_directory)
