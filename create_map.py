import os
import numpy as np
from PIL import Image, ImageFilter


## Read in data ##
print("Read in character data ...")
# Specify the folder containing the images
cur_path = os.getcwd()
print(cur_path)
img_dir_path = os.path.join(cur_path, "resource_img")

# List all PNG images in the folder
image_files = [f for f in os.listdir(img_dir_path) if f.endswith('.png')]

if not image_files:
    print("No PNG images found in the folder.")
    exit()

# Initialize the minimum width and height to infinity
min_width, min_height = float('inf'), float('inf')

# Find the smallest image dimensions
for image_file in image_files:
    image_path = os.path.join(img_dir_path, image_file)
    with Image.open(image_path) as img:
        width, height = img.size
        # print(img.size, img.mode)
        if width < min_width:
            min_width = width
        if height < min_height:
            min_height = height

print(f"The smallest image dimensions are: {min_width}x{min_height}")

# Resize all images to the smallest dimensions
# img_list = []
for image_file in image_files:
    image_path = os.path.join(img_dir_path, image_file)
    with Image.open(image_path) as img:
        grayscale_img = img.convert('L')
        resized_img = grayscale_img.resize((min_width, min_height))

        # save a copy as background for processing
        resized_img.save(os.path.join(cur_path, "Processing/data", image_file))
        resized_img_np = np.asarray(resized_img)

        # create (gradient) map
        blurred_img = resized_img.filter(ImageFilter.GaussianBlur(7))
        blurred_img_np = np.asarray(blurred_img)

        augment_img_np = (resized_img_np + blurred_img_np).astype(float)
        augment_img_np /= augment_img_np.max() # normalize

        grad_row, grad_col = np.gradient(augment_img_np)
        map = np.stack((grad_row, grad_col), axis=0)
        np.save(os.path.join(cur_path, "map", image_file[:-4]+".npy"), map)
        # img_list.append(np.asarray(binary_img))

# print(len(img_list))
# print(img_list[4])
# import matplotlib.pyplot as plt
# plt.imshow(img_list[4])
# plt.waitforbuttonpress()

