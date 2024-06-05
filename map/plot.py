import numpy as np
import matplotlib.pyplot as plt

# Assume data is a 3D numpy array with shape (2, 255, 255)
# For the purpose of this example, let's create a sample data array
# Replace this with your actual data
data = np.load("liu.npy")

# Extracting the x and y components of the vectors
U = data[0]  # x components
V = data[1]  # y components

# Creating a grid of coordinates (X, Y)
x = np.arange(data.shape[1])
y = np.arange(data.shape[2])
X, Y = np.meshgrid(x, y)

# Plotting the vector field
plt.figure(figsize=(10, 10))
plt.quiver(X, Y, U, V, scale=10, color='b')

# Inverting the y-axis to fix the upside-down issue
plt.gca().invert_yaxis()

# Setting plot labels and title
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Vector Field')

# Displaying the plot
plt.show()
