import numpy as np
import cv2

def generate_color_pattern(h, w, d, r):
    # Initialize the pattern with zeros
    pattern = np.zeros((h, w, 3))

    # Calculate the number of patches in each dimension
    num_patches_x = h // d
    num_patches_y = w // d

    # Maximum number of attempts to find a suitable color
    max_attempts = 1000

    # Iterate over the patches
    for i in range(num_patches_x):
        for j in range(num_patches_y):
            # Get the neighbors
            neighbors = []
            if i > 0:
                neighbors.append(pattern[(i-1)*d:i*d, j*d:(j+1)*d])
            if j > 0:
                neighbors.append(pattern[i*d:(i+1)*d, (j-1)*d:j*d])

            # Generate a new color
            for attempt in range(max_attempts):
                new_color = np.random.rand(3)
                if all(np.min((new_color - neighbor.mean(axis=(0, 1)))**2) >= r for neighbor in neighbors):
                    break
            if attempt == max_attempts - 1:
                print(f"Warning: Could not find a suitable color for patch ({i}, {j}) after {max_attempts} attempts")

            # Apply the new color to the patch
            pattern[i*d:(i+1)*d, j*d:(j+1)*d] = new_color

            print('Patch ({}, {})'.format(i, j))

    return pattern

# Test the function
pattern = generate_color_pattern(300, 300, 3, 0.5)

# Convert the pattern to a format suitable for display with OpenCV
pattern_display = (pattern * 255).astype(np.uint8)

# Display the pattern
cv2.imshow('Pattern', pattern_display)
cv2.waitKey(0)
cv2.destroyAllWindows()
