import numpy as np
import cv2

def generate_chessboard_with_border(rows, cols, square_size, border_size, save_path):
    """
    Generate a chessboard pattern image with a white border and save it.

    Parameters:
    - rows: Number of rows in the chessboard.
    - cols: Number of columns in the chessboard.
    - square_size: Size of each square in the chessboard.
    - border_size: Size of the white border around the chessboard.
    - save_path: Path to save the generated chessboard image.
    """

    # Calculate the size of the entire image (including the border)
    img_rows = rows * square_size + 2 * border_size
    img_cols = cols * square_size + 2 * border_size

    # Create an empty image with a white border
    chessboard_with_border = np.ones((img_rows, img_cols), dtype=np.uint8) * 255

    # Create an empty chessboard without the border
    chessboard = np.ones((rows * square_size, cols * square_size), dtype=np.uint8) * 255

    # Alternate black and white squares in the chessboard
    for i in range(rows):
        for j in range(cols):
            if (i + j) % 2 == 1:
                chessboard[i * square_size:(i + 1) * square_size, j * square_size:(j + 1) * square_size] = 0

    # Copy the chessboard into the larger image, leaving the white border
    chessboard_with_border[border_size:border_size + rows * square_size,
                           border_size:border_size + cols * square_size] = chessboard

    # Save the chessboard image with a white border
    cv2.imwrite(save_path, chessboard_with_border)

    # Display the generated chessboard with a white border
    cv2.imshow('Generated Chessboard with Border', chessboard_with_border)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Specify the parameters for the chessboard with border
rows = 8  # Number of rows
cols = 8  # Number of columns
square_size = 100  # Size of each square in pixels
border_size = 100  # Size of the white border around the chessboard
save_path = 'chessboard_pattern_with_border.png'  # Path to save the generated chessboard image with a border

# Generate and save the chessboard pattern with a white border
generate_chessboard_with_border(rows, cols, square_size, border_size, save_path)
