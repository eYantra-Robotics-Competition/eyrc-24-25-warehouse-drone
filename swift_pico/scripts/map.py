import cv2
import numpy as np
import cv2.aruco as aruco

class Arena:

    def __init__(self, image_path):
        self.width = 1000
        self.height = 1000
        self.image_path = image_path
        self.detected_markers = []
        self.obstacles = 0
        self.total_area = 0

    def identification(self):

        # Read the image
        frame = cv2.imread()
        



        ###################################
        # Identify the Aruco ID's in the given image

        

        self.detected_markers = []
        ###################################
        # Apply Perspeactive Transform



        transformed_image = ()
        ###################################
        # Use the transformed image to find obstacles and their area



        self.total_area = 0
        ###################################


    def text_file(self):
        with open("obstacles.txt", "w") as file:
            file.write(f"Aruco ID: {self.detected_markers}\n")
            file.write(f"Obstacles: {self.obstacles}\n")
            file.write(f"Area: {self.total_area}\n")


if __name__ == '__main__':
    image_path = 'task1c_image.jpg'
    arena = Arena(image_path)
    arena.identification()
    arena.text_file()
