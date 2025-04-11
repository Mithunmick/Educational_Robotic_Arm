import numpy as np
import cv2
def detect_smarties(image):
    # Convert the image from BGR to HSV color space
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define HSV thresholds for red color
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Create a mask for red regions by combining both ranges
    mask1 = cv2.inRange(image_hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(image_hsv, lower_red2, upper_red2)
    red_mask = mask1 | mask2

    # Find contours of the red regions
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Calculate areas of all bounding boxes and find the largest area
    bounding_box_areas = [cv2.boundingRect(contour)[2] * cv2.boundingRect(contour)[3] for contour in contours]
    max_area = max(bounding_box_areas) if bounding_box_areas else 0

    # Define a threshold for filtering: 80% of the largest bounding box area
    threshold_area = 0.80 * max_area

    # Prepare a list to store the bounding boxes
    filtered_bounding_boxes = []

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        box_area = w * h  # Calculate the area of the bounding box

        if box_area >= threshold_area:  # Retain bounding boxes above the threshold
            filtered_bounding_boxes.append((x, y, w, h))

    return filtered_bounding_boxes
