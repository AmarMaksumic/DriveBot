import cv2
import numpy as np

# read image 
rgb_image = cv2.imread('camera.png')
# to hsv
hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
# bounds
low_1 = np.array([100,140,140])
high_1 = np.array([140,255,255])

# low_2 = np.array([160,144,190])
# high_2 = np.array([180,179,255])
# hsv threshold
mask_1 = cv2.inRange(hsv_image, low_1, high_1)
# mask_2 = cv2.inRange(hsv_image, low_2, high_2)
mask = mask_1 #+ mask_2
cv2.imshow('mask', mask)
cv2.waitKey(0)
# find largest contour
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if len(contours) > 0:
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    # draw rectangle
    cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

# show image
cv2.imshow('filtered_mask', rgb_image)
cv2.waitKey(0)