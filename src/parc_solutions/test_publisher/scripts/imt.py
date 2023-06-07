import cv2
import numpy as np


def on_trackbar_change(value):
    # Handle trackbar value change
    pass

def main():
    image = cv2.imread('front_cam_img.jpg')
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    COLOR_MIN = (12, 120, 124)
    COLOR_MAX = (80, 255, 189)
    thresh_img = cv2.inRange(gray_image, COLOR_MIN, COLOR_MAX)
    thresh_img = cv2.erode(thresh_img, None)
    params = cv2.SimpleBlobDetector_Params()
    # Set parameters for blob detection (change as needed)
    params.minThreshold = 30
    params.maxThreshold = 20000
    params.filterByArea = True
    params.minArea = 100
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False

    # Create blob detector
    detector = cv2.SimpleBlobDetector_create(params)
    
    # Detect blobs
    keypoints = detector.detect(thresh_img)
    print(keypoints)
    image_with_blobs = cv2.drawKeypoints(thresh_img, keypoints, np.array([]), (0, 0, 255),
                                             cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    cv2.imshow('Trackbar Window', image_with_blobs)

    # Break the loop if 'q' is pressed
    while True:
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
