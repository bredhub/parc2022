import cv2


def on_trackbar_change(value):
    # Handle trackbar value change
    pass

def main():
    image = cv2.imread('im1.png')

    # Create a window and trackbar
    cv2.namedWindow('Trackbar Window')
    cv2.createTrackbar('Threshold', 'Trackbar Window', 0, 255, on_trackbar_change)
    cv2.createTrackbar('Threshold', 'Trackbar Window', 255, 255, on_trackbar_change)
    while True:
        # Get the trackbar value
        threshold_value = cv2.getTrackbarPos('Threshold', 'Trackbar Window')

        # Process the image using the threshold value
        _, thresholded_image = cv2.threshold(image, threshold_value, 255, cv2.THRESH_BINARY)

        # Display the image
        cv2.imshow('Trackbar Window', thresholded_image)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
