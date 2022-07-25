from doctest import OutputChecker
import cv2
from cv2 import cvtColor
from cv2 import aruco

import numpy as np

class detect_colours():

    def __init__(self):

        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cam.set(cv2.CAP_PROP_EXPOSURE, 50)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cam.set(cv2.CAP_PROP_FPS, 60)

        while True:
            ret, self.img= self.cam.read()

            # convert to hsv colorspace
            hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

            # lower bound and upper bound for Green color
            lower_bound = np.array([50, 20, 20])   
            upper_bound = np.array([100, 255, 255])

            # find the colors within the boundaries
            mask = cv2.inRange(hsv, lower_bound, upper_bound)

            #define kernel size  
            kernel = np.ones((7,7),np.uint8)

            # Remove unnecessary noise from mask

            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Segment only the detected region
            segmented_img = cv2.bitwise_and(self.img, self.img, mask=mask)

            # Find contours from the mask
            contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            output = cv2.drawContours(segmented_img, contours, -1, (0, 0, 255), 3)

            detector = cv2.SimpleBlobDetector()

            blobs = detector.detect(output)

            #im_with_keypoints = cv2.drawKeypoints(self.img, blobs, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            #cv2.imshow("Keypoints", im_with_keypoints)

            cv2.imshow("fnajn",output)

            print(contours)

            if cv2.waitKey(1) and  0xFF == ord("q"):
                break

        self.cam.release()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    go = detect_colours()