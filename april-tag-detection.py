import pupil_apriltags as apriltag
import cv2
import numpy as np

# TODO: get distance from april tag

'''
    Pick one of the following:
        * Focal-length (in mm and pixels per mm)
        * Physical size of the image sensor (to calculate pixels per mm)
    
    Sensor datasheet is needed for the second option, likely to use the first method in testing

    Focal-length steps: 
        1. Calibrate the camera with OpenCV and chessboard png: 
            https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        2. Use co-efficients to convert mm to px
                                        focal length (mm) * real height of the object (mm) * image height (pixels)
             distance to object (mm) = ----------------------------------------------------------------------------
                                                        object height (pixels) * sensor height (mm)
'''

types = 'tag36h11'

detector = apriltag.Detector(families=types)

real_size = (192, 192) # in mm

# image = cv2.imread('apriltag.png')
# grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


cam = cv2.VideoCapture(0)
ptA, ptB, ptC, ptD = None, None, None, None

def main():
    while True:
        ret, frame = cam.read()
        if not ret or cv2.waitKey(1) == ord('x'): break

        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        results = detector.detect(grayscale)

        # reset bbox
        if results is None or len(results) == 0: 
            ptB = None
            ptC = None
            ptD = None
            ptA = None

        # draw bbox 
        else:
            # get bbox
            ptA, ptB, ptC, ptD = [], [], [], []
            for res in results:
                (a, b, c, d) = res.corners
                ptB.append((int(b[0]), int(b[1])))
                ptC.append((int(c[0]), int(c[1])))
                ptD.append((int(d[0]), int(d[1])))
                ptA.append((int(a[0]), int(a[1])))

            for (a, b, c, d) in zip(ptA, ptB, ptC, ptD):
                cv2.line(frame, a, b, (0, 255, 0), 2)
                cv2.line(frame, b, c, (0, 255, 0), 2)
                cv2.line(frame, c, d, (0, 255, 0), 2)
                cv2.line(frame, d, a, (0, 255, 0), 2) 

        cv2.imshow('frame', frame)


    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()


# # draw the center (x, y)-coordinates of the AprilTag
#     (cX, cY) = (int(res.center[0]), int(res.center[1]))
#     cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
# # draw the tag family on the image
#     tagFamily = res.tag_family.decode("utf-8")
#     cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
#                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#     print("[INFO] tag family: {}".format(tagFamily))
# # show the output image after AprilTag detection
# cv2.imshow("Image", image)
# cv2.waitKey(0)

# cam = cv2.VideoCapture(0)

# while True:
#     ret, frame = cam.read()
#     if not ret: break

#     grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
