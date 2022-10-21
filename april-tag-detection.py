import apriltag
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
'''

types = 'tag36h11'
options = apriltag.DetectorOptions(families=types)

detector = apriltag.Detector(options)

# image = cv2.imread('apriltag.png')
# grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


class AprilTagDetector: 

    cam = cv2.VideoCapture(0)
    ptA, ptB, ptC, ptD = None, None, None, None

    def __init__(self):
        pass

    def main(self):
        while True:
            ret, frame = self.cam.read()
            if not ret or cv2.waitKey(1) == ord('x'): break

            grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 

            # draw bbox 
            if (self.ptA != None and self.ptB != None and self.ptC != None and self.ptD != None):
                for (a, b, c, d) in zip(self.ptA, self.ptB, self.ptC, self.ptD):
                    cv2.line(frame, a, b, (0, 255, 0), 2)
                    cv2.line(frame, b, c, (0, 255, 0), 2)
                    cv2.line(frame, c, d, (0, 255, 0), 2)
                    cv2.line(frame, d, a, (0, 255, 0), 2) 

            cv2.imshow('frame', frame)
 
            results = detector.detect(grayscale)

            # reset bbox
            if results is None or len(results) == 0: 
                self.ptB = None
                self.ptC = None
                self.ptD = None
                self.ptA = None
                continue

            # get bbox
            self.ptA, self.ptB, self.ptC, self.ptD = [], [], [], []
            for index, res in enumerate(results):
                # print(str(index))
                (ptA, ptB, ptC, ptD) = res.corners
                self.ptB.append((int(ptB[0]), int(ptB[1])))
                self.ptC.append((int(ptC[0]), int(ptC[1])))
                self.ptD.append((int(ptD[0]), int(ptD[1])))
                self.ptA.append((int(ptA[0]), int(ptA[1])))

            
            print(len(results))


        self.cam.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    run = AprilTagDetector()
    run.main()


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
