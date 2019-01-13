import numpy as np
import glob
import cv2

width = 640
height = 480

distCoeff = np.zeros((4, 1), np.float64)

# TODO: add your coefficients here!
k1 = -1.3e-4  # negative to remove barrel distortion
k2 = 0.0
p1 = 0.0
p2 = 0.0

distCoeff[0, 0] = k1
distCoeff[1, 0] = k2
distCoeff[2, 0] = p1
distCoeff[3, 0] = p2

# assume unit matrix for camera
cam = np.eye(3, dtype=np.float32)

cam[0, 2] = width / 2.0  # define center x
cam[1, 2] = height / 2.0  # define center y
cam[0, 0] = 10.  # define focal length x
cam[1, 1] = 10.  # define focal length y


for image in glob.glob('*.jpg'):
    img = cv2.imread(image)
    border = 20
    imgResized = cv2.copyMakeBorder(img, border, border, border, border, cv2.BORDER_REPLICATE)
    dst = cv2.undistort(imgResized, cam, distCoeff)
    cv2.imshow(image, dst)

cv2.waitKey(0)
cv2.destroyAllWindows()
