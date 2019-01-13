import glob

import cv2

assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np

# You should replace these 3 lines with the output in calibration step
DIM = (640, 480)
K = np.array(
    [[402.56679421300635, 0.0, 326.0125901609549], [0.0, 402.63097026394826, 225.2408285699041], [0.0, 0.0, 1.0]])
D = np.array([[-0.2847610313234561], [1.5393686107435003], [-3.7781953960127272], [3.045419119554048]])


def undistort(img_path, balance=0.0, dim2=None, dim3=None):
    img = cv2.imread(img_path)
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
    assert dim1[0] / dim1[1] == DIM[0] / DIM[
        1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow(img_path, undistorted_img)


for image in glob.glob('*.jpg'):
    undistort(image, balance=0.8)

cv2.waitKey(0)
cv2.destroyAllWindows()
