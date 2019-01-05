import cv2

assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np

# You should replace these 3 lines with the output in calibration step
DIM = (829, 664)
K = np.array([[493.385734691119, 0.0, 417.7816201303714], [0.0, 490.98098374639596, 300.734269553709], [0.0, 0.0, 1.0]])
D = np.array([[-0.06091755643228847], [0.4191362746297005], [-2.053750143753869], [3.3069882520198037]])


def undistort(img_path):
    img = cv2.imread(img_path)
    h, w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


undistort("Screenshot_1.png")
undistort("Screenshot_2.png")
undistort("Screenshot_3.png")
undistort("Screenshot_4.png")
