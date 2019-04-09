import cv2
import json
import libjevois as jevois
import math
import numpy as np
import time


class ReflectiveTape:

    def __init__(self):
        # Load this later when we know the frame dimensions
        self.cameraMatrix = None
        self.distortionMatrix = None

        self.reflectiveVision = ReflectiveTapeProcess()

        self.TARGET_WIDTH = 14.5
        self.TARGET_HEIGHT = 6.0
        # self.TARGET_WIDTH = 14.627
        # self.TARGET_HEIGHT = 5.826

        # camera mount angle (radians)
        # NOTE: not sure if this should be positive or negative
        self.tilt_angle = math.radians(30.0)
        self.is_front_camera = False

        self.target_coords = np.array([[-self.TARGET_WIDTH / 2.0, self.TARGET_HEIGHT / 2.0, 0.0],
                                       [-self.TARGET_WIDTH / 2.0, -self.TARGET_HEIGHT / 2.0, 0.0],
                                       [self.TARGET_WIDTH / 2.0, -self.TARGET_HEIGHT / 2.0, 0.0],
                                       [self.TARGET_WIDTH / 2.0, self.TARGET_HEIGHT / 2.0, 0.0]],
                                      dtype=np.float)

    def load_camera_calibration(self, w, h):
        cpf = "/jevois/share/camera/calibration{}x{}.yaml".format(w, h)
        fs = cv2.FileStorage(cpf, cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            self.cameraMatrix = fs.getNode("camera_matrix").mat()
            self.distortionMatrix = fs.getNode("distortion_coefficients").mat()
            jevois.LINFO("Loaded camera calibration from {}".format(cpf))
        else:
            jevois.LFATAL("Failed to read camera parameters from file [{}]".format(cpf))

    def processAndSend(self, source0):
        capture_timestamp = time.time()
        # numpy.array(source0, copy=True)
        self.reflectiveVision.process(source0)
        height, width, _ = source0.shape

        if self.cameraMatrix is None or self.distortionMatrix is None:
            self.load_camera_calibration(width, height)

        json_pair_list = []
        for pair in self.reflectiveVision.pairs:
            lx, ly, lw, lh = pair.left.bounding_rect
            rx, ry, rw, rh = pair.right.bounding_rect

            image_corners = np.array([[lx, ly + lh],
                                      [lx, ly],
                                      [rx + rw, ry],
                                      [rx + rw, ry + rh]],
                                     dtype=np.float)

            pair.solvePnPData = cv2.solvePnP(self.target_coords, image_corners,
                                             self.cameraMatrix, self.distortionMatrix)

            retval, rvec, tvec = pair.solvePnPData

            if retval:
                output = self.compute_output_values(rvec, tvec)
                json_pair_list.append({
                    "angle": -output[1],
                    "rotation": -output[2],
                    "distance": output[0]
                })

        send_timestamp = time.time()
        capture_ago = send_timestamp - capture_timestamp
        jevois.sendSerial(
            json.dumps({"is_front": self.is_front_camera, "capture_ago": capture_ago, "targets": json_pair_list}))

    def compute_output_values(self, rvec, tvec):
        # Compute the necessary output distance and angles

        # The tilt angle only affects the distance and angle1 calcs

        x = tvec[0][0]
        z = math.sin(self.tilt_angle) * tvec[1][0] + math.cos(self.tilt_angle) * tvec[2][0]

        # distance in the horizontal plane between camera and target
        distance = math.sqrt(x ** 2 + z ** 2)

        # horizontal angle between camera center line and target
        angle1 = math.atan2(x, z)

        rot, _ = cv2.Rodrigues(rvec)
        rot_inv = rot.transpose()
        # This should be pzero_world = numpy.matmul(rot_inv, -tvec)
        B = np.mat(rot_inv)
        C = np.mat(-tvec)

        A = B * C
        pzero_world = A

        angle2 = math.atan2(pzero_world[0][0], pzero_world[2][0])

        return distance, math.degrees(angle1), math.degrees(angle2)

    # Process function with no USB output
    def processNoUSB(self, inframe):
        self.processAndSend(inframe.getCvBGR())

    # Process function with USB output
    def process(self, inframe, outframe):
        inimg = inframe.getCvBGR()
        inframe.done()
        self.processAndSend(inimg)

        outimg = inimg.copy()

        axis = np.float32(
            [[-self.TARGET_WIDTH / 2, -self.TARGET_HEIGHT / 2, 0], [-self.TARGET_WIDTH / 2, self.TARGET_HEIGHT / 2, 0],
             [self.TARGET_WIDTH / 2, self.TARGET_HEIGHT / 2, 0], [self.TARGET_WIDTH / 2, -self.TARGET_HEIGHT / 2, 0],
             [-self.TARGET_WIDTH / 2, -self.TARGET_HEIGHT / 2, -3],
             [-self.TARGET_WIDTH / 2, self.TARGET_HEIGHT / 2, -3], [self.TARGET_WIDTH / 2, self.TARGET_HEIGHT / 2, -3],
             [self.TARGET_WIDTH / 2, -self.TARGET_HEIGHT / 2, -3]])

        for tape in self.reflectiveVision.tapes:
            tape.draw(outimg)

        for pair in self.reflectiveVision.pairs:
            retval, rvec, tvec = pair.solvePnPData
            if retval:
                # project 3D points to image plane
                imgpts, jac = cv2.projectPoints(axis, rvec, tvec, self.cameraMatrix, self.distortionMatrix)

                imgpts = np.int32(imgpts).reshape(-1, 2)
                # draw ground floor in green
                cv2.drawContours(outimg, [imgpts[:4]], -1, (0, 255, 0), 3)
                # draw pillars in blue color
                for i, j in zip(range(4), range(4, 8)):
                    cv2.line(outimg, tuple(imgpts[i]), tuple(imgpts[j]), (255, 0, 0), 3)
                # draw top layer in red color
                cv2.drawContours(outimg, [imgpts[4:]], -1, (0, 0, 255), 3)

        outframe.sendCv(outimg)


class ReflectiveTapeProcess:

    def __init__(self):
        self.gripPipeline = GripPipeline()

        self.tapes = None
        self.pairs = None

    def process(self, source: []):
        self.gripPipeline.process(source)

        self.tapes = []

        for contour in self.gripPipeline.convex_hulls_output:
            self.tapes.append(Tape(contour))

        left_tapes = []
        right_tapes = []

        for tape in self.tapes:
            if tape.is_left:
                left_tapes.append(tape)
            else:
                right_tapes.append(tape)

        self.pairs = []
        # pair up the vision tapes
        for tape1 in left_tapes:
            x1, y1, w1, h1 = tape1.bounding_rect

            cx1 = x1 + w1 / 2
            cy1 = y1 + h1 / 2

            best_match = None
            best_distance = 0

            for tape2 in right_tapes:
                x2, y2, w2, h2 = tape2.bounding_rect

                # Only pair up if right is on the right of left
                if x1 + w1 > x2:
                    continue

                cx2 = x2 + w2 / 2
                cy2 = y2 + h2 / 2

                distance = math.sqrt(math.pow(cx1 - cx2, 2) + math.pow(cy1 - cy2, 2))

                if best_match is None or distance < best_distance:
                    best_match = tape2
                    best_distance = distance

            if best_match is not None:
                self.pairs.append(TapePair(tape1, best_match))
                right_tapes.remove(best_match)


class TapePair:

    def __init__(self, left, right):
        self.left = left
        self.right = right
        self.solvePnPData = None


class Tape:

    def __init__(self, contour):
        self.contour = contour
        self.bounding_rect = cv2.boundingRect(contour)
        self.rotated_rect = cv2.minAreaRect(contour)
        self.angle = self.rotated_rect[2]
        self.is_left = self.angle < -46

    def draw(self, img):
        box = cv2.boxPoints(self.rotated_rect)
        box = np.int0(box)
        cv2.drawContours(img, [box], 0, (64, 64, 64), 2)


class GripPipeline:
    """
    An OpenCV pipeline generated by GRIP.
    """

    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsl_threshold_hue = [45.32374100719425, 87.84982935153582]
        self.__hsl_threshold_saturation = [185.74640287769782, 255.0]
        self.__hsl_threshold_luminance = [52.74280575539568, 255.0]

        self.hsl_threshold_output = None

        self.__find_contours_input = self.hsl_threshold_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

        self.__filter_contours_contours = self.find_contours_output
        self.__filter_contours_min_area = 0.0
        self.__filter_contours_min_perimeter = 0.0
        self.__filter_contours_min_width = 10.0
        self.__filter_contours_max_width = 1000.0
        self.__filter_contours_min_height = 10.0
        self.__filter_contours_max_height = 1000.0
        self.__filter_contours_solidity = [80.03597122302158, 100]
        self.__filter_contours_max_vertices = 1000000.0
        self.__filter_contours_min_vertices = 0.0
        self.__filter_contours_min_ratio = 0.2
        self.__filter_contours_max_ratio = 1.0

        self.filter_contours_output = None

        self.__convex_hulls_contours = self.filter_contours_output

        self.convex_hulls_output = None

    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step HSL_Threshold0:
        self.__hsl_threshold_input = source0
        (self.hsl_threshold_output) = self.__hsl_threshold(self.__hsl_threshold_input, self.__hsl_threshold_hue,
                                                           self.__hsl_threshold_saturation,
                                                           self.__hsl_threshold_luminance)

        # Step Find_Contours0:
        self.__find_contours_input = self.hsl_threshold_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input,
                                                           self.__find_contours_external_only)

        # Step Filter_Contours0:
        self.__filter_contours_contours = self.find_contours_output
        (self.filter_contours_output) = self.__filter_contours(self.__filter_contours_contours,
                                                               self.__filter_contours_min_area,
                                                               self.__filter_contours_min_perimeter,
                                                               self.__filter_contours_min_width,
                                                               self.__filter_contours_max_width,
                                                               self.__filter_contours_min_height,
                                                               self.__filter_contours_max_height,
                                                               self.__filter_contours_solidity,
                                                               self.__filter_contours_max_vertices,
                                                               self.__filter_contours_min_vertices,
                                                               self.__filter_contours_min_ratio,
                                                               self.__filter_contours_max_ratio)

        # Step Convex_Hulls0:
        self.__convex_hulls_contours = self.filter_contours_output
        (self.convex_hulls_output) = self.__convex_hulls(self.__convex_hulls_contours)

    @staticmethod
    def __hsl_threshold(input, hue, sat, lum):
        """Segment an image based on hue, saturation, and luminance ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max luminance.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HLS)
        return cv2.inRange(out, (hue[0], lum[0], sat[0]), (hue[1], lum[1], sat[1]))

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if (external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        contours, hierarchy = cv2.findContours(input, mode=mode, method=method)
        return contours

    @staticmethod
    def __filter_contours(input_contours, min_area, min_perimeter, min_width, max_width,
                          min_height, max_height, solidity, max_vertex_count, min_vertex_count,
                          min_ratio, max_ratio):
        """Filters out contours that do not meet certain criteria.
        Args:
            input_contours: Contours as a list of numpy.ndarray.
            min_area: The minimum area of a contour that will be kept.
            min_perimeter: The minimum perimeter of a contour that will be kept.
            min_width: Minimum width of a contour.
            max_width: MaxWidth maximum width.
            min_height: Minimum height.
            max_height: Maximimum height.
            solidity: The minimum and maximum solidity of a contour.
            min_vertex_count: Minimum vertex Count of the contours.
            max_vertex_count: Maximum vertex Count.
            min_ratio: Minimum ratio of width to height.
            max_ratio: Maximum ratio of width to height.
        Returns:
            Contours as a list of numpy.ndarray.
        """
        output = []
        for contour in input_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if (w < min_width or w > max_width):
                continue
            if (h < min_height or h > max_height):
                continue
            area = cv2.contourArea(contour)
            if (area < min_area):
                continue
            if (cv2.arcLength(contour, True) < min_perimeter):
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            if (solid < solidity[0] or solid > solidity[1]):
                continue
            if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
                continue
            ratio = (float)(w) / h
            if (ratio < min_ratio or ratio > max_ratio):
                continue
            output.append(contour)
        return output

    @staticmethod
    def __convex_hulls(input_contours):
        """Computes the convex hulls of contours.
        Args:
            input_contours: A list of numpy.ndarray that each represent a contour.
        Returns:
            A list of numpy.ndarray that each represent a contour.
        """
        output = []
        for contour in input_contours:
            output.append(cv2.convexHull(contour))
        return output
