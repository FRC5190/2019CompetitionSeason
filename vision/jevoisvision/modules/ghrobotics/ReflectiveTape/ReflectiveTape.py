import libjevois as jevois
import json
import time
import cv2
import math
import numpy


class ReflectiveTape:

    def __init__(self):
        fov_norm_x = 0.42
        fov_distance_a = 65.0
        fov_distance_o = 27.0
        self.FOV = math.degrees(math.atan2(fov_distance_o, fov_distance_a)) / fov_norm_x

        self.actualDimensionH = 5.75
        self.actualDistanceH = 67.0
        self.pixelDimensionH = 32.0
        self.focalLengthH = self.pixelDimensionH * self.actualDistanceH / self.actualDimensionH

        self.reflectiveVision = ReflectiveTapeProcess()

    def processAndSend(self, source0):
        timestamp = time.time()
        # numpy.array(source0, copy=True)
        self.reflectiveVision.process(source0)
        height, width, _ = source0.shape

        json_pair_list = []
        for pair in self.reflectiveVision.pairs:
            y = pair.cY
            h = pair.cH

            pair.norm_center_x = (pair.center_x * 2 - width) / width

            pair.angle = pair.norm_center_x * self.FOV

            cAH = numpy.arctan2(y + h / 2 - height / 2, self.focalLengthH)
            cDH = self.actualDimensionH * self.focalLengthH / h

            angleH = numpy.arctan2(cDH * numpy.sin(cAH), cDH * numpy.cos(cAH))
            pair.distance = cDH * numpy.cos(cAH) / numpy.cos(angleH)

            json_pair_list.append({
                "angle": pair.angle,
                "distance": pair.distance
            })
        jevois.sendSerial(json.dumps({"Epoch Time": timestamp, "Targets": json_pair_list}))

    def undistort(self, inimg):
        if not hasattr(self, 'fisheye_maps'):
            width = inimg.shape[1]
            height = inimg.shape[0]

            distCoeff = np.zeros((4, 1), np.float64)

            # TODO: add your coefficients here!
            k1 = -2.2e-3;  # negative to remove barrel distortion
            k2 = 0.0;
            p1 = 0.0;
            p2 = 0.0;

            distCoeff[0, 0] = k1;
            distCoeff[1, 0] = k2;
            distCoeff[2, 0] = p1;
            distCoeff[3, 0] = p2;
            self.distCoeff = distCoeff

            # assume unit matrix for camera
            cam = np.eye(3, dtype=np.float32)

            cam[0, 2] = width / 2.0  # define center x
            cam[1, 2] = height / 2.0  # define center y
            cam[0, 0] = 40.  # define focal length x
            cam[1, 1] = 40.  # define focal length y
            self.cam = cam

        return cv2.undistort(inimg,self.cam,self.distCoeff)

    # Process function with no USB output
    def processNoUSB(self, inframe):
        self.processAndSend(self.undistort(inframe.getCvBGR()))

    # Process function with USB output
    def process(self, inframe, outframe):
        inimg = self.undistort(inframe.getCvBGR())
        inframe.done()
        self.processAndSend(inimg)

        outimg = inimg.copy()

        for tape in self.reflectiveVision.tapes:
            tape.draw(outimg)

        for pair in self.reflectiveVision.pairs:
            pair.draw(outimg)

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


class Tape:

    def __init__(self, contour):
        self.contour = contour
        self.bounding_rect = cv2.boundingRect(contour)
        self.rotated_rect = cv2.minAreaRect(contour)
        self.angle = self.rotated_rect[2]
        self.is_left = self.angle < -46

    def draw(self, img):
        box = cv2.boxPoints(self.rotated_rect)
        box = numpy.int0(box)
        cv2.drawContours(img, [box], 0, (64, 64, 64), 2)

    def bounds(self, other_tape):
        min_x1, min_y1, w1, h1 = self.bounding_rect
        min_x2, min_y2, w2, h2 = other_tape.bounding_rect

        max_x1 = min_x1 + w1
        max_y1 = min_y1 + h1
        max_x2 = min_x2 + w2
        max_y2 = min_y2 + h2

        x = min(min_x1, min_x2)
        y = min(min_y1, min_y2)
        w = max(max_x1, max_x2) - x
        h = max(max_y1, max_y2) - y

        return x, y, w, h


class TapePair:

    def __init__(self, left, right):
        self.left = left
        self.right = right
        self.bounding_rect = self.left.bounds(self.right)
        self.distance = None
        self.angle = None
        x, y, w, h = self.bounding_rect
        lx, ly, lw, lh = self.left.bounding_rect
        rx, ry, rw, rh = self.right.bounding_rect

        self.cX = x
        self.cY = (ly + ly) / 2.0
        self.cW = w
        self.cH = ((ly + lh) + (ry + rh)) / 2.0 - self.cY

        self.center_x = self.cX + self.cW / 2
        self.norm_center_x = None

        self.imagePoints = numpy.array([(lx, ly),
                                        (lx, ly + lh),
                                        (rx + rw, ry + rh),
                                        (rx + rw, ry)], dtype=numpy.int32).reshape((-1, 1, 2))

    def draw(self, img):
        x, y, w, h = self.bounding_rect
        cv2.polylines(img, [self.imagePoints], True, (128, 128, 128))
        cv2.putText(img, "W: " + str(self.cH) + "px " + str(self.distance) + "in", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255))
        cv2.putText(img, str(round(self.norm_center_x * 1000.0) / 1000.0) + " norm " + str(int(self.angle)) +
                    " deg", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255))


class GripPipeline:
    """
    An OpenCV pipeline generated by GRIP.
    """

    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsl_threshold_hue = [27.5179856115108, 98.60068259385665]
        self.__hsl_threshold_saturation = [87.14028776978417, 255.0]
        self.__hsl_threshold_luminance = [43.57014388489208, 255.0]

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
        self.__filter_contours_min_ratio = 0.0
        self.__filter_contours_max_ratio = 1000.0

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
