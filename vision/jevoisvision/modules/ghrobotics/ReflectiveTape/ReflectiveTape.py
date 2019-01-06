import libjevois as jevois
import json
import time
import cv2
import math
import numpy

class ReflectiveTape:

    def __init__(self):
        self.actualDimension = 7.875
        self.actualDistance = 44.5
        self.pixelDimension = 61
        self.focalLength = self.pixelDimension * self.actualDistance / self.actualDimension
        self.reflectiveVision = ReflectiveTapeProcess()

    def processAndSend(self, source0):
        timestamp = time.time()
        height, width, _ = source0.shape
        list = []
        # for contour in self.gripPipeline.filter_contours_output:
        #     x, y, w, h = cv2.boundingRect(contour)
        #
        #     # angle and distance from camera (angles in radians)
        #     cA = numpy.arctan2(x + w / 2 - width / 2, self.focalLength)
        #     cD = self.actualDimension * self.focalLength / w
        #
        #     # angle and distance after accounting for camera displacement (angles in radians)
        #     angle = numpy.arctan2(cD * numpy.sin(cA), cD * numpy.cos(cA))
        #     distance = cD * numpy.cos(cA) / numpy.cos(angle)
        #
        #     list.append({
        #         "x": x,
        #         "y": y,
        #         "w": w,
        #         "h": h,
        #         "angle": numpy.degrees(angle),
        #         "distance": distance
        #     })
        jevois.sendSerial(json.dumps({"Epoch Time": timestamp, "Contours": list}))

    # Process function with no USB output
    def processNoUSB(self, inframe):
        inimg = inframe.getCvBGR()
        self.reflectiveVision.process(inimg)
        self.processAndSend(inimg)

    # Process function with USB output
    def process(self, inframe, outframe):
        inimg = inframe.get()

        imgbgr = jevois.convertToCvBGR(inimg)
        outimg = outframe.get()
        jevois.paste(inimg, outimg, 0, 0)
        inframe.done()

        self.reflectiveVision.process(numpy.array(imgbgr, copy=True))
        self.processAndSend(imgbgr)

        for leftTape in self.reflectiveVision.leftTapes:
            x, y, w, h = leftTape.bounding_rect

            jevois.drawRect(outimg, x, y, w, h, jevois.YUYV.White)
            jevois.writeText(outimg, str(leftTape.angle), x, y, jevois.YUYV.White, jevois.Font.Font6x10)

        for rightTape in self.reflectiveVision.rightTapes:
            x, y, w, h = rightTape.bounding_rect

            jevois.drawRect(outimg, x, y, w, h, jevois.YUYV.White)
            jevois.writeText(outimg, str(rightTape.angle), x, y, jevois.YUYV.White, jevois.Font.Font6x10)

        for pair in self.reflectiveVision.pairs:
            x, y, w, h = pair[0].bounds(pair[1])

            jevois.drawRect(outimg, x - 5, y - 5, w + 10, h + 10, jevois.YUYV.White)

        # jevois.writeText(outimg, fps, 3, h - 10, jevois.YUYV.White, jevois.Font.Font6x10)
        outframe.send()


class ReflectiveTapeProcess:

    def __init__(self):
        self.gripPipeline = GripPipeline()

        self.leftTapes = None
        self.rightTapes = None

        self.pairs = None

    def process(self, source: []):
        self.gripPipeline.process(source)

        self.filterTapes()

        self.pairs = []
        temp_right_tapes = self.rightTapes.copy()
        # pair up the vision tapes
        for tape1 in self.leftTapes:
            x1, y1, w1, h1 = tape1.bounding_rect

            cx1 = x1 + w1 / 2
            cy1 = y1 + h1 / 2

            best_match = None
            best_distance = 0

            for tape2 in temp_right_tapes:
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
                self.pairs.append((tape1, best_match))
                temp_right_tapes.remove(best_match)

    def filterTapes(self):
        self.leftTapes = []
        self.rightTapes = []

        for contour in self.gripPipeline.convex_hulls_output:
            tape = Tape(contour)

            if tape.is_left:
                self.leftTapes.append(tape)
            else:
                self.rightTapes.append(tape)


class Tape:

    def __init__(self, contour):
        self.contour = contour
        self.bounding_rect = cv2.boundingRect(contour)
        self.rotated_rect = cv2.minAreaRect(contour)
        self.angle = self.rotated_rect[2]
        self.is_left = self.angle < -46

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


class GripPipeline:
    """
    An OpenCV pipeline generated by GRIP.
    """

    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsl_threshold_hue = [48.5611510791367, 95.52901023890786]
        self.__hsl_threshold_saturation = [135.29676258992808, 255.0]
        self.__hsl_threshold_luminance = [61.915467625899275, 255.0]

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
        (self.hsl_threshold_output) = self.__hsl_threshold(self.__hsl_threshold_input, self.__hsl_threshold_hue, self.__hsl_threshold_saturation, self.__hsl_threshold_luminance)

        # Step Find_Contours0:
        self.__find_contours_input = self.hsl_threshold_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input, self.__find_contours_external_only)

        # Step Filter_Contours0:
        self.__filter_contours_contours = self.find_contours_output
        (self.filter_contours_output) = self.__filter_contours(self.__filter_contours_contours, self.__filter_contours_min_area, self.__filter_contours_min_perimeter, self.__filter_contours_min_width, self.__filter_contours_max_width, self.__filter_contours_min_height, self.__filter_contours_max_height, self.__filter_contours_solidity, self.__filter_contours_max_vertices, self.__filter_contours_min_vertices, self.__filter_contours_min_ratio, self.__filter_contours_max_ratio)

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
        return cv2.inRange(out, (hue[0], lum[0], sat[0]),  (hue[1], lum[1], sat[1]))

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if(external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
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
            x,y,w,h = cv2.boundingRect(contour)
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


