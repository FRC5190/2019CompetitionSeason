import math

import cv2

from . import grip


class ReflectiveTape:

    def __init__(self):
        self.gripPipeline = grip.GripPipeline()

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

    # this is usb debug/streaming
    def processAndDraw(self, source, outimg):
        self.process(source)

        for leftTape in self.leftTapes:
            x, y, w, h = leftTape.bounding_rect

            cv2.rectangle(outimg, (x, y), (x + w, y + h), (0, 255, 0))

        for rightTape in self.rightTapes:
            x, y, w, h = rightTape.bounding_rect

            cv2.rectangle(outimg, (x, y), (x + w, y + h), (255, 0, 255))

        for pair in self.pairs:
            x, y, w, h = pair[0].bounds(pair[1])

            cv2.rectangle(outimg, (x - 5, y - 5), (x + w + 5, y + h + 5), (255, 0, 0))


class Tape:

    def __init__(self, contour):
        self.contour = contour
        self.bounding_rect = cv2.boundingRect(contour)
        self.rotated_rect = cv2.minAreaRect(contour)
        self.is_left = self.rotated_rect[2] < -46

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
