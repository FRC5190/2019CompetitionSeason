import libjevois as jevois
import json
import time
from . import white_tape


class FRCRobot:

    def __init__(self):
        self.actualDimension = 7.875
        self.actualDistance = 44.5
        self.pixelDimension = 61
        self.focalLength = self.pixelDimension * self.actualDistance / self.actualDimension
        self.reflectiveVision = white_tape.ReflectiveTape()

    def processAndSend(self, source0):
        timestamp = time.time()
        self.reflectiveVision.process(source0)
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
        self.processAndSend(inimg)

    # Process function with USB output
    def process(self, inframe, outframe):
        inimg = inframe.get()

        imgbgr = jevois.convertToCvBGR(inimg)
        h, w, chans = imgbgr.shape
        outimg = outframe.get()
        outimg.require("output", w, h + 12, jevois.V4L2_PIX_FMT_YUYV)
        jevois.paste(inimg, outimg, 0, 0)
        jevois.drawFilledRect(outimg, 0, h, outimg.width, outimg.height - h, jevois.YUYV.Black)
        inframe.done()

        self.processAndSend(imgbgr)

        # for contour in self.gripPipeline.filter_contours_output:
        #     x, y, w, h = cv2.boundingRect(contour)
        #     jevois.drawRect(outimg, x, y, w, h, 2, jevois.YUYV.White)

        # jevois.writeText(outimg, fps, 3, h - 10, jevois.YUYV.White, jevois.Font.Font6x10)
        outframe.send()
