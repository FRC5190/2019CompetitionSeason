import libjevois as jevois
import cv2
import numpy
import json
import time
from . import grip


class FRCRobot:
    """
    An OpenCV pipeline generated by GRIP.
    """

    def __init__(self):
        self.DIM = (829, 664)
        self.K = numpy.array(
            [[493.385734691119, 0.0, 417.7816201303714], [0.0, 490.98098374639596, 300.734269553709], [0.0, 0.0, 1.0]])
        self.D = numpy.array([[-0.06091755643228847], [0.4191362746297005], [-2.053750143753869], [3.3069882520198037]])

        self.actualDimension = 7.875
        self.actualDistance = 44.5
        self.pixelDimension = 61
        self.focalLength = self.pixelDimension * self.actualDistance / self.actualDimension
        self.gripPipeline = grip.GripPipeline()

    # ###################################################################################################
    # Load camera calibration from JeVois share directory
    def load_camera_calibration(self, w, h):
        cpf = "/jevois/share/camera/calibration{}x{}.yaml".format(w, h)
        fs = cv2.FileStorage(cpf, cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            self.camMatrix = fs.getNode("camera_matrix").mat()
            self.distCoeffs = fs.getNode("distortion_coefficients").mat()
            jevois.LINFO("Loaded camera calibration from {}".format(cpf))
        else:
            jevois.LFATAL("Failed to read camera parameters from file [{}]".format(cpf))

    def processAndSend(self, source0):
        timestamp = time.time()
        self.gripPipeline.process(source0)
        height, width, _ = source0.shape
        list = []
        for contour in self.gripPipeline.filter_contours_output:
            x, y, w, h = cv2.boundingRect(contour)

            # angle and distance from camera (angles in radians)
            cA = numpy.arctan2(x + w / 2 - width / 2, self.focalLength)
            cD = self.actualDimension * self.focalLength / w

            # angle and distance after accounting for camera displacement (angles in radians)
            angle = numpy.arctan2(cD * numpy.sin(cA), cD * numpy.cos(cA))
            distance = cD * numpy.cos(cA) / numpy.cos(angle)

            list.append({
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "angle": numpy.degrees(angle),
                "distance": distance
            })
        outputData = {"Epoch Time": timestamp, "Contours": list}
        jevois.sendSerial(json.dumps(outputData))

    # ###################################################################################################
    # Process function with no USB output
    def processNoUSB(self, inframe):
        inimg = inframe.getCvBGR()
        self.processAndSend(inimg)

    # ###################################################################################################
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

        for contour in self.gripPipeline.filter_contours_output:
            x, y, w, h = cv2.boundingRect(contour)
            jevois.drawRect(outimg, x, y, w, h, 2, jevois.YUYV.White)

        # jevois.writeText(outimg, fps, 3, h - 10, jevois.YUYV.White, jevois.Font.Font6x10)
        outframe.send()
