from jevoisvision.modules.JeVois.ReflectiveTape import grip


class ReflectiveTape:

    def __init__(self):
        self.gripPipeline = grip.GripPipeline()

    def process(self, source: []):
        self.gripPipeline.process(source)

    # this is usb debug/streaming
    def processAndDraw(self, source, outimg):
        self.process(source)
