import cv2

from jevoisvision.modules.JeVois.ReflectiveTape import reflective_tape

reflectiveTape = reflective_tape.ReflectiveTape()

# img = cv2.imread("../images/CargoAngledDark48in.jpg")
# img = cv2.imread("../images/CargoSideStraightDark60in.jpg")
img = cv2.imread("../images/CargoStraightDark48in.jpg")

outimg = img.copy()

reflectiveTape.processAndDraw(img, outimg)

cv2.imshow('img', outimg)
cv2.waitKey(0)

