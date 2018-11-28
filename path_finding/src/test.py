from PIL import Image
import numpy
import cv2

im = Image.open('a_image.tif')
im.show()
imarray = numpy.array(im)

cv2.imshow('image',imarray)
ret,thresh = cv2.threshold(imarray,127,255,cv2.THRESH_BINARY)
cv2.imshow('image threshold',thresh)

cv2.waitKey(0)
cv2.destroyAllWindows()


