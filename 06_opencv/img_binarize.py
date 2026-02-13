import cv2
import numpy as np
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    #将图片转为二值图像
    _, binary_image = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    #显示图片
    cv2.imshow('Image', binary_image)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')