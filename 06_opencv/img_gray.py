import cv2
import numpy as np
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    #将图片转为灰度图
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #显示图片
    cv2.imshow('Image', gray_image)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')