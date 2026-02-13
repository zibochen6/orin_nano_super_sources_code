import cv2
import numpy as np
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    #检查图像的边缘
    edges = cv2.Canny(img, 100, 255)
    #显示图片
    cv2.imshow('Image', edges)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')