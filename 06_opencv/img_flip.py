import cv2
import numpy as np
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    #对图像进行视频镜像
    flipped_image = cv2.flip(img, 1)
    #显示图片
    cv2.imshow('Image', flipped_image)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')