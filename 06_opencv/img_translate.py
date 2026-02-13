import cv2
import numpy as np
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    M = np.float32([[1, 0, 50], [0, 1, 50]])
    #对图像进行仿射变换
    shifted_image = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))
    #显示图片
    cv2.imshow('Image', shifted_image)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')