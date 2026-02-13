import cv2
import numpy as np
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    """
    cv2.polylines(img, 点集, 是否闭合,颜色, 线粗)
    """
    points = np.array( [(0, 15), (50, 70), (111, 125), (35, 125)], np.int32)
    points = points.reshape((-1, 1, 2))
    cv2.polylines(img, [points], True, (0, 255, 0), 5)
    #显示图片
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')