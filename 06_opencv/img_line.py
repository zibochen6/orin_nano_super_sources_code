import cv2
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    """
    cv2.line(img, 起点坐标, 终点坐标, 颜色, 线粗)
    """
    cv2.line(img, (50, 150), (700, 150), (0, 0, 255), 5)
    #显示图片
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')