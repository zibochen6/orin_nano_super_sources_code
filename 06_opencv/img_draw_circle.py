import cv2
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    """
    cv2.circle(img, 圆心坐标, 半径长度, 颜色, 线粗)
    """
    cv2.circle(img,  (50, 50), 30, (0, 0, 255), 2)
    #显示图片
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')