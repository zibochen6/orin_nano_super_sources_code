import cv2
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    #画线
    """
    cv2.rectangle(img, 左上角坐标, 右下角坐标, 颜色, 线粗)
    """
    cv2.rectangle(img, (10, 10), (100, 100), (255, 255, 0), 5)
    #显示图片
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')