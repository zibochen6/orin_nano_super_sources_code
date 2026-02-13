import cv2
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    """
    cv2.ellipse(img, 椭圆中心点坐标, 椭圆的长短轴半径, 椭圆旋转角度, 绘制起始角度, 绘制结束角度, 颜色, 线粗)
    """
    cv2.ellipse(img, (100, 100), (20, 60), 0, 0, 360, (0, 255, 0), 2)
    #显示图片
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')