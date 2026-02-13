import cv2
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    """
    cv2.putText(img, 文字, 位置, 字体, 字体大小, 颜色, 线粗, 线型)
    """
    text = "Hello, World!"
    cv2.putText(img, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),2,cv2.LINE_AA)
    #显示图片
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')