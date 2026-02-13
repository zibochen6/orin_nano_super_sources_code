import cv2
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    #保存图片
    cv2.imwrite('./images/test_write.png', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')