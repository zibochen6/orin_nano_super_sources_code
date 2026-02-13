import cv2
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    #修改图片数组的值
    img[:100,:100] = [255,255,255]
    #显示图片
    cv2.imshow('Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')