import cv2
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    #裁切图像部分区域
    img_cropped = img[50:100,100:150]
    #显示图片
    cv2.imshow('Image_cropped', img_cropped)
    cv2.imshow('Image_original', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')