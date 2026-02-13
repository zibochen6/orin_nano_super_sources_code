import cv2
#读取本地图片
img = cv2.imread('./images/test.png')

if img is not None:
    #修改图片的尺寸
    img_resize = cv2.resize(img, (100, 100))
    #保存修改的图片
    img_resize_path = './images/test_resize.png'
    cv2.imwrite(img_resize_path, img_resize)
    #显示修改过的图片
    cv2.imshow('Image_resize', img_resize)
    cv2.imshow('Image_original', img)
    cv2.waitKey(0)  
    cv2.destroyAllWindows()
else:
    print('Image not found')