# 导入必要的库
# OpenCV是一个开源的计算机视觉和机器学习软件库，提供图像和视频处理功能
import cv2
# 导入Ultralytics YOLO库，用于加载和使用YOLO深度学习模型进行物体检测
from ultralytics import YOLO

# 1. 加载预训练的YOLO定向物体检测模型
# YOLO()函数用于加载一个YOLO模型。这里指定模型文件为"yolo11n-obb.pt"
# "yolo11n"表示YOLO版本11的纳米（nano）模型，这是最小的模型，适合边缘设备
# "obb"代表Oriented Bounding Boxes（定向边界框），表示该模型支持检测带有方向的物体
# 如果本地没有该模型文件，程序会尝试从Ultralytics的官方服务器下载
model = YOLO("yolo11n-obb.pt")

# 2. 初始化视频捕获设备
# cv2.VideoCapture()用于从摄像头或视频文件中捕获视频
# 参数0表示使用计算机的默认摄像头（通常是内置摄像头或第一个外接摄像头）
cap = cv2.VideoCapture(0)

# 打印提示信息到控制台，指导用户如何操作程序
print("开始定向物体检测... (按 'q' 退出)")

# 3. 进入主循环，持续从摄像头捕获视频帧并进行处理
while True:
    # 从摄像头读取一帧图像
    # ret是一个布尔值，表示是否成功读取到帧（True表示成功）
    # frame是读取到的图像数据，是一个三维的numpy数组（高度×宽度×颜色通道）
    ret, frame = cap.read()
    
    # 检查帧是否读取成功
    # 如果读取失败（例如摄像头断开、视频文件结束等），则打印错误信息并跳出循环
    if not ret:
        print("错误：无法从摄像头读取帧")
        break  # 跳出while循环，结束程序
        
    # 4. 使用YOLO模型对当前帧进行推理（即进行物体检测）
    # model(frame)将当前图像帧传入模型，模型会返回检测结果
    # verbose=False表示不输出模型的详细推理信息，使控制台输出更简洁
    results = model(frame, verbose=False)

    # 5. 可视化检测结果
    # results是一个列表，通常包含一个元素（因为我们只传入了一帧图像）
    # results[0]获取第一个（也是唯一一个）检测结果对象
    # .plot()方法自动在图像上绘制检测结果，包括：
    #   - 定向边界框（旋转矩形框，而不仅仅是水平框）
    #   - 物体类别标签（如“car”、“person”等）
    #   - 置信度分数（表示模型对该检测的把握程度）
    # annotated_frame是绘制了检测结果的图像帧
    annotated_frame = results[0].plot()
    
    # 6. 在窗口中显示处理后的图像帧
    # cv2.resize()将图像大小调整为640x480像素，这可以确保窗口大小合适
    # cv2.imshow()创建一个名为"Result"的窗口，并在其中显示图像
    cv2.imshow("Result", cv2.resize(annotated_frame, (640, 480)))

    # 7. 检查键盘输入，实现退出功能
    # cv2.waitKey(1)等待1毫秒，并返回按键的ASCII码。如果没有按键，返回-1
    # & 0xFF是位掩码操作，只取返回值的低8位，确保兼容性
    # ord('q')返回字符'q'的ASCII码值
    # 当用户按下'q'键时，条件成立，跳出循环
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# 8. 程序结束前的清理工作
# 释放VideoCapture对象，关闭摄像头连接，释放摄像头资源
cap.release()
# 销毁所有由OpenCV创建的窗口，释放窗口资源
cv2.destroyAllWindows()

# 打印程序结束信息
print("检测结束")
