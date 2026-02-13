# 导入OpenCV库，用于视频捕获和图像显示
import cv2
# 导入Ultralytics YOLO库，用于加载模型和执行物体检测
from ultralytics import YOLO

# 1. 加载预训练YOLO模型
# 使用YOLO类加载模型文件，参数为模型文件的路径
# 这里加载的是YOLO11纳米模型（yolo11n.pt），它是YOLO系列中最轻量级的版本，适合边缘设备和实时应用
# 如果要追求更高精度，可以替换为：'yolo11s.pt'（小型）、'yolo11m.pt'（中型）或'yolo11x.pt'（大型）
model = YOLO('yolo11n.pt')

# 2. 初始化视频捕获设备
# 创建VideoCapture对象，参数0表示使用系统默认的摄像头（通常是内置或第一个连接的USB摄像头）
# 如果要使用其他摄像头，可以尝试1、2等索引，或者使用视频文件路径（如'video.mp4'）
cap = cv2.VideoCapture(0)

# 检查摄像头是否成功打开，如果失败则打印错误信息并退出
if not cap.isOpened():
    print("错误：无法打开摄像头")
    exit()

# 打印操作提示到控制台
print("开始实时检测... (按 'q' 退出)")

# 3. 进入主循环，持续从摄像头捕获帧并进行处理
while True:
    # 从摄像头读取一帧图像
    # success: 布尔值，表示帧是否成功读取（True/False）
    # frame: 读取到的图像帧，是一个三维的numpy数组（高度，宽度，通道）
    success, frame = cap.read()
    
    # 如果帧读取失败（如摄像头断开），则跳出循环
    if not success:
        print("错误：无法从摄像头读取帧")
        break
    
    # 4. 使用YOLO模型对当前帧进行推理
    # model(frame)将当前帧传递给模型进行检测
    # verbose=False: 禁用模型内部的详细日志输出，保持控制台整洁
    # 返回值results是一个Results对象列表，包含检测到的目标信息（边界框、置信度、类别等）
    results = model(frame, verbose=False)
    
    # 5. 可视化检测结果
    # results[0]获取第一个（也是唯一一个）Results对象
    # .plot()方法自动在图像上绘制检测结果，包括：
    #   - 边界框（矩形）
    #   - 类别标签（如'person', 'car'）
    #   - 置信度分数（0-1，表示检测的可信程度）
    # annotated_frame是绘制了检测结果的新图像帧
    annotated_frame = results[0].plot()
    
    # 6. 显示处理后的图像帧
    # 创建一个名为'Result'的窗口，并显示annotated_frame图像
    cv2.imshow('Result', annotated_frame)
    
    # 7. 检查键盘输入，等待1毫秒
    # waitKey(1)等待1毫秒并检查是否有按键输入
    # 0xFF是位掩码，用于确保只取按键值的低8位
    # ord('q')获取字符'q'的ASCII值
    # 如果按下'q'键，则跳出循环，结束程序
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 8. 释放资源并清理窗口
# 释放VideoCapture对象，关闭摄像头连接
cap.release()
# 销毁所有OpenCV创建的窗口
cv2.destroyAllWindows()

# 打印结束信息到控制台
print("检测结束")
