# 导入计算机视觉库OpenCV，用于视频捕获和图像显示
import cv2
# 导入Ultralytics的YOLO库，用于加载预训练模型和执行姿态估计
from ultralytics import YOLO

# 1. 加载预训练的YOLO姿态估计模型
# 参数 'yolo11n-pose.pt' 指定模型文件路径
# 这是YOLO11系列中的纳米（nano）版本，专为姿态估计任务设计
# 该模型在COCO数据集上训练，能够检测人体的17个关键点
# 文件会自动从Ultralytics服务器下载（如果本地不存在）
model = YOLO('yolo11n-pose.pt')

# 2. 初始化视频捕获设备
# cv2.VideoCapture(0) 表示打开系统默认的摄像头
# 参数0通常对应内置摄像头或第一个连接的外部摄像头
# 如果要使用其他摄像头，可以尝试1、2等索引
# 也可以传入视频文件路径（如'video.mp4'）来处理视频文件
cap = cv2.VideoCapture(0)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("错误：无法打开摄像头")
    exit()  # 如果打开失败，则退出程序

# 在控制台显示操作提示信息
print("开始姿态估计... (按 'q' 退出)")

# 3. 进入主循环，持续从摄像头读取帧并进行处理
while True:
    # 从摄像头读取一帧图像
    # ret: 布尔值，表示帧是否成功读取（True表示成功）
    # frame: 读取到的图像数据，是一个三维numpy数组（高度×宽度×通道）
    ret, frame = cap.read()
    
    # 如果读取失败（如摄像头断开或视频结束），则跳出循环
    if not ret:
        print("错误：无法从摄像头读取帧")
        break  # 退出循环
    
    # 4. 使用YOLO模型对当前帧进行姿态估计推理
    # model(frame) 将当前图像帧传入模型进行推理
    # verbose=False 表示不输出详细的推理过程信息，保持控制台简洁
    # 返回值results是一个列表，包含所有检测结果
    results = model(frame, verbose=False)
    
    # 5. 可视化推理结果
    # results[0] 获取第一个（也是唯一一个）结果对象
    # .plot() 方法自动在图像上绘制：
    #   - 人体边界框（矩形框）
    #   - 17个关键点（如鼻、眼、肩、肘、腕等）
    #   - 骨架连接线（将关键点按人体结构连接）
    #   - 置信度分数（显示在边界框上方）
    # result_frame是绘制了姿态估计结果的新图像
    result_frame = results[0].plot()
    
    # 6. 显示处理后的图像
    # 创建一个名为'Pose Estimation'的窗口
    # 在该窗口中显示result_frame图像
    cv2.imshow('Pose Estimation', result_frame)
    
    # 7. 检查键盘输入，实现退出功能
    # cv2.waitKey(1) 等待1毫秒并检查是否有按键输入
    # & 0xFF 是位掩码操作，确保只取按键值的低8位
    # ord('q') 获取字符'q'的ASCII码值
    # 当按下'q'键时，条件成立，跳出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break  # 退出循环

# 8. 程序结束前的清理工作
# 释放VideoCapture对象，关闭摄像头连接
cap.release()
# 销毁所有OpenCV创建的窗口
cv2.destroyAllWindows()

# 在控制台显示程序结束信息
print("结束")

