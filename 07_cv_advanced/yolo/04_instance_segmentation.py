# 导入OpenCV库，用于视频捕获和图像显示
import cv2
# 导入Ultralytics YOLO库，用于加载模型和执行实例分割
from ultralytics import YOLO

# 1. 加载预训练的YOLO实例分割模型
# 注意：模型文件后缀为'-seg'，表示这是专门用于分割的模型。
# 'yolo11n-seg.pt'是纳米级别的分割模型，适合边缘设备实时运行。
# 可根据精度和速度需求替换为：
# 'yolo11s-seg.pt' (小型), 'yolo11m-seg.pt' (中型), 'yolo11l-seg.pt' (大型), 'yolo11x-seg.pt' (超大型)
model = YOLO('yolo11n-seg.pt')

# 2. 初始化视频捕获设备
# 参数0代表系统默认摄像头。可替换为视频文件路径（如'video.mp4'）进行文件分析。
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("错误：无法打开摄像头")
    exit()

print("开始实时实例分割... (按 'q' 键退出)")

# 3. 进入主循环，持续处理视频流
while True:
    # 从摄像头读取一帧
    success, frame = cap.read()
    if not success:
        print("错误：无法从摄像头读取帧")
        break

    # 4. 使用YOLO分割模型对当前帧进行推理
    # `verbose=False` 关闭模型内部的冗长日志，保持控制台输出简洁。
    # 返回值 `results` 是一个列表，包含检测和分割的所有结果。
    results = model(frame, verbose=False)

    # 5. 可视化结果
    # `results[0].plot()` 方法自动完成所有可视化工作：
    #   a) 绘制每个物体实例的**边界框**。
    #   b) 在边界框上方显示**类别标签**和**置信度**。
    #   c) 用半透明的彩色**掩码**覆盖在每个物体实例的精确轮廓区域上。
    #      不同实例的掩码颜色不同，易于区分。
    annotated_frame = results[0].plot()

    # 6. （可选）访问原始的分割结果数据进行自定义处理
    # 如果需要对掩码、边界框等数据进行更复杂的操作，可以像下面这样访问：
    # result = results[0]  # 获取第一个结果对象
    # boxes = result.boxes   # 边界框数据 (xyxy, conf, cls)
    # masks = result.masks   # 分割掩码数据
    # class_ids = result.boxes.cls  # 类别ID
    # 例如，可以计算某类物体的像素面积，或根据自定义规则过滤结果。

    # 7. 显示处理后的帧
    cv2.imshow('YOLO Instance Segmentation', annotated_frame)

    # 8. 监听键盘输入，按 'q' 退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 9. 释放资源并清理窗口
cap.release()
cv2.destroyAllWindows()

print("实例分割结束")
