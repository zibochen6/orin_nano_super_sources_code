import cv2
from ultralytics import YOLO

# 1. 加载分类模型
model = YOLO('yolo11n-cls.pt')

# 2. 打开摄像头
cap = cv2.VideoCapture(0)

print("开始实时图像分类... (按 'q' 退出)")

while True:
    # 3. 读取视频帧
    ret, frame = cap.read()
    if not ret:
        break
    
    # 4. 执行分类推理
    results = model(frame, verbose=False)
    
    # 5. 获取分类结果
    result = results[0]
    if hasattr(result, 'probs'):
        top1_index = result.probs.top1
        top1_conf = result.probs.top1conf
        top1_class = result.names[top1_index]
        
        # 6. 在帧上显示结果
        text = f"{top1_class}: {top1_conf:.2%}"
        cv2.putText(frame, text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # 7. 显示帧
    cv2.imshow('Real-time Classification', frame)
    
    # 8. 按'q'退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 9. 释放资源
cap.release()
cv2.destroyAllWindows()
print("结束")
