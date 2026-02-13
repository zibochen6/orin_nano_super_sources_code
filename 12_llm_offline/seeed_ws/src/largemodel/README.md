# Largemodel 功能包使用指南

## 一、使用本地 Python 3.10 环境

### 1. 确认 Python 版本
```bash
# 检查当前 Python 版本
python3 --version

# 确认使用系统 Python
which python3
```

## 二、一次性装完所有依赖（编译 + 运行）

### 1. 安装 ROS 2 编译工具依赖
```bash
# 安装 ROS 开发工具，包含所有必要的编译依赖
sudo apt update && sudo apt install -y ros-dev-tools
```

### 2. 安装功能包业务依赖
直接装代码里用到的所有库：
```bash
# 进入功能包目录
cd /opt/seeed/development_guide/12_llm_offline/seeed_ws/src/largemodel

# 安装依赖（setup.py 里有 install_requires，会自动安装所有必要的包）
pip3 install -e .
```

## 三、编译工作空间

### 1. 进入工作空间根目录
```bash
cd /opt/seeed/development_guide/12_llm_offline/seeed_ws
```

### 2. 清理残留 + 编译
```bash
# 删除旧的编译产物（首次编译可跳过）
rm -rf build install log

# 使用系统 Python 编译
colcon build --packages-select largemodel
```

## 四、运行功能包（必须加载环境变量）

### 1. 加载编译生成的环境变量
```bash
source install/setup.bash
```

### 2. 启动 launch 文件
```bash
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
```

### 3. 注意：每次新开终端都需要设置环境
```bash
# 每次在新终端运行前都要执行
cd /opt/seeed/development_guide/12_llm_offline/seeed_ws && source install/setup.bash
```

## 五、配置文件说明

### 1. 模型路径配置
功能包使用 `config/large_model_interface.yaml` 文件存储模型路径和API配置：
- 本地ASR模型路径
- 中英文TTS模型路径
- 各AI平台API密钥配置

### 2. 路径修正
确保配置文件中的路径使用 `/opt/seeed/development_guide/12_llm_offline/` 而非 `/home/jetson/`，例如：
```yaml
local_asr_model: "/opt/seeed/development_guide/12_llm_offline/seeed_ws/src/largemodel/MODELS/asr/SenseVoiceSmall"
zh_tts_model: "/opt/seeed/development_guide/12_llm_offline/seeed_ws/src/largemodel/MODELS/tts/zh/zh_CN-huayan-medium.onnx"
```

## 关键注意事项

1. **环境变量**：每次编译后或在新终端中运行前，必须执行 `source install/setup.bash`，否则 ROS 找不到本地包；
2. **依赖补充**：遇到 `ModuleNotFoundError: xxx`，直接使用 `pip3 install xxx` 补充即可；
3. **路径配置**：确保配置文件中的路径与实际系统路径一致；
4. **权限问题**：如果遇到权限错误，检查并修复文件和目录权限。

## 技术实现细节

### 配置文件修复
- 更新了 `/opt/seeed/development_guide/12_llm_offline/seeed_ws/src/largemodel/config/large_model_interface.yaml` 中的模型路径
- 确保路径从 `/home/jetson/` 改为 `/opt/seeed/development_guide/12_llm_offline/`

### 依赖项管理
- 修改了 `setup.py` 文件，添加了所有必需的依赖项，包括：
  - 基本依赖：setuptools、ollama、pygame、opencv-python、dashscope、openai、piper-tts、funasr、websocket-client、pyyaml
  - 额外依赖：cryptography、distro、decorator
- 重新构建了ROS2包以应用更改：
  ```bash
  colcon build --packages-select largemodel
  ```

### 节点启动
- 成功启动了 `largemodel_control` 节点，启用了文本聊天模式：
  ```bash
  ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
  ```

### 运行状态
节点已成功初始化并运行，日志显示：
- `LargeModelService node Initialization completed...`
- `Using LLM platform: ollama`（或其他配置的平台）
- `TTS initialized with local model`
- 摄像头初始化失败（但这对文本聊天模式无影响）

现在系统已完全准备就绪，可以使用文本聊天功能了。

## 故障排查

### 常见错误及解决方法

1. **ModuleNotFoundError: No module named 'pygame'**
   - 解决：`pip3 install pygame`

2. **ModuleNotFoundError: No module named 'cryptography'**
   - 解决：`pip3 install cryptography`

3. **ModuleNotFoundError: No module named 'distro'**
   - 解决：`pip3 install distro`

4. **pkg_resources.DistributionNotFound: The 'decorator>=4.3.0'**
   - 解决：`pip3 install decorator`

5. **FileNotFoundError: [Errno 2] No such file or directory: '/home/jetson/...'**
   - 解决：修改配置文件中的路径为 `/opt/seeed/development_guide/12_llm_offline/seeed_ws/...` 并重新编译

6. **Package 'largemodel' not found**
   - 解决：执行 `source install/setup.bash` 设置环境变量

7. **Camera initialization failed**
   - 解决：检查摄像头连接，或在不需要摄像头时忽略此警告

### 验证方法

启动节点后，查看终端输出，确认以下信息：
- `LargeModelService node Initialization completed...`
- `Using LLM platform: ollama`（或其他配置的平台）
- `TTS initialized with local model`

如果看到这些信息，说明节点已成功启动，可以开始使用文本聊天功能。