# 逐飞 MSPM0G3507 电赛工程库

基于逐飞（SeekFree）MSPM0G3507 开发板的电子设计竞赛参赛工程，题目为 **2026 年电赛 H 题：车载平衡滚球运动控制系统**。

## 简介

本仓库包含底车主控程序、外设插件驱动、TI / 逐飞 SDK、PCB 文件、机械结构件，以及钢球视觉识别程序。用于实现小车循线、运动计算与球平衡控制：主控负责循迹与摆杆闭环，RDK X5 负责识别钢球位置并通过串口发送偏移量和速度。

## 目录结构

```
diansai-MBH/
├── 电赛m0/                       # 主控工程与 SDK
│   ├── project/                  # 用户工程
│   │   ├── user/src/main.c       # 主程序（任务调度、循线、显示）
│   │   ├── user/src/isr.c        # 中断服务
│   │   └── keil/hareware/        # 底层硬件驱动（电机/编码器/循迹/陀螺仪等）
│   ├── libraries/                # 逐飞 + TI SDK
│   │   ├── zf_device/            # 外设插件驱动（IMU/OLED/TFT/按键等）
│   │   ├── zf_driver/            # 底层驱动（GPIO/PWM/UART/SPI 等）
│   │   ├── zf_common/            # 通用库
│   │   └── sdk/                  # TI MSPM0 SDK + CMSIS
│   └── ProPrj_*.epro2            # PCB 文件
├── 视觉程序/                      # RDK X5 钢球视觉识别
│   ├── rdk_ball_stable_v5.py     # 主程序：识别钢球并串口发送
│   ├── vision_tuner.py           # 上位机：实时调参与串口联调
│   ├── ball_stable_config.json   # 相机 / ROI / 识别 / 映射 / 串口配置
│   ├── requirements.txt          # Python 依赖
│   └── README.md                 # 视觉端协议与启动说明
└── 结构件/                       # 机械结构件（SolidWorks）
```

## 功能特性

- **循线**：12 路循迹传感器 + PID 循线控制
- **运动计算**：编码器测速 + JY901P 陀螺仪航向 + PID 速度环
- **钢球视觉**：ROI 暗残差识别 + 几何筛选 + 跳变门限，输出位置偏移与速度
- **串口联调**：RDK → 主控 ASCII CSV 协议，调参器可实时开关发送
- **TFT 显示**：TFT180 屏实时显示任务、计时、球位、航向、目标值、速度等
- **任务调度**：按键选择 H 题的 6 个子任务

## 硬件清单

- 主控：逐飞 MSPM0G3507
- 屏幕：TFT180（1.8 寸 TFT）
- 循迹：12 路循迹模块
- 运动：编码器电机 ×2、JY901P 陀螺仪
- 交互：按键 ×4
- 视觉：RDK X5 + USB 摄像头（安装于摆杆凹槽上方，画面覆盖整根摆杆）
- 通信：UART 115200 8N1（RDK TX → 主控 RX，GND 共地）

## 主程序说明

主程序（`project/user/src/main.c`）实现 H 题 6 个任务的调度：

| 任务 | 内容 | 状态 |
|------|------|------|
| 1 | 图传装置验证 | 待实现 |
| 2 | 循线一圈（A→A，≤20s，停车≤2cm） | 已实现 |
| 3 | 静止摆球控制（O→±5cm，误差≤1cm） | 待实现 |
| 4 | A→B 循线 + 球稳定在中心（≤8s） | 待实现 |
| 5 | A→A 一圈 + 球稳定在中心（≤30s） | 待实现 |
| 6 | A→A 一圈 + 球稳定在指定位置（≤30s） | 待实现 |

赛题要求：凹槽内钢球位置检测必须采用摄像头；图传发送模块须固定在车上，回传画面能覆盖整根摆杆。

## 视觉程序说明

视觉端跑在 RDK X5 上，不计算 PID、不输出杆角度。流程：

1. 摄像头采集后立即裁剪 ROI（摆杆凹槽区域）
2. 暗残差二值化 + 形态学 + 圆度/面积筛选，得到钢球横坐标
3. 将 ROI 横坐标线性映射为位置量 `position`（当前量程 0～448，对应摆杆刻度）
4. 计算 `offset = target_position - position`，并对速度做轻量 EMA
5. 按协议发给主控，由主控做摆杆 PID 与电机控制

### 数据定义

```text
position = position_min
         + clamp((x_roi - map_x_left) / (map_x_right - map_x_left), 0, 1)
         × (position_max - position_min)

offset   = target_position - position
```

- 球在目标左侧：`offset > 0`；右侧：`offset < 0`；对准目标：`offset ≈ 0`
- 向右运动：`velocity > 0`；向左：`velocity < 0`
- 只对速度做 EMA，不对位置做滤波，避免一像素抖动被高帧率放大成速度尖峰

### 串口协议

```text
seq,offset_px,speed_px_s,valid\n
```

| 字段 | 含义 | 范围 / 约定 |
|------|------|-------------|
| `seq` | 帧序号，逐帧 +1，按 `0xFFFFFFFF` 回绕 | 主控按序号去重 |
| `offset_px` | 像素偏移，**右正左负** | ±200 |
| `speed_px_s` | 像素速度，向右为正 | ±20000 |
| `valid` | `1` 本帧可靠跟踪；`0` 丢球或跳变拒绝 | `valid=0` 时两个数值均为 0 |

视觉内部 `offset = target - position` 为右负，发送前取负以匹配主控「右正」约定：

```text
offset_px  = clamp(round(-offset × px_per_unit), -200, 200)
speed_px_s = clamp(round(velocity × px_per_unit), -20000, 20000)
px_per_unit = (map_x_right - map_x_left) / (position_max - position_min)
```

主控必须先看 `valid`，不能把 `valid=0` 的 `offset=0` 当成「球正好在中心」。

### 识别状态

| 状态 | 含义 | 发送 |
|------|------|------|
| `TRACKING` | 当前帧可靠跟踪 | `valid=1`，带真实 offset / velocity |
| `LOST` | 无候选 | `valid=0`，数值清零（短时可用上一速度外推，见 `hold_on_lost_s`） |
| `JUMP_REJECTED` | 候选跳变过大 | 不采用该帧位置，避免用过期数据驱动电机 |

### 启动

在 RDK X5 上：

```bash
cd 视觉程序
DISPLAY=:0.0 python3 -u rdk_ball_stable_v5.py          # 带预览
python3 -u rdk_ball_stable_v5.py --headless            # 无显示
python3 -u vision_tuner.py                             # 调参上位机（串口默认关闭）
```

依赖：`opencv-python`、`numpy`、`pyserial`；调参界面另需 `Pillow`。摄像头占用冲突时先关掉旧进程：

```bash
pkill -f rdk_ball_stable
pkill -f vision_tuner
```

调参器左侧为完整画面（青框为 ROI，紫线为目标位置），右侧可调曝光、二值阈值、ROI、映射和速度滤波。点「串口：关」才会开始向主控发帧。

联调建议：主控先不驱电机，只打印解析结果 → 球放目标左侧确认 offset 为正、右侧为负 → 右移确认 velocity 为正 → 拿走钢球确认 `valid=0` → 再启摆杆 PID。

## 使用说明

1. 使用 Keil MDK 打开 `电赛m0/project/keil/Myzhufei.uvprojx`
2. 编译并下载到 MSPM0G3507 开发板
3. 上电后通过按键选择任务：`KEY_1`/`KEY_2` 切换、`KEY_3` 启动、`KEY_4` 复位
4. 视觉端按上一节启动，确认串口接线与 `ball_stable_config.json` 中的 `serial.port`
