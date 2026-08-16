# 钢球视觉识别

RDK X5 端程序：识别摆杆凹槽内钢球位置，通过串口向主控发送位置偏移和速度。本端不计算 PID，也不输出摆杆角度。

## 文件

| 文件 | 说明 |
|------|------|
| `rdk_ball_stable_v5.py` | 主程序：采集、识别、映射、串口发送 |
| `vision_tuner.py` | 上位机：实时调相机 / ROI / 识别 / 速度参数，可开关串口 |
| `ball_stable_config.json` | 运行配置（相机、ROI、几何、映射、串口） |
| `requirements.txt` | Python 依赖 |

## 依赖

```bash
pip3 install -r requirements.txt
```

中文预览另需字体，例如：

```bash
sudo apt install python3-pil fonts-wqy-zenhei
```

## 启动

先关掉占用摄像头或串口的旧进程：

```bash
pkill -f rdk_ball_stable
pkill -f vision_tuner
```

```bash
DISPLAY=:0.0 python3 -u rdk_ball_stable_v5.py          # 带预览
python3 -u rdk_ball_stable_v5.py --headless            # 无显示
python3 -u vision_tuner.py                             # 调参（串口默认关闭）
```

指定配置：`python3 -u rdk_ball_stable_v5.py --config /path/to.json`

调参器点「串口：关」才会开始发帧。左侧青框为 ROI，紫线为目标位置。

## 数据定义

```text
position = position_min
         + clamp((x_roi - map_x_left) / (map_x_right - map_x_left), 0, 1)
         × (position_max - position_min)

offset   = target_position - position
```

当前量程：`position` 0～448，默认目标 224（摆杆中点附近，以配置为准）。

- 球在目标左侧：`offset > 0`；右侧：`offset < 0`
- 向右运动：`velocity > 0`；向左：`velocity < 0`
- 只对速度做 EMA，不对位置滤波

## 串口协议

接线：RDK TX → 主控 RX，GND 共地。115200 8N1。

```text
seq,offset_px,speed_px_s,valid\n
```

| 字段 | 含义 | 约定 |
|------|------|------|
| `seq` | 帧序号，逐帧 +1，按 `0xFFFFFFFF` 回绕 | 主控按序号去重 |
| `offset_px` | 像素偏移 | **右正左负**，±200 |
| `speed_px_s` | 像素速度 | 向右为正，±20000 |
| `valid` | 本帧是否可靠 | `0` 时两个数值均为 0 |

内部 `offset` 为右负，发送前取负以匹配主控「右正」：

```text
offset_px  = clamp(round(-offset × px_per_unit), -200, 200)
speed_px_s = clamp(round(velocity × px_per_unit), -20000, 20000)
px_per_unit = (map_x_right - map_x_left) / (position_max - position_min)
```

主控必须先看 `valid`，不能把 `valid=0, offset=0` 当成球在中心。

## 识别状态

| 状态 | 含义 | 发送 |
|------|------|------|
| `TRACKING` | 当前帧可靠跟踪 | `valid=1` |
| `LOST` | 无候选 | `valid=0`（`hold_on_lost_s` 内可用上一速度外推） |
| `JUMP_REJECTED` | 候选跳变过大 | 不采用该帧位置 |

检测器可短暂保留上次位置用于下一帧选球，过期位置不会发给主控。

## 配置要点

`ball_stable_config.json` 主要段：

- `camera`：分辨率、曝光、增益等 UVC 参数
- `roi`：摆杆凹槽在画面中的裁剪区域
- `binary`：暗残差阈值与形态学核
- `geometry`：半径 / 面积 / 圆度筛选
- `v5`：边缘放宽、跳变门限随速度放大、丢球保持帧数
- `mapping`：ROI 像素 ↔ 位置量程、目标位置
- `signal`：速度 EMA、死区、最大 dt、丢球外推时间
- `serial`：`port` / `baudrate` / `enabled`

`signal` 建议：

| 项 | 作用 |
|----|------|
| `velocity_alpha` | 速度 EMA 新值权重，建议 0.2～0.4 |
| `velocity_deadband_unit_s` | 低于此值的速度发 0；静止抖动大时可试 1～5 |
| `max_dt_s` | 可靠检测间隔超过此值则速度置 0，避免遮挡恢复后尖峰 |
| `velocity_limit_unit_s` | 原始差分速度限幅 |
| `hold_on_lost_s` | 短时丢球外推时间，`0` 则立即 `valid=0` |

## 快捷键（主程序预览）

| 键 | 作用 |
|----|------|
| `M` | 显示/隐藏二值图 |
| `S` | 保存 ROI、标注图、掩码到 `stable_snapshots/` |
| `R` | 清空跟踪与速度状态 |
| `Q` / `Esc` | 退出 |

## 联调顺序

1. 主控先不驱电机，只打印解析出的 `valid / offset / velocity`
2. 球放目标左侧，确认 `offset > 0`；右侧确认 `offset < 0`
3. 向右移动确认 `velocity > 0`；向左确认 `velocity < 0`
4. 拿走钢球，确认 `valid=0` 且数值为 0
5. 方向与单位无误后再开摆杆 PID：先 P，再速度阻尼，最后按需加 I
