# STM32H743ZIT6_KIT 工程说明

## 1. 项目概述
本工程基于 STM32H743ZIT6，使用 STM32CubeMX + HAL + CMake 构建，围绕以下核心能力实现：

- LVGL 人机交互（按键导航 + 菜单页面）
- 四路直流电机闭环控制（速度环 + 位置环串级）
- 麦克纳姆底盘运动控制（平移/旋转/混合轨迹）
- MPU6500 姿态数据采集与航向角参与控制
- WS2812 灯带控制（定时器 PWM + DMA）
- SD 文件浏览与媒体相关功能
- VOFA+ 实时数据显示（速度、占空比）
- USB CDC 与 UART5 指令通道

---

## 2. 软件架构与主要模块

### 2.1 控制层级

- 底层执行层：`dc_motor_ol.c`
  - 负责编码器读取、速度测量、PWM 输出
  - 实现速度 PID、位置 PID（外环）以及串级控制
- 底盘运动层：`mecanum.c`
  - 将车体速度/位移指令解算为四轮目标
  - 提供航向角闭环（角度环）与混合模式（速度 + 距离）
- UI 与协议层：`lvgl_app.c`
  - 一级菜单、页面逻辑、键盘导航
  - 命令帧解析、页面联动控制
- 系统调度层：`main.c`
  - 初始化全部外设与模块
  - 周期任务调度（TIM6/TIM7/TIM13/TIM16）

### 2.2 定时任务分工

`HAL_TIM_PeriodElapsedCallback()` 中的任务分配：

- TIM6：`lv_tick_inc(1)`，LVGL 1ms 系统时基
- TIM7：`MJPEG_Scheduler_OnTim7Tick()`，媒体播放调度
- TIM13：`DCMotor_OL_Tick10ms()` + `Mecanum_Tick10ms()`，10ms 控制周期
- TIM16：ADC 采样与电压计算

---

## 3. LVGL 一级菜单功能（Main Menu）

一级菜单位于 `lvgl_app_show_main_menu()`，共 6 项：

1. `Motor Control`
2. `Command Control`
3. `SD Card Files`
4. `Mecanum Control`
5. `MPU6500 Data`
6. `WS2812 Control`

对应核心函数如下：

- 主菜单构建：`lvgl_app_show_main_menu()`
- 一级菜单事件分发：`lvgl_app_menu_event_cb()`
- 二级页面入口：
  - `lvgl_app_show_motor_control_menu()`
  - `lvgl_app_show_command_control()`
  - `lvgl_app_show_sd_browser()`
  - `lvgl_app_show_mecanum_control()`
  - `lvgl_app_show_mpu6500_data()`
  - `lvgl_app_show_ws2812_control()`

---

## 4. 一级页面功能说明

### 4.1 Motor Control

包含两个子功能：

- `Motor Speed`：四路电机速度百分比设定与回读
- `Servo Angle`：舵机角度控制

核心函数：

- `lvgl_app_show_motor_control_menu()`
- `lvgl_app_show_motor_speed_control()`
- `lvgl_app_motor_speed_send_cmd()`
- `lvgl_app_motor_speed_sync_actual()`

### 4.2 Command Control

用于接收并解析命令帧，转发到电机/底盘/舵机控制。

核心函数：

- `lvgl_app_show_command_control()`
- `lvgl_app_com_rx_cb()`
- `lvgl_app_cmd_parse()`

### 4.3 SD Card Files

用于浏览 SD 卡目录并进入媒体相关功能。

核心函数：

- `lvgl_app_show_sd_browser()`
- `lvgl_app_sd_scan_current_path()`
- `lvgl_app_sd_file_event_cb()`

### 4.4 Mecanum Control

用于设置底盘 X/Y/旋转速度与位移参数，并执行或停止麦轮混合控制。

核心函数：

- `lvgl_app_show_mecanum_control()`
- `lvgl_app_control_confirm_selected()`
- `Mecanum_MixedControl()`

### 4.5 MPU6500 Data

实时显示 MPU6500 采样与姿态相关信息。

核心函数：

- `lvgl_app_show_mpu6500_data()`
- `mpu6500_timer_cb()`

### 4.6 WS2812 Control

用于 RGB 实时调节与灯带验证显示。

核心函数：

- `lvgl_app_show_ws2812_control()`
- `lvgl_app_ws2812_key_cb()`
- `lvgl_app_ws2812_slider_cb()`
- `ws2812_update()`

---

## 5. VOFA+ 显示与通信链路

### 5.1 VOFA+ 数据上传

`VOFA_Task_Process()` 每 20ms（50Hz）发送一帧 JustFloat 数据：

- `speed[4]`：四个电机的实时转速 RPM
- `duty[4]`：四个电机的实际 PWM 占空比百分比
- 帧尾：`0x00 0x00 0x80 0x7f`

数据源函数：

- `DCMotor_OL_GetSpeedRpm()`
- `DCMotor_OL_GetDutyPercent()`

发送接口：

- `CDC_Transmit_FS()`

### 5.2 下行命令输入

- USB CDC：`CDC_Receive_FS()` 中分发给
  - `lvgl_app_com_rx_cb()`
  - `vofa_usb_rx_cb()`
- UART5：中断接收回调 `HAL_UART_RxCpltCallback()` 中分发给
  - `lvgl_app_com_rx_cb()`

简易调速命令（VOFA 输入）示例：

- `M1:50`：1号电机 50%
- `STOP`：四电机停止

---

## 6. 电机控制算法详解

控制算法核心文件：`dc_motor_ol.c`。

### 6.1 速度环（内环）

#### 6.1.1 采样与测速
每 10ms 从编码器计数差计算实际转速：

$$
\omega_{meas\_rpm} = \frac{\Delta count \times 60000}{N_{enc} \times T_s}
$$

其中：

- $N_{enc}=1000$（每圈脉冲数）
- $T_s=10ms$

#### 6.1.2 目标转速换算

$$
\omega_{ref\_rpm} = \frac{speed_{\%} \times RPM_{max\_target}}{100}
$$

#### 6.1.3 前馈 + PID
代码使用了前馈项 + PID：

$$
u_{ff} = \frac{\omega_{ref\_rpm} \times 100}{RPM_{no\_load}}
$$

$$
e = \omega_{ref\_rpm} - \omega_{meas\_rpm}
$$

$$
u = u_{ff} + K_p e + K_i \sum e + K_d(e - e_{k-1})
$$

并进行：

- 输出限幅：$u \in [-100,100]$
- 积分限幅：$I \in [-8000,8000]$
- 抗积分饱和：当输出饱和且误差继续推动饱和时暂停积分

当前参数：

- `Kp = 0.08`
- `Ki = 0.015`
- `Kd = 0.0`

---

### 6.2 位置环（外环）

位置环输出不是 PWM，而是“目标转速”，再交给速度环跟踪（典型串级控制）。

#### 6.2.1 误差与输出

$$
e_p = p_{ref} - p_{meas}
$$

$$
\omega_{ref\_rpm} = K_{p,p}e_p + K_{i,p}\sum e_p + K_{d,p}(e_p - e_{p,k-1})
$$

并加入速度上限约束（`speed_limit_percent`）：

$$
\omega_{ref\_rpm} \in [-\omega_{limit},\omega_{limit}]
$$

其中：

$$
\omega_{limit} = \frac{speed\_limit_{\%} \times RPM_{max\_target}}{100}
$$

#### 6.2.2 到位策略

- 位置误差死区：`|error| < 15 pulses` 直接输出 0，防止到位抖动

当前参数：

- `Kp = 0.5`
- `Ki = 0.0`
- `Kd = 0.0`

---

## 7. 角度环（航向环）算法详解

角度环位于底盘层 `mecanum.c` 的速度模式分支中，作用是维持车体航向或按设定角速度旋转。

### 7.1 目标角更新
当用户给出旋转速度命令 `wz_raw` 时，积分生成目标角：

$$
\theta_{ref}(k) = \theta_{ref}(k-1) + wz_{cmd} \cdot T_s
$$

并做 $[-180,180]$ 回绕。

### 7.2 角误差与 PI

$$
e_\theta = \theta_{ref} - \theta_{meas}
$$

对误差做最短角距离回绕后，计算：

$$
wz_{corr} = K_{p,\theta} e_\theta + K_{i,\theta} \int e_\theta dt
$$

工程实现中包含以下实用策略：

- 小误差死区：`|error| < 1.5°` 输出置零
- 积分限幅：`[-50,50]`
- 破静摩擦补偿：非零小输出拉到 `±12`
- 输出限幅：`[-100,100]`

当前参数：

- `Kp = 3.0`
- `Ki = 0.15`

---

## 8. 麦轮控制算法详解

控制核心：`Mecanum_MixedControl()` + `Mecanum_Tick10ms()`。

### 8.1 运动学逆解
先将旋转角速度换算为线速度补偿：

$$
V_w = \omega_{z,rad} \cdot K,\quad K = \frac{L+W}{2}
$$

四轮线速度：

$$
\begin{aligned}
V_1 &= V_x + V_y + V_w\\
V_2 &= -V_x + V_y + V_w\\
V_3 &= -V_x - V_y + V_w\\
V_4 &= V_x - V_y + V_w
\end{aligned}
$$

再通过 `Mecanum_HW_SetSpeed()` 转换为电机百分比命令。

### 8.2 混合模式（速度 + 距离）
当输入包含 `dx/dy/dw` 任一非零时，进入混合轨迹模式：

- 有距离约束的轴：按剩余距离积分推进，步进量受 `dt=10ms` 与当前速度限制
- 无距离约束的轴：保留本次速度偏移
- 每周期更新四轮目标脉冲：`DCMotor_OL_SetTargetPosition()`
- 所有有界轴到达后，自动清零并切回速度模式，避免残余自转

---

## 9. 关键函数速查（按用途）

### 系统与调度

- `main()`：模块初始化与主循环
- `HAL_TIM_PeriodElapsedCallback()`：控制任务调度
- `VOFA_Task_Process()`：VOFA+ 数据帧上传

### 电机与底盘

- `DCMotor_OL_Init()`
- `DCMotor_OL_Tick10ms()`
- `DCMotor_OL_SetSpeed()`
- `DCMotor_OL_SetTargetPosition()`
- `Mecanum_MixedControl()`
- `Mecanum_Tick10ms()`

### 界面与命令

- `LVGL_App_Init()`
- `LVGL_App_Process()`
- `lvgl_app_show_main_menu()`
- `lvgl_app_menu_event_cb()`
- `lvgl_app_com_rx_cb()`
- `lvgl_app_cmd_parse()`

### 灯带

- `ws2812_Init()`
- `ws2812_update()`
- `ws2812_set_all()`

---

## 10. 构建与运行

### 10.1 构建
建议使用 CMake 方式：

- 生成并进入构建目录（如 `build/Debug`）
- 执行 Ninja 构建

### 10.2 运行前检查

- TIM13 中断正常（10ms 控制周期）
- 编码器计数方向与电机方向一致
- USB CDC 已枚举（VOFA+ 才能接收数据）
- WS2812 使用 TIM15 CH2 + DMA，初始化后应可见测试灯

---

## 11. 后续可优化项（建议）

- 给速度环加入转速前馈线性标定表，改善低速一致性
- 角度环增加 D 项或观测器，进一步降低动态超调
- 麦轮混合模式支持 S 曲线速度规划，减少冲击
- VOFA+ 增加航向角、位置误差、控制输出通道，提升调参效率

---

## 12. 串口/CDC 指令全集（本工程当前实现）

本工程存在两套下行命令协议：

- 文本协议（主要用于 VOFA+ 串口助手快速控制）
- 二进制帧协议（`0x77 0x68` 帧头）

### 12.1 命令入口与通道

#### 12.1.1 文本协议入口

- 入口函数：`vofa_usb_rx_cb()`（`Core/Src/main.c`）
- 数据来源：`CDC_Receive_FS()`（USB CDC）
- 说明：当前仅在 USB CDC 路径调用，不走 UART5。

#### 12.1.2 二进制帧协议入口

- 入口函数：`lvgl_app_com_rx_cb()` -> `lvgl_app_cmd_parse()`（`Drivers/User/Src/lvgl_app.c`）
- 数据来源：
  - USB CDC：`CDC_Receive_FS()` 中调用 `lvgl_app_com_rx_cb()`
  - UART5：`HAL_UART_RxCpltCallback()` 中调用 `lvgl_app_com_rx_cb()`
- 重要限制：仅当当前页面是 `Command Control` 时生效（`s_ctrl_page == LVGL_APP_CTRL_PAGE_COMMAND`）。

### 12.2 文本协议指令

#### 12.2.1 单电机速度设置

- 格式：`M<电机号>:<速度百分比>`
- 范围：
  - 电机号：`1~4`
  - 速度：`-100~100`（超出自动截断）
- 示例：
  - `M1:50` -> 1号电机 +50%
  - `M3:-30` -> 3号电机 -30%

#### 12.2.2 全部停止

- 指令：`STOP`
- 行为：调用 `DCMotor_OL_StopAll()`。

### 12.3 二进制帧协议通用格式

通用帧结构（按当前实现）：

- `Byte0`：`0x77`
- `Byte1`：`0x68`
- `Byte2`：`LEN`（整帧长度，包含头尾）
- `Byte3`：`DEV_ID`
- `Byte4`：`CMD`
- `Byte5..`：负载
- `Byte(LEN-1)`：`0x0A`

解析规则：

- `LEN` 允许范围：`0x04~0x10`
- 尾字节必须是 `0x0A`
- 若帧头不对或长度异常，接收缓冲会移位丢弃并继续找同步。

### 12.4 二进制写命令（`CMD=0x02`）

#### 12.4.1 四电机同时写入

- 识别条件：`DEV_ID=0x01` 且 `LEN=0x0A`
- 负载：
  - `Byte5~Byte8`：4 路电机速度（`int8`，对应 M1~M4）
- 示例：
  - `77 68 0A 01 02 0A F6 14 00 0A`
  - 含义：M1=+10, M2=-10, M3=+20, M4=0

#### 12.4.2 单电机写入

- 识别条件：`DEV_ID=0x02` 且 `LEN>=0x08`
- 负载：
  - `Byte5`：端口号（1~4）
  - `Byte6`：速度（`int8`）
- 示例：
  - `77 68 08 02 02 01 32 0A`
  - 含义：1号电机 +50%

#### 12.4.3 麦轮混合控制写入

- 识别条件：`DEV_ID=0x03` 且 `LEN>=0x0D`
- 负载：
  - `Byte5`：模式 `mode`
    - `0`：速度模式
    - `1`：距离模式
  - `Byte6~7`：`Vx`（`int16`, little-endian）
  - `Byte8~9`：`Vy`（`int16`, little-endian）
  - `Byte10~11`：`Wz`（`int16`, little-endian）
- 行为：
  - `mode=0`：`Mecanum_MixedControl(Vx,Vy,Wz,0,0,0)`
  - `mode=1`：`Mecanum_MixedControl(100,100,100,Vx,Vy,Wz)`（以固定默认速度执行距离）
- 示例（速度模式）：
  - `77 68 0D 03 02 00 64 00 00 00 14 00 0A`
  - 含义：Vx=100, Vy=0, Wz=20

#### 12.4.4 舵机角度写入

- 识别条件：`DEV_ID=0x05` 且 `LEN>=0x09`
- 负载：
  - `Byte5`：舵机端口（当前有效 1~2）
  - `Byte7`：角度（0~180）
- 行为：内部映射到 0~270：`target_angle = angle * 3 / 2`
- 示例：
  - `77 68 09 05 02 01 00 5A 0A`
  - 含义：1号舵机设为 90°（内部映射为 135）

### 12.5 二进制读命令（`CMD=0x01`）

在处理读命令前，系统会先刷新一次电机实际转速：`lvgl_app_motor_speed_sync_actual()`。

#### 12.5.1 四路编码器/速度读取

- 请求条件：`DEV_ID=0x03`
- 回复格式（长度 `0x0A`）：
  - `77 68 0A 03 01 s1 s2 s3 s4 0A`
  - 其中 `s1~s4` 为 4 路实际速度（按 `uint8` 打包）。

#### 12.5.2 单路编码器/速度读取

- 请求条件：`DEV_ID=0x04`
- 请求负载：`Byte5=port(1~4)`
- 回复格式（长度 `0x08`）：
  - `77 68 08 04 01 port speed 0A`

#### 12.5.3 回复发送通道注意事项

当前实现中，读命令回包固定使用 `HAL_UART_Transmit(&huart5, ...)`，即通过 UART5 发出。

这意味着：

- 即使请求来自 USB CDC，回包也不会经 CDC 返回，而是走 UART5。
- 如果上位机通过 USB CDC 发读命令却没接 UART5，会看到“无回包”。

### 12.6 虚拟摇杆帧（特殊分支）

存在一个特殊分支：当 `DEV_ID=0x0C` 且 `LEN=0x0A` 时，按虚拟摇杆解释：

- `Byte4~Byte7`：`LX, LY, RX, RY`（`int8`）
- 映射关系：
  - `wz = LX * 2.0`
  - `vy = RX * 10.0`
  - `vx = RY * 10.0`
- 执行：`Mecanum_MixedControl(vx, vy, wz, 0, 0, 0)`

注意：该分支优先于 `CMD=0x02/0x01` 的普通解析逻辑。

### 12.7 使用建议

- 需要二进制协议生效时，先进入 LVGL 的 `Command Control` 页面。
- 需要读命令回包时，请连接并监听 UART5。
- 文本协议（`M1:50`/`STOP`）优先用于快速调试，不依赖 Command 页面。
