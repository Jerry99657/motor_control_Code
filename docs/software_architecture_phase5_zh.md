# 第五阶段软件架构重构说明

## 1. 重构目标

本阶段只调整应用层的软件边界，不改变以下实时行为：

- TIM13 仍以 10 ms 周期执行安全检查、麦轮控制、电机闭环和里程计。
- TIM6、TIM7、TIM16 的职责及中断优先级保持不变。
- 主循环仍以约 5 ms 周期运行。
- USB CDC、UART5、UART4 的协议格式和应答通道保持不变。
- Command Control 仍是远程运动命令的页面级权限门。

## 2. 模块结构

```text
main.c
  └─ app_runtime
       ├─ app_scheduler       前台任务顺序和周期
       ├─ app_event           跨模块异步事件队列
       ├─ app_boot            启动、Cache、LCD、QSPI和自检
       ├─ app_hal_bridge      UART/TIM/ADC HAL回调桥接
       ├─ command_control     Command Control状态和安全停机
       ├─ telemetry_service   USB文本命令和VOFA输出
       ├─ app_health          架构层健康快照
       └─ LVGL_App_Process    页面刷新和事件消费

comm_service
  └─ AppEvent_PostCommRx()
       └─ app_runtime事件分发
            ├─ LVGL命令协议解析
            └─ USB快速文本命令解析

lvgl_app
  └─ ui_navigation
       ├─ 页面请求
       ├─ 页面提交
       └─ 离页生命周期清理
```

## 3. 5.1与5.2：入口和调度

`main.c` 现在只负责 CubeMX 外设初始化、构造 `AppContext`、启动应用层以及调用 `AppRuntime_Process()`。启动动画、QSPI、LVGL、IMU、电机和看门狗初始化已经迁移到 `app_boot.c`。

`app_scheduler` 使用固定容量任务表，不进行动态内存分配。当前前台任务顺序与原主循环一致：

1. 通信接收
2. 应用事件分发
3. FOC链路
4. IMU
5. 摄像头
6. ADC和电池
7. 启动诊断
8. LVGL
9. VOFA遥测
10. CDC日志
11. 看门狗前台心跳

## 4. 5.3：Command Control独立模型

`command_control.c` 统一保存：

- Command Control是否激活。
- M1～M4设定速度。
- ESP32虚拟摇杆的LX、LY、RX、RY。
- 小陀螺开关和带方向的速度百分比。
- 接受、拒绝和停止命令计数。

进入页面调用 `CommandControl_Enter()`，它先激活权限门，再清零所有历史控制量。离开页面调用 `CommandControl_Leave()`，会依次关闭小陀螺、清零麦轮命令、停止四个电机并向UART4 FOC板发送STOP。

通信和遥测模块不再通过 `LVGL_App_CommandSetMotorSpeed()` 控制电机，避免“通信层依赖UI实现”的反向耦合。

## 5. 5.4：固定容量应用事件队列

UART5和USB CDC每次最多投递64字节的 `APP_EVENT_COMM_RX` 事件。队列深度为8，全部使用静态内存：

- UART/USB接收服务只负责收集字节和投递事件。
- `app_runtime` 在同一轮主循环中、LVGL处理之前消费事件。
- UART5和USB仍分别使用通道0和通道1，应答返回原通道。
- NES手机手柄的全局帧仍可在NES运行时处理，不受Command Control页面限制。
- 队列满时不会覆盖旧事件，而是增加丢弃计数，便于诊断通信拥塞。

正常情况下 `posted_count` 应等于 `handled_count`，`dropped_count` 应长期为0。

## 6. 5.5：页面导航和生命周期

`ui_navigation.c` 统一保存当前页面、待切换页面、请求次数和完成切换次数，并根据页面层级决定前进/后退动画方向。

离页生命周期钩子当前负责三个关键清理动作：

- 离开 Mecanum Control：停止未完成的轨迹和倒计时。
- 离开 MPU6500 Data：结束记录/标定相关的页面资源。
- 离开 Command Control：执行完整远程控制停机并关闭权限门。

因此以后增加新的返回路径时，只要通过 `UiNavigation_Request()` 切页，就不会绕过上述安全清理。

## 7. 5.6：架构健康诊断

`app_health.c` 汇总以下数据：

- 应用事件投递、处理、丢弃、当前积压和历史最高水位。
- UART5接收环形缓冲溢出次数和发送队列丢弃次数。
- Command Control接受、拒绝和停止次数。
- 调度器循环次数、任务调用次数和注册任务数。
- IWDG喂狗、漏票、栈保护和运行状态。

进入 `8 UI Diagnostics`，使用Left/Right切换到 `8/8 ARCHITECTURE`：

```text
Health GOOD    Flags 00000000
Events P/H/D ...
Q p/h ... UART ...
Cmd A/R/S ...
Nav C/P ... Tr/Re ...
Sched loop/call ...
```

健康标志定义：

| 位 | 含义 |
| ---: | --- |
| bit0 | 应用事件队列发生丢弃 |
| bit1 | UART5接收环形缓冲溢出 |
| bit2 | UART5发送队列丢弃 |
| bit3 | 看门狗双投票出现漏票 |
| bit4 | 栈保护字损坏 |

## 8. 上板回归建议

1. 开机后进入Architecture页，确认`Health GOOD`、`Flags 00000000`和事件丢弃为0。
2. 进入Command Control，通过UART5摇杆持续操作30秒，确认事件P/H同步增长且小车跟手。
3. 退出Command Control，确认四轮、小陀螺和FOC立即停止；继续发送遥控帧不应驱动车辆。
4. 运行NES手机手柄，确认NES帧在非Command Control页面仍有效。
5. 连续切换Mecanum、MPU6500、SD、Diagnostics和主菜单，确认没有HardFault，导航`Tr`持续增加。
6. 播放AVI/BIN并返回目录，确认事件队列和UART统计没有异常增长。
7. 在Reliability页确认IWDG feed持续增加、miss保持0、Stack guard为OK。

## 9. 扩展约束

- 中断中只采集数据、更新标志或投递小型事件，不执行LVGL、FatFs或阻塞式通信。
- 新的远程运动命令必须经过独立控制模型和Safety层，不直接修改页面静态变量。
- 新页面必须通过统一导航接口切换，并在生命周期钩子中注册资源释放和运动停止操作。
- 事件载荷超过64字节时应传递静态缓冲区句柄或拆包，不应扩大队列中的内联载荷来堆积大块数据。
