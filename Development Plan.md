# ESP32-S3智能陪护手环软件开发计划

## 项目概览

**项目名称**: ESP32-S3多功能智能手环
**开发周期**: 4周（28天）
**开发框架**: ESP-IDF 5.2.3
**目标人群**: ESP-IDF初学者
**项目重点**: 功能完整性，确保所有硬件模块可演示

---

## 硬件模块清单（基于原理图分析）

| 模块 | 型号/接口 | GPIO引脚 | 功能描述 |
|------|----------|---------|---------|
| 主控芯片 | ESP32-S3 | - | 双核处理器，支持Wi-Fi/BLE |
| OLED显示屏 | 1.3" OLED | SDA(IO8), SCL(IO9) | I2C显示，用户界面 |
| 心率血氧传感器 | MAX30102 | SDA(IO8), SCL(IO9), INT(IO5) | I2C心率/血氧检测 |
| 温度传感器 | DS18B20 | DQ(IO4) | 1-Wire体温测量 |
| 加速度陀螺仪 | MPU6050 | SDA(IO8), SCL(IO9), INT(IO46) | I2C运动检测 |
| 麦克风 | INMP441 | SCK(IO15), SD(IO16), WS(IO17) | I2S音频采集 |
| 扬声器放大器 | MAX98357A | BCLK(IO16), LRCLK(IO17), DIN(IO18) | I2S音频输出 |
| 用户按键 | 4个按键 | SW1(报警), SW2(电源), SW3(复位), BOOT(IO0) | 用户交互 |
| 状态LED | 3个LED | LED1(IO6,电源), LED2(IO2,充满), LED3(IO1,充电) | 状态指示 |
| 电池管理 | TP4056 | ADC(IO38), CHRG, STDBY | 充电管理和电量检测 |
| USB接口 | USB Type-C | D+, D- | 充电和串口通信 |
| 3.3V稳压器 | RT9013-33GB | - | 电源管理 |

**I2C总线共享**: OLED、MAX30102、MPU6050共享I2C总线(SDA=IO8, SCL=IO9)，需实现I2C地址管理。

**新增核心功能**:
1. **微信小程序交互**: 通过BLE与微信小程序通信，实时上传心率、血氧、步数等健康数据，接收报警通知
2. **语音识别功能**: 支持语音命令"查询心率"、"查询步数"、"呼叫家人"，实现语音播报和紧急呼叫
3. **语音播报**: 通过扬声器播报查询结果（心率、步数等）
4. **远程报警**: 按下报警键或语音"呼叫家人"时，通过小程序通知紧急联系人

---

## 四周开发计划

### 第一周（Day 1-7）：基础驱动层开发

**目标**: 完成硬件抽象层，实现基本通信协议和关键传感器驱动

#### Day 1-2: 开发环境搭建与I2C总线驱动
- **任务1.1**: 验证ESP-IDF开发环境
  - 编译并烧录Hello World程序
  - 验证串口输出和调试功能
  - 熟悉`idf.py build/flash/monitor`命令

- **任务1.2**: 实现I2C主机驱动 [components/drivers/i2c/i2c_master.c](components/drivers/i2c/i2c_master.c)
  - 初始化I2C总线（SDA=IO8, SCL=IO9, 400kHz）
  - 实现读写函数：`i2c_write_byte()`, `i2c_read_bytes()`
  - 实现设备扫描函数：`i2c_scan_devices()`
  - 配置驱动层CMakeLists.txt，将I2C驱动集成到drivers组件
  - 测试：扫描并打印I2C总线上的所有设备地址

**交付物**:
- 可编译通过的I2C驱动代码
- 能够识别I2C总线上的3个设备（OLED、MAX30102、MPU6050）

#### Day 3: OLED显示驱动开发
- **任务1.3**: 实现OLED显示驱动 [components/drivers/oled/oled.c](components/drivers/oled/oled.c)
  - 集成SH1106驱动库（推荐使用ESP-IDF组件或开源库）
  - 实现初始化函数：`oled_init()`
  - 实现基本绘图函数：
    - `oled_clear()` - 清屏
    - `oled_show_string(x, y, text)` - 显示字符串
    - `oled_show_number(x, y, num)` - 显示数字
    - `oled_refresh()` - 刷新显示缓冲区
  - 更新drivers组件CMakeLists.txt，添加oled驱动编译
  - 测试：显示"ESP32-S3 Wristband"和实时计数

**交付物**:
- OLED能正常显示文字和数字
- 提供演示代码展示基本UI框架

#### Day 4: DS18B20温度传感器驱动
- **任务1.4**: 实现1-Wire协议驱动 [components/drivers/onewire/onewire.c](components/drivers/onewire/onewire.c)
  - 实现1-Wire时序控制（复位、写位、读位）
  - 实现CRC8校验函数

- **任务1.5**: 实现DS18B20驱动 [components/drivers/ds18b20/ds18b20.c](components/drivers/ds18b20/ds18b20.c)
  - 初始化函数：`ds18b20_init()`
  - 读取温度函数：`ds18b20_read_temperature()`（返回浮点数，单位℃）
  - 测试：每秒读取并在OLED上显示体温数据

**交付物**:
- 能够实时读取并显示温度（精度0.1℃）

#### Day 5-6: MPU6050运动传感器驱动
- **任务1.6**: 实现MPU6050驱动 [components/drivers/mpu6050/mpu6050.c](components/drivers/mpu6050/mpu6050.c)
  - 初始化MPU6050（I2C地址0x68）
  - 读取6轴原始数据（加速度+陀螺仪）：
    - `mpu6050_read_accel()` - 读取加速度（X, Y, Z）
    - `mpu6050_read_gyro()` - 读取角速度（X, Y, Z）
  - 配置量程和滤波参数
  - 测试：在OLED上实时显示X/Y/Z轴加速度值

- **任务1.7**: 实现基础姿态计算
  - 计算俯仰角和横滚角（使用三角函数）
  - 实现简单卡尔曼滤波或互补滤波
  - 测试：摇晃手环观察角度变化

**交付物**:
- 输出稳定的三轴加速度和角速度数据
- 能够检测静止、行走、挥手等基本动作

#### Day 7: 按键和LED驱动
- **任务1.8**: 实现按键驱动 [components/drivers/button/button.c](components/drivers/button/button.c)
  - 配置GPIO输入引脚（SW1报警按键、SW2电源开关、SW3复位按键、BOOT按键IO0）
  - 实现按键扫描和消抖
  - 支持短按、长按、双击检测
  - SW1特殊功能：触发报警（LED1急速闪烁 + 扬声器警报）
  - SW2特殊功能：电源开关（长按3秒开机/关机）
  - 测试：按键触发LED闪烁

- **任务1.9**: 实现LED控制驱动 [components/drivers/alarm_io/alarm_io.c](components/drivers/alarm_io/alarm_io.c)
  - 配置GPIO输出引脚（LED1=IO6, LED2=IO2, LED3=IO1）
  - LED1：电源指示灯（设备通电时常亮）
  - LED2：充满指示灯（TP4056 STDBY引脚控制，电池充满时亮）
  - LED3：充电指示灯（TP4056 CHRG引脚控制，充电时亮）
  - 实现LED开关和PWM呼吸灯效果
  - 实现报警模式：LED1以10Hz频率急速闪烁
  - 测试：不同按键控制不同LED

**交付物**:
- 4个按键全部响应正常（SW1报警、SW2电源、SW3复位、BOOT系统）
- 3个LED可独立控制，状态指示正确
- SW1触发报警功能正常（LED1闪烁）

**第一周里程碑检查**:
- ✅ I2C总线正常工作，能识别3个设备
- ✅ OLED能显示温度和加速度信息
- ✅ DS18B20能实时读取体温
- ✅ MPU6050能输出运动数据
- ✅ 按键和LED交互正常

---

### 第二周（Day 8-14）：核心传感器和数据采集层

**目标**: 完成心率血氧监测、音频采集、电池管理等高级功能

#### Day 8-9: MAX30102心率血氧传感器驱动
- **任务2.1**: 实现MAX30102驱动 [components/drivers/max30102/max30102.c](components/drivers/max30102/max30102.c)
  - 初始化MAX30102（I2C地址0x57）
  - 配置LED电流和采样率（100Hz）
  - 读取红光和红外光原始数据
  - 实现FIFO缓冲区读取
  - 测试：在OLED上显示原始光电信号波形

- **任务2.2**: 实现心率检测算法
  - 实现峰值检测算法（寻找波峰）
  - 计算心率（BPM）：60秒/波峰间隔时间
  - 实现滑动窗口平均滤波
  - 测试：手指按压传感器，显示实时心率（60-100 BPM）

**交付物**:
- 能够检测到手指按压
- 显示稳定的心率数值（误差±5 BPM）
- （可选）实现血氧饱和度计算（R值法）

#### Day 10-11: I2S麦克风音频采集
- **任务2.3**: 实现I2S驱动 [components/drivers/i2s_mic/i2s_mic.c](components/drivers/i2s_mic/i2s_mic.c)
  - 配置I2S接口（SCK=IO15, SD=IO16, WS=IO17）
  - 设置采样率16kHz，位宽32bit
  - 实现DMA循环缓冲区
  - 实现音频数据读取函数：`i2s_read_audio_buffer()`
  - 测试：录制2秒音频并计算音量（RMS值）

- **任务2.4**: 实现音频可视化
  - 计算音频幅度（峰值或RMS）
  - 在OLED上显示音量条（实时柱状图）
  - 测试：对着麦克风说话，观察音量变化

**交付物**:
- 能够实时采集音频数据
- OLED显示环境音量大小
- 能够检测拍手、说话等声音事件

#### Day 12-13: 电池管理和电量监测
- **任务2.5**: 实现电池电压ADC采集 [components/drivers/battery/battery.c](components/drivers/battery/battery.c)
  - 配置ADC1通道（IO38连接电池分压电路）
  - 实现电压采集函数：`battery_read_voltage()`
  - 电压转换：ADC值 → 实际电压（通过R15/R16分压比计算）
  - 实现电量百分比计算（3.0V-4.2V映射到0-100%）
  - 测试：在OLED上显示电池电压和电量百分比

- **任务2.6**: 实现充电状态检测
  - 读取CHRG和STDBY引脚状态（IO2, IO1）
  - 判断充电状态：充电中/充满/未充电
  - 实现LED充电指示（充电时闪烁，充满常亮）
  - 测试：连接USB观察充电指示

**交付物**:
- 实时显示电池电量百分比
- 充电状态自动识别和LED指示

#### Day 14: UART调试接口和日志系统
- **任务2.7**: 配置UART串口 [components/drivers/usart/usart.c](components/drivers/usart/usart.c)
  - 配置UART0（RXD0, TXD0用于调试）
  - 实现串口发送和接收函数
  - 集成ESP-IDF日志系统（ESP_LOGI/ESP_LOGW/ESP_LOGE）
  - 测试：通过串口发送命令控制LED

**交付物**:
- 完善的日志输出系统
- 可通过串口调试各个模块

**第二周里程碑检查**:
- ✅ MAX30102能显示稳定心率
- ✅ 麦克风能实时采集音频
- ✅ 电池电量和充电状态显示正常
- ✅ 所有驱动层代码完成并测试通过

---

### 第三周（Day 15-21）：应用逻辑层和UI开发

**目标**: 实现数据处理算法、业务逻辑和用户界面

#### Day 15-16: 传感器数据采集管理模块
- **任务3.1**: 实现数据采集任务调度 [components/sensing/sensor_manager.c](components/sensing/sensor_manager.c)
  - 创建FreeRTOS任务管理各传感器
  - 实现任务优先级分配：
    - 高优先级：按键响应（实时性）
    - 中优先级：心率/体温采集（1Hz）
    - 低优先级：MPU6050数据处理（50Hz）
  - 实现数据结构体：
    ```c
    typedef struct {
        float temperature;      // 体温
        uint8_t heart_rate;     // 心率
        uint8_t spo2;           // 血氧
        int16_t accel_x, accel_y, accel_z;  // 加速度
        uint32_t step_count;    // 步数
        uint8_t battery_level;  // 电量
    } sensor_data_t;
    ```
  - 实现全局数据共享（使用互斥锁保护）
  - 创建sensing组件CMakeLists.txt
  - 测试：所有传感器数据能同步更新

- **任务3.2**: 实现数据平滑和异常值过滤 [components/sensing/data_filter.c](components/sensing/data_filter.c)
  - 对心率、体温数据进行滑动平均滤波
  - 过滤异常值（如心率>200或<40）
  - 测试：数据波动更稳定

**交付物**:
- 统一的传感器数据管理系统
- 数据更新频率稳定

#### Day 17-18: 运动算法实现（计步和姿态识别）
- **任务3.3**: 实现计步算法 [components/logic/step_counter.c](components/logic/step_counter.c)
  - 计算合成加速度：`sqrt(ax² + ay² + az²)`
  - 实现峰值检测和阈值判断（识别步态周期）
  - 添加防误判逻辑（时间窗口过滤）
  - 创建logic组件CMakeLists.txt
  - 测试：手臂摆动模拟行走，验证计步准确性（误差<5%）

- **任务3.4**: 实现简单姿态识别 [components/logic/gesture_recognition.c](components/logic/gesture_recognition.c)
  - 识别基本姿态：
    - 静止：加速度变化<阈值
    - 行走：周期性摆动
    - 挥手：Y轴快速变化
    - 抬手看时间：X轴倾角>45°
  - 测试：执行各种动作验证识别率

**交付物**:
- 准确的计步功能
- 能识别4种基本姿态

#### Day 19-20: 用户界面开发
- **任务3.5**: 实现OLED多页面UI [components/ui/ui_manager.c](components/ui/ui_manager.c)
  - 设计5个显示页面：
    1. **主界面**：时间、日期、电量
    2. **健康监测**：心率、血氧、体温
    3. **运动数据**：步数、距离、卡路里
    4. **环境监测**：噪音等级
    5. **设置菜单**：屏幕亮度、关于
  - 实现页面切换逻辑（按键控制）
  - 实现图标和动画效果（心跳图标、进度条）
  - 创建ui组件CMakeLists.txt
  - 测试：按键切换所有页面流畅无卡顿

- **任务3.6**: 实现交互逻辑 [components/ui/input_handler.c](components/ui/input_handler.c)
  - 按键映射：
    - BOOT按键：页面切换
    - SW1：报警触发（长按开始/停止报警）
    - SW2：电源开关（长按3秒开机/关机）
    - SW3：复位/返回主界面
  - 实现长按唤醒屏幕功能
  - 实现屏幕自动休眠（30秒无操作）
  - 测试：完整交互流程

**交付物**:
- 完整的5页UI系统
- 流畅的按键交互体验

#### Day 21: 蓝牙BLE与微信小程序通信
- **任务3.7**: 实现BLE广播和连接 [components/net/ble_service.c](components/net/ble_service.c)
  - 初始化BLE栈（使用NimBLE，兼容微信小程序）
  - 配置设备名称："ESP32-Wristband"
  - 实现微信小程序专用BLE GATT服务：
    - 服务UUID：自定义健康监测服务（兼容微信小程序蓝牙协议）
    - 特征值1：心率数据（可读+通知，实时推送）
    - 特征值2：血氧数据（可读+通知）
    - 特征值3：体温数据（可读）
    - 特征值4：步数数据（可读+通知）
    - 特征值5：报警状态（可读+可写+通知，用于紧急呼叫）
  - 实现BLE广播（微信可扫描）
  - 实现连接管理和断线重连
  - 创建net组件CMakeLists.txt
  - 测试：使用微信小程序蓝牙API连接并读取数据

- **任务3.8**: 实现数据上报协议 [components/net/wechat_protocol.c](components/net/wechat_protocol.c)
  - 定义数据打包格式（JSON或二进制协议）
  - 实现定时上报机制（心率每5秒，步数每分钟）
  - 实现报警消息推送（按下SW1或语音"呼叫家人"）
  - 实现小程序命令接收（远程查询、设置提醒等）
  - 测试：小程序能实时接收健康数据

**交付物**:
- 微信小程序能搜索并连接手环
- 能通过小程序实时查看心率、血氧、步数
- 报警功能能触发小程序通知

**第三周里程碑检查**:
- ✅ 所有传感器数据统一管理
- ✅ 计步功能准确可用
- ✅ 5页UI显示完整
- ✅ BLE与微信小程序通信正常
- ✅ 小程序能实时接收健康数据

---

### 第四周（Day 22-28）：功能集成、测试和优化

**目标**: 系统集成、完善功能、压力测试、准备答辩演示

#### Day 22-23: 语音识别、语音播报和报警系统
- **任务4.1**: 实现报警系统 [components/drivers/alarm_io/alarm_system.c](components/drivers/alarm_io/alarm_system.c)
  - 实现SW1按键触发报警功能
  - LED1急速闪烁控制（10Hz PWM）
  - 扬声器输出警报音（2kHz音调，间断性发声）
  - 实现报警自动停止（30秒超时或再次按SW1）
  - 实现BLE报警消息推送到微信小程序
  - 测试：按SW1触发报警，LED+扬声器工作，小程序收到通知

- **任务4.2**: 实现语音识别功能 [components/voice/voice_recognition.c](components/voice/voice_recognition.c)
  - 集成离线语音识别库（推荐：PocketSphinx移植版或自定义关键词检测）
  - 实现关键词唤醒："小手环"或"你好手环"
  - 实现命令识别：
    - "查询心率" → 获取当前心率并播报
    - "查询步数" → 获取当前步数并播报
    - "呼叫家人" → 触发紧急呼叫，通过小程序通知
  - 实现简化算法：端点检测 + MFCC特征 + 模板匹配
  - 备选方案：基于幅度和时长的简单关键词识别
  - 创建voice组件CMakeLists.txt
  - 测试：对麦克风说命令，手环正确识别并执行

- **任务4.3**: 实现语音播报功能 [components/voice/voice_tts.c](components/voice/voice_tts.c)
  - 方案1：预录音频文件（推荐，更清晰）
    - 录制常用播报语音："当前心率XX次每分钟"、"当前步数XX步"等
    - 使用ADPCM压缩存储在SPIFFS
  - 方案2：简单TTS（文字转语音）
    - 集成轻量级TTS库或数字语音合成
  - 实现I2S音频播放 [components/voice/audio_player.c](components/voice/audio_player.c)
    - 使用MAX98357A放大器输出
    - 支持播放预设音频和组合播报
  - 测试：语音命令后，扬声器播报查询结果

**交付物**:
- SW1按键报警功能完整可用（LED闪烁+扬声器警报+小程序通知）
- 语音命令识别准确率>80%（3个命令）
- 语音播报清晰可听
- "呼叫家人"功能触发小程序紧急通知

#### Day 24: 低功耗优化
- **任务4.4**: 实现省电模式 [components/common/power_manager.c](components/common/power_manager.c)
  - 屏幕休眠时进入Light Sleep模式
  - 配置唤醒源（按键中断、定时器）
  - 降低传感器采样频率（屏幕关闭时）
  - 测试：测量休眠和工作电流

- **任务4.5**: 电量管理策略
  - 低电量报警（<20%时LED1闪烁）
  - 极低电量自动关机（<5%）
  - 测试：模拟低电量场景

**交付物**:
- 待机电流<10mA
- 低电量保护机制正常

#### Day 25-26: 系统集成测试
- **任务4.6**: 完整功能流程测试
  - 开机自检流程：
    1. 显示启动画面
    2. 初始化所有传感器
    3. 检测传感器连接状态
    4. 播放开机提示音
    5. 自动连接上次配对的微信小程序
  - 日常使用流程测试：
    1. 按键唤醒屏幕
    2. 浏览各个页面
    3. 测量心率体温
    4. 查看计步数据
    5. 微信小程序连接并查看数据
    6. 语音命令："查询心率" → 语音播报
    7. 语音命令："查询步数" → 语音播报
    8. SW1触发报警测试（本地+小程序通知）
    9. 语音命令："呼叫家人" → 小程序紧急通知
  - 异常情况测试：
    1. 传感器断开处理
    2. 低电量处理
    3. 按键失灵保护
    4. BLE断线重连测试
    5. 语音识别失败重试机制

- **任务4.7**: 性能优化和微信小程序联调
  - 使用FreeRTOS任务监控工具分析CPU占用率
  - 优化I2C读取频率
  - 优化OLED刷新率（降低到30FPS）
  - 优化BLE数据传输频率（减少功耗）
  - 微信小程序UI优化和数据同步测试
  - 测试：系统运行流畅无死机，小程序实时性良好

**交付物**:
- 完整的功能演示视频（包含语音识别和小程序交互）
- 性能测试报告
- 微信小程序测试报告

#### Day 27: 代码整理和文档编写
- **任务4.8**: 代码规范化
  - 统一代码风格（使用clang-format）
  - 添加关键函数注释（Doxygen格式）
  - 编写README文档：
    - 项目简介和硬件清单
    - 编译烧录步骤
    - 功能使用说明
  - 编写开发文档：
    - 模块架构图
    - 各模块API说明
    - 引脚连接表

**交付物**:
- 完善的代码注释
- README.md和开发文档

#### Day 28: 答辩准备和演示优化
- **任务4.9**: 准备演示内容
  - 制作演示PPT：
    - 项目背景和意义
    - 系统架构和技术方案
    - 功能演示视频
    - 遇到的问题和解决方案
    - 项目总结和展望
  - 准备实物演示流程
  - 预演答辩Q&A

- **任务4.10**: 最终优化
  - 美化UI界面
  - 添加启动动画
  - 优化用户体验细节

**交付物**:
- 完整的答辩材料
- 可演示的成品设备

**第四周里程碑检查**:
- ✅ 所有功能模块集成完成
- ✅ 系统稳定运行无死机
- ✅ 文档齐全
- ✅ 答辩材料准备完毕

---

## 项目架构说明

本项目采用 **ESP-IDF 组件化架构**,将功能模块划分为独立的 components,提高代码的可维护性和可复用性。

### 目录结构

```
ESP32_S3_Wristband/
├── main/                           # 主程序
│   ├── main.c                      # 程序入口
│   └── CMakeLists.txt
├── components/                     # 组件目录（所有功能模块）
│   ├── common/                     # 公共模块
│   │   ├── app_config.h            # 全局配置头文件
│   │   └── CMakeLists.txt
│   ├── drivers/                    # 硬件驱动层
│   │   ├── i2c/                    # I2C总线驱动
│   │   ├── oled/                   # OLED显示驱动
│   │   ├── onewire/                # 1-Wire协议驱动
│   │   ├── ds18b20/                # DS18B20温度传感器
│   │   ├── max30102/               # MAX30102心率血氧传感器
│   │   ├── mpu6050/                # MPU6050运动传感器
│   │   ├── i2s_mic/                # I2S麦克风驱动
│   │   ├── button/                 # 按键驱动
│   │   ├── alarm_io/               # LED和蜂鸣器驱动
│   │   ├── battery/                # 电池管理驱动
│   │   ├── usart/                  # UART串口驱动
│   │   └── CMakeLists.txt          # 驱动层统一编译配置
│   ├── sensing/                    # 传感器数据采集层
│   ├── logic/                      # 应用逻辑层
│   ├── ui/                         # 用户界面层
│   ├── net/                        # 网络通信层
│   └── voice/                      # 语音处理层
├── CMakeLists.txt                  # 项目主配置
├── sdkconfig                       # ESP-IDF配置
├── PROGRESS.md                     # 开发进度跟踪
├── Development Plan.md             # 开发计划（本文档）
└── BUILD_GUIDE.md                  # 编译烧录指南
```

### 组件化优势

1. **模块独立性**: 每个驱动作为独立模块,可单独测试和复用
2. **依赖管理清晰**: 通过 CMakeLists.txt 的 `REQUIRES` 明确组件依赖关系
3. **编译优化**: ESP-IDF 自动处理组件依赖,只编译需要的模块
4. **易于维护**: 新增功能只需在对应 components 子目录下添加代码

---

## 关键文件清单

### 驱动层 (components/drivers/)
| 文件路径 | 功能描述 | 优先级 |
|---------|---------|-------|
| [components/drivers/i2c/i2c_master.c](components/drivers/i2c/i2c_master.c) | I2C总线驱动 | P0 |
| [components/drivers/i2c/i2c_master.h](components/drivers/i2c/i2c_master.h) | I2C驱动头文件 | P0 |
| [components/drivers/oled/oled.c](components/drivers/oled/oled.c) | OLED显示驱动 | P0 |
| [components/drivers/oled/oled.h](components/drivers/oled/oled.h) | OLED驱动头文件 | P0 |
| [components/drivers/onewire/onewire.c](components/drivers/onewire/onewire.c) | 1-Wire协议驱动 | P0 |
| [components/drivers/ds18b20/ds18b20.c](components/drivers/ds18b20/ds18b20.c) | DS18B20温度传感器 | P0 |
| [components/drivers/max30102/max30102.c](components/drivers/max30102/max30102.c) | MAX30102心率血氧传感器 | P0 |
| [components/drivers/mpu6050/mpu6050.c](components/drivers/mpu6050/mpu6050.c) | MPU6050运动传感器 | P0 |
| [components/drivers/i2s_mic/i2s_mic.c](components/drivers/i2s_mic/i2s_mic.c) | I2S麦克风驱动 | P1 |
| [components/drivers/button/button.c](components/drivers/button/button.c) | 按键输入驱动 | P0 |
| [components/drivers/alarm_io/alarm_io.c](components/drivers/alarm_io/alarm_io.c) | LED和蜂鸣器驱动 | P0 |
| [components/drivers/battery/battery.c](components/drivers/battery/battery.c) | 电池管理驱动 | P1 |
| [components/drivers/usart/usart.c](components/drivers/usart/usart.c) | UART串口驱动 | P2 |
| [components/drivers/CMakeLists.txt](components/drivers/CMakeLists.txt) | 驱动层统一编译配置 | P0 |

### 传感器数据层 (components/sensing/)
| 文件路径 | 功能描述 | 优先级 |
|---------|---------|-------|
| [components/sensing/sensor_manager.c](components/sensing/sensor_manager.c) | 传感器数据采集管理 | P0 |
| [components/sensing/data_filter.c](components/sensing/data_filter.c) | 数据滤波和平滑 | P1 |
| [components/sensing/CMakeLists.txt](components/sensing/CMakeLists.txt) | 传感器层编译配置 | P0 |

### 应用逻辑层 (components/logic/)
| 文件路径 | 功能描述 | 优先级 |
|---------|---------|-------|
| [components/logic/step_counter.c](components/logic/step_counter.c) | 计步算法 | P0 |
| [components/logic/gesture_recognition.c](components/logic/gesture_recognition.c) | 姿态识别 | P1 |
| [components/logic/heart_rate_algo.c](components/logic/heart_rate_algo.c) | 心率算法 | P0 |
| [components/logic/CMakeLists.txt](components/logic/CMakeLists.txt) | 逻辑层编译配置 | P0 |

### 用户界面层 (components/ui/)
| 文件路径 | 功能描述 | 优先级 |
|---------|---------|-------|
| [components/ui/ui_manager.c](components/ui/ui_manager.c) | UI页面管理 | P0 |
| [components/ui/input_handler.c](components/ui/input_handler.c) | 按键输入处理 | P0 |
| [components/ui/display_pages.c](components/ui/display_pages.c) | 各个显示页面实现 | P0 |
| [components/ui/CMakeLists.txt](components/ui/CMakeLists.txt) | UI层编译配置 | P0 |

### 网络通信层 (components/net/)
| 文件路径 | 功能描述 | 优先级 |
|---------|---------|-------|
| [components/net/ble_service.c](components/net/ble_service.c) | 蓝牙BLE通信服务（微信小程序） | P0 |
| [components/net/ble_gatt_server.c](components/net/ble_gatt_server.c) | BLE GATT服务器 | P0 |
| [components/net/wechat_protocol.c](components/net/wechat_protocol.c) | 微信小程序数据协议 | P0 |
| [components/net/emergency_call.c](components/net/emergency_call.c) | 紧急呼叫通知模块 | P0 |
| [components/net/CMakeLists.txt](components/net/CMakeLists.txt) | 网络层编译配置 | P0 |

### 语音处理层 (components/voice/)
| 文件路径 | 功能描述 | 优先级 |
|---------|---------|-------|
| [components/voice/voice_recognition.c](components/voice/voice_recognition.c) | 语音识别（关键词检测） | P0 |
| [components/voice/voice_tts.c](components/voice/voice_tts.c) | 语音播报（TTS或预录音频） | P0 |
| [components/voice/audio_player.c](components/voice/audio_player.c) | 音频播放 | P0 |
| [components/voice/audio_recorder.c](components/voice/audio_recorder.c) | 音频录制（调试用） | P2 |
| [components/voice/CMakeLists.txt](components/voice/CMakeLists.txt) | 语音层编译配置 | P0 |

### 公共模块层 (components/common/)
| 文件路径 | 功能描述 | 优先级 |
|---------|---------|-------|
| [components/common/app_config.h](components/common/app_config.h) | 全局配置头文件 | P0 |
| [components/common/power_manager.c](components/common/power_manager.c) | 电源管理 | P1 |
| [components/common/nvs_storage.c](components/common/nvs_storage.c) | 数据持久化存储 | P2 |
| [components/common/time_manager.c](components/common/time_manager.c) | 时间管理（RTC） | P1 |
| [components/common/CMakeLists.txt](components/common/CMakeLists.txt) | 公共模块编译配置 | P0 |

### 主程序 (main/)
| 文件路径 | 功能描述 | 优先级 |
|---------|---------|-------|
| [main/main.c](main/main.c) | 主程序入口 | P0 |
| [main/CMakeLists.txt](main/CMakeLists.txt) | 主程序编译配置 | P0 |

**优先级说明**:
- P0: 核心功能，必须实现
- P1: 重要功能，建议实现
- P2: 扩展功能，时间允许时实现

---

## 技术难点和解决方案

### 难点1: I2C总线共享冲突
**问题**: OLED、MAX30102、MPU6050共享同一I2C总线，可能产生冲突。

**解决方案**:
1. 为每个设备分配不同的I2C地址：
   - OLED: 0x3C
   - MAX30102: 0x57
   - MPU6050: 0x68（AD0接GND）
2. 使用互斥锁保护I2C总线访问：
   ```c
   SemaphoreHandle_t i2c_mutex;
   i2c_mutex = xSemaphoreCreateMutex();

   // 访问I2C前获取锁
   xSemaphoreTake(i2c_mutex, portMAX_DELAY);
   i2c_master_write(...);
   xSemaphoreGive(i2c_mutex);
   ```
3. 合理分配任务优先级，避免高频率轮询

### 难点2: MAX30102心率算法准确性
**问题**: 原始光电信号噪声大，直接计算心率误差高。

**解决方案**:
1. **硬件层面**:
   - 确保传感器紧贴皮肤
   - 避免强光干扰
2. **软件层面**:
   - 带通滤波器（0.5-4Hz，过滤呼吸和高频噪声）
   - 峰值检测算法（寻找波峰间隔）
   - 滑动窗口平均（取最近5个心跳的平均值）
   - 异常值剔除（心率范围40-200 BPM）
3. **推荐开源库**: MAX30102 Arduino库（可移植到ESP-IDF）

### 难点3: 计步算法防误判
**问题**: 手臂摆动、乘车颠簸等会误触发计步。

**解决方案**:
1. 设置合理阈值（加速度变化量0.5-2g）
2. 时间窗口过滤（两步间隔0.3-2秒）
3. 频率分析（步频1-3Hz范围内）
4. 状态机设计（需连续检测到3个有效步态周期）

### 难点4: 蓝牙和Wi-Fi共存（如果需要）
**问题**: ESP32-S3的蓝牙和Wi-Fi共享射频，同时使用会冲突。

**解决方案**:
1. **推荐方案**: 仅使用蓝牙BLE（低功耗，适合手环）
2. 如必须同时使用：
   - 启用ESP-IDF的共存机制（coexist）
   - 配置`menuconfig` -> Component config -> Wi-Fi -> WiFi/Bluetooth coexistence
   - 降低Wi-Fi传输速率，优先保证BLE稳定

### 难点5: 低功耗设计
**问题**: 多传感器同时工作导致功耗高，电池续航短。

**解决方案**:
1. 动态电源管理：
   - 屏幕关闭时进入Light Sleep
   - 降低传感器采样率（心率从100Hz降至10Hz）
   - 关闭不用的外设时钟
2. 优化代码：
   - 减少不必要的日志输出
   - 使用DMA传输减少CPU唤醒
3. 硬件优化：
   - 选择低功耗传感器工作模式
   - OLED显示尽量使用暗色主题

---

## 测试验收标准

### 功能测试（必须全部通过）
| 测试项 | 验收标准 | 测试方法 |
|-------|---------|---------|
| OLED显示 | 显示清晰，刷新率>20FPS | 目视检查，无闪烁 |
| 体温测量 | 误差<±0.5℃ | 与医用温度计对比 |
| 心率测量 | 误差<±5 BPM | 与智能手表对比 |
| 计步功能 | 误差<10% | 实际行走100步对比 |
| 按键响应 | 延迟<100ms | 按键触发LED，秒表计时 |
| 电量显示 | 误差<5% | 万用表测量电压对比 |
| 充电功能 | 正常充电，LED指示正确 | USB充电测试 |
| 蓝牙连接 | 10米内稳定连接 | 手机APP连接测试 |
| 音频录放 | 能录制和播放，音质清晰 | 录制语音播放 |

### 性能测试
| 测试项 | 验收标准 |
|-------|---------|
| 启动时间 | <3秒进入主界面 |
| 传感器响应 | 心率显示延迟<2秒 |
| 页面切换 | <100ms无卡顿 |
| 内存占用 | <80% DRAM使用率 |
| CPU占用率 | 空闲时<30% |
| 待机电流 | <10mA（屏幕关闭） |
| 工作电流 | <100mA（全功能运行） |
| 续航时间 | 典型使用>12小时（400mAh电池） |

### 稳定性测试
| 测试项 | 验收标准 |
|-------|---------|
| 长时间运行 | 连续运行24小时无死机 |
| 温度适应性 | -10℃至50℃正常工作 |
| 按键寿命 | 连续按压1000次无失灵 |
| 传感器热插拔 | 断开后重新连接能自动恢复 |

---

## 风险评估和应对策略

### 高风险项（需要重点关注）

#### 风险1: MAX30102心率测量不稳定
**影响**: 核心功能无法演示。

**应对策略**:
1. 预留充足时间（Day 8-9）进行调试
2. 如算法复杂，使用开源库（如Sparkfun MAX30102库）
3. 备选方案：仅实现光电信号波形显示，简化心率算法为波峰计数

#### 风险2: I2S音频采集和播放调试困难
**影响**: 语音功能无法实现。

**应对策略**:
1. 该功能优先级较低，可放在第四周
2. 参考ESP-IDF官方I2S例程
3. 备选方案：简化为环境噪音监测（仅显示音量大小）

#### 风险3: 时间不足导致功能删减
**影响**: 项目完整度降低。

**应对策略**:
1. 严格按照周计划执行，避免拖延
2. 功能优先级：P0功能必须完成，P1功能尽量完成，P2功能可选
3. 每周进行里程碑检查，及时调整计划

### 中风险项

#### 风险4: BLE通信调试复杂
**应对策略**:
- 使用ESP-IDF的BLE示例代码作为模板
- 先实现基础的数据读取，通知功能可选
- 使用nRF Connect等工具辅助调试

#### 风险5: 低功耗优化效果不佳
**应对策略**:
- 低功耗优化放在最后一周
- 如时间不够，只实现屏幕休眠即可
- 答辩时说明可优化空间

---

## 学习资源推荐

### ESP-IDF官方文档
- **ESP-IDF编程指南**: https://docs.espressif.com/projects/esp-idf/zh_CN/v5.2.3/esp32s3/
- **API参考**: https://docs.espressif.com/projects/esp-idf/zh_CN/v5.2.3/esp32s3/api-reference/
- **示例代码**: `$IDF_PATH/examples/`

### 传感器驱动参考
- **MAX30102**: Sparkfun MAX30102库（GitHub）
- **MPU6050**: TDK官方数据手册和应用笔记
- **DS18B20**: Maxim官方1-Wire协议文档
- **SSD1306 OLED**: U8g2图形库（支持ESP-IDF）

### FreeRTOS学习
- 《Mastering the FreeRTOS Real Time Kernel》（有中文版）
- ESP-IDF FreeRTOS任务管理文档

### 算法参考
- 心率检测算法论文：Peak Detection Algorithm（搜索相关论文）
- 计步算法：Android Pedometer源码

### 调试工具
- **串口调试**: PuTTY, SecureCRT
- **BLE调试**: nRF Connect（iOS/Android）
- **逻辑分析仪**: 调试I2C/I2S时序（可选）

---

## 开发环境清单

### 必备软件
- ESP-IDF 5.2.3（已安装）
- Visual Studio Code + ESP-IDF插件
- Python 3.11（已配置）
- Git（版本管理）

### 推荐工具
- Serial Flasher（烧录工具）
- ESP-IDF Monitor（串口监控）
- nRF Connect（BLE调试）
- Proteus（电路仿真，可选）

### 硬件工具
- ESP32-S3开发板
- USB数据线（Type-C）
- 400mAh锂电池
- 万用表（电压测试）
- 面包板和杜邦线（调试用）

---

## 答辩准备建议

### 演示视频内容（建议录制5-7分钟）
1. **开机自检** (20秒)
   - 显示启动画面和初始化日志
   - 自动连接微信小程序

2. **健康监测演示** (60秒)
   - 实时显示心率、血氧、体温
   - 展示数值变化
   - 微信小程序同步显示数据

3. **运动追踪演示** (45秒)
   - 模拟行走，展示计步功能
   - 展示姿态识别（挥手、抬腕）
   - 小程序显示步数更新

4. **语音交互演示** (90秒)
   - 语音命令："查询心率" → 扬声器播报"当前心率XX次每分钟"
   - 语音命令："查询步数" → 扬声器播报"当前步数XX步"
   - 演示语音识别准确性

5. **紧急报警演示** (60秒)
   - 按SW1触发报警：LED闪烁 + 警报音
   - 微信小程序收到报警通知
   - 语音命令："呼叫家人" → 小程序收到紧急呼叫

6. **交互功能演示** (30秒)
   - 按键切换页面
   - 音量柱状图显示

7. **电源管理演示** (15秒)
   - 电量显示
   - 充电指示

### 答辩可能问到的问题
1. **硬件相关**:
   - Q: 为什么选择ESP32-S3芯片？
   - A: 双核处理器，支持BLE 5.0，丰富的GPIO和通信接口，适合多传感器集成。

2. **软件架构**:
   - Q: 你的软件架构是如何设计的？
   - A: 分层设计：驱动层、传感器数据层、应用逻辑层、UI层、网络层，模块化清晰，易于维护。

3. **技术难点**:
   - Q: I2C总线多设备如何防冲突？
   - A: 使用FreeRTOS互斥锁保护临界区，不同设备分配不同地址。

4. **算法原理**:
   - Q: 心率是如何计算的？
   - A: MAX30102输出光电信号，经过带通滤波后进行峰值检测，通过波峰间隔计算心率。

5. **功耗优化**:
   - Q: 如何降低功耗延长续航？
   - A: 屏幕休眠进入Light Sleep，降低传感器采样率，关闭不用的外设。

### PPT结构建议
1. **项目背景** (1页)
2. **系统架构图** (1页)
3. **硬件设计** (1页：原理图关键部分）
4. **软件架构** (2页：模块图+流程图）
5. **功能演示** (3页：配截图）
6. **技术难点** (2页：问题+解决方案）
7. **测试结果** (1页：测试数据表）
8. **项目总结** (1页）
9. **致谢** (1页）

---

## 项目交付清单

### 代码交付
- [ ] 完整源代码（所有.c/.h文件）
- [ ] CMakeLists.txt编译配置
- [ ] sdkconfig项目配置
- [ ] .bin固件文件（可直接烧录）

### 文档交付
- [ ] README.md（项目说明）
- [ ] 硬件连接表（引脚定义）
- [ ] 软件架构文档
- [ ] API接口文档
- [ ] 测试报告

### 演示材料
- [ ] 演示视频（3-5分钟）
- [ ] 答辩PPT
- [ ] 实物展示（手环成品）

### 可选材料
- [ ] 原理图和PCB文件
- [ ] 3D外壳设计文件
- [ ] 开发日志/博客

---

## 每日开发建议

### 开发习惯
1. **每天编码前**:
   - 查看今日任务清单
   - 回顾昨天代码
   - 更新TODO列表

2. **编码过程中**:
   - 每完成一个小功能就测试
   - 及时提交代码到Git（每天至少1次commit）
   - 遇到问题立即记录和寻求帮助

3. **每天结束时**:
   - 整理当天完成的功能
   - 记录遇到的问题和解决方案
   - 规划明天任务

### 调试技巧
1. 使用ESP_LOGI打印关键变量
2. 使用JTAG调试器单步调试（可选）
3. 查看FreeRTOS任务堆栈使用情况（防溢出）
4. 使用`idf.py monitor`查看崩溃日志

### 寻求帮助的渠道
1. ESP32官方论坛
2. GitHub Issues（开源库）
3. CSDN/博客园（中文教程）
4. 指导老师/同学

---

## 总结

这个一个月的开发计划是针对ESP-IDF初学者、注重功能完整性的毕业设计而制定的。计划的核心特点：

✅ **循序渐进**: 从简单驱动到复杂算法，逐步提升难度
✅ **模块化开发**: 每个功能独立测试，降低集成风险
✅ **组件化架构**: 采用ESP-IDF标准组件结构，提升代码可维护性
✅ **预留缓冲时间**: 第四周专门用于集成测试和答辩准备
✅ **优先级分明**: P0功能优先保证，P1/P2功能灵活调整
✅ **实践导向**: 每个任务都有明确的测试标准和交付物

**关键成功因素**:
1. 严格按照时间表执行，不拖延
2. 遇到难题及时调整方案，不死磕
3. 每周进行里程碑检查，确保进度
4. 保持代码整洁，方便后期维护
5. 充分利用ESP-IDF组件系统，模块间依赖清晰

**组件化架构优势**:
1. **清晰的模块边界**: 驱动层、传感器层、逻辑层、UI层、网络层、语音层各司其职
2. **便于团队协作**: 不同模块可并行开发，互不干扰
3. **易于测试**: 每个组件可独立编译和单元测试
4. **高度可复用**: 驱动组件可用于其他ESP32-S3项目
5. **依赖管理自动化**: ESP-IDF构建系统自动处理组件依赖关系

祝您开发顺利，答辩成功！🎓
