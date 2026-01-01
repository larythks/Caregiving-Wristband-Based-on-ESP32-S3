# ESP32-S3智能手环 - 快速开始指南

## 项目状态

**当前进度**: Day 3 - OLED显示驱动开发完成 ✅

已完成功能:
- ✅ 项目基础框架搭建
- ✅ 全局配置文件 (app_config.h)
- ✅ I2C主机驱动 (完整实现)
  - 初始化和去初始化
  - 读写单个/多个字节
  - I2C总线扫描功能
  - 线程安全(互斥锁保护)
- ✅ 主程序集成I2C设备扫描
- ✅ **OLED显示驱动 (SH1106) - 新增!**
  - 完整的SH1106驱动实现
  - 6x8 ASCII字体库
  - 文字、数字、图形显示
  - 显示缓冲区管理
  - 实时计数器显示

## 编译和烧录步骤

### 1. 打开ESP-IDF命令行

在Windows上:
1. 打开"ESP-IDF Command Prompt (cmd.exe)" 或 "ESP-IDF PowerShell"
2. 或者在普通命令行中运行:
   ```bash
   D:\studying\Espressif\frameworks\esp-idf-v5.2.3\export.bat
   ```

### 2. 进入项目目录

```bash
cd f:\graduation_project\project\ESP32_S3_Wristband
```

### 3. 配置项目 (首次编译)

```bash
idf.py menuconfig
```

关键配置项:
- **Component config → ESP32S3-Specific → CPU frequency**: 240 MHz (推荐)
- **Component config → FreeRTOS → Tick rate (Hz)**: 1000
- **Serial flasher config → Flash size**: 根据您的ESP32-S3模块选择

### 4. 编译项目

```bash
idf.py build
```

预期输出:
```
...
Project build complete. To flash, run:
  idf.py -p COM4 flash
...
```

### 5. 烧录到ESP32-S3

首先确认串口号 (在设备管理器中查看):
```bash
idf.py -p COM4 flash
```

将 `COM4` 替换为您的实际串口号。

### 6. 查看串口输出

```bash
idf.py -p COM4 monitor
```

或者一步完成编译、烧录和监控:
```bash
idf.py -p COM4 flash monitor
```

退出监控: 按 `Ctrl + ]`

## 预期串口输出

如果一切正常，您应该看到类似以下输出:

```
========================================
ESP32-S3 Smart Wristband Starting...
========================================
I (xxx) MAIN: NVS initialized successfully
I (xxx) MAIN: ESP-IDF Version: v5.2.3
I (xxx) MAIN: Free heap: 390000 bytes
========================================
I (xxx) MAIN: Initializing Hardware Drivers (Day 1-2)
========================================
I (xxx) I2C: Initializing I2C master...
I (xxx) I2C:   SDA: GPIO8, SCL: GPIO9
I (xxx) I2C:   Frequency: 400000 Hz
I (xxx) I2C: I2C master initialized successfully
I (xxx) MAIN: Scanning I2C bus for devices...
I (xxx) I2C: Scanning I2C bus...
I (xxx) I2C:      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- 57 -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
I (xxx) I2C: Found 3 device(s) on I2C bus
I (xxx) MAIN: I2C device detection:
I (xxx) MAIN:   OLED (0x3C):     FOUND
I (xxx) MAIN:   MAX30102 (0x57): FOUND
I (xxx) MAIN:   MPU6050 (0x68):  FOUND
========================================
I (xxx) MAIN: Hardware initialization complete!
========================================
I (xxx) MAIN: Initializing OLED Display (Day 3)
========================================
I (xxx) OLED: Initializing SH1106 OLED display...
I (xxx) OLED: OLED initialized successfully
I (xxx) MAIN: OLED display initialized successfully
I (xxx) MAIN: Welcome message displayed on OLED
========================================
I (xxx) MAIN: Entering main loop...
I (xxx) MAIN: Loop #1 - Free heap: 380000 bytes
I (xxx) MAIN: Loop #2 - Free heap: 380000 bytes
...
```

### OLED屏幕显示内容

如果OLED正常工作，您应该在屏幕上看到:

```
ESP32-S3           ← 第0行
Wristband          ← 第1行
                   ← 第2行(空)
Hardware OK!       ← 第3行
                   ← 第4行(空)
I2C Devices:       ← 第5行
Found: 3           ← 第6行
Count: 1           ← 第7行 (每秒更新)
```

计数器会每秒递增: `Count: 1`, `Count: 2`, `Count: 3`...

### 如果没有检测到I2C设备

```
I (xxx) I2C: Found 0 device(s) on I2C bus
W (xxx) MAIN: No I2C devices found! Check hardware connections.
```

**排查步骤**:
1. 检查硬件连接:
   - SDA (GPIO8) 是否正确连接
   - SCL (GPIO9) 是否正确连接
   - 3.3V 供电是否正常
   - GND 是否接地
2. 检查传感器模块是否上电
3. 用万用表测量I2C总线上是否有上拉电阻 (通常 4.7K)
4. 尝试降低I2C频率 (在 app_config.h 中修改 `I2C_MASTER_FREQ_HZ` 为 100000)

## 常见问题

### 1. 编译错误: `command not found`
**解决**: 确保已运行 ESP-IDF 环境初始化脚本 (export.bat)

### 2. 烧录失败: `Failed to connect`
**解决**:
- 检查USB线缆是否连接
- 按住 BOOT 按钮，然后按 RESET 按钮进入下载模式
- 检查设备管理器中串口是否正常

### 3. 编译错误: `i2c_master.h: No such file or directory`
**解决**: 检查 CMakeLists.txt 中是否正确添加了 EXTRA_COMPONENT_DIRS

## 下一步 (Day 4)

完成 Day 3 验证后，下一步将实现:
- 1-Wire 协议驱动
- DS18B20 温度传感器驱动
- 在 OLED 上显示实时体温数据

---

**文档更新日期**: 2026-01-01
**项目进度**: Week 1, Day 3 完成 (OLED显示驱动)
