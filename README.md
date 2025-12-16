# 主板通信与协议说明

本文档说明本项目固件中涉及的三类通信协议及其在代码中的实现位置，方便上位机或其他设备进行对接与调试。主要包含三部分：

1. 上位机交互协议（USART0，简易 6 字节帧）
2. 另一串口接收 ADC 数据包协议（USART1，来自下位 MCU 的采样数据）
3. 周期性发送给上位机的两个扩展数据包协议（AA AA AA 头，ADC 周期上报 + 电池信息周期上报）

---

## 1. 上位机交互协议（USART0）

### 1.1 物理连接与初始化

- 串口：`USART0`
- 引脚：`PA9(TX), PA10(RX)`
- 波特率：`115200`
- 数据位：`8 bit`
- 校验：`无`
- 停止位：`1`
- 接口方向：与「主芯片 / 上位机」通信

初始化代码：

- `User/tool.c` -> `void com_init0(void)`
- 在 `main()` 中被调用：

```c
int main(void)
{
    nvic_irq_enable(USART0_IRQn, 0, 0);
    ...
    com_init0();
    ...
}
```

接收中断处理：

- `User/gd32f1x0_it.c` -> `void USART0_IRQHandler(void)`

发送函数：

- `User/tool.c` -> `void usart_transmit(uint32_t usart_periph, uint8_t *data, uint8_t length)`
- `printf` 重定向到 `USART0`：
  - `User/main.c` -> `int fputc(int ch, FILE *f)`

### 1.2 帧格式（6 字节命令/应答）

上位机与本板之间采用固定 6 字节命令帧：

| 字节索引 | 含义       | 示例/取值                      |
|----------|------------|---------------------------------|
| [0]      | 帧头 1     | 固定 `0xAA`                     |
| [1]      | 帧头 2     | 固定 `0x55`                     |
| [2]      | 命令字     | 见下文命令列表                  |
| [3]      | 数据高字节 | 含义依命令不同                  |
| [4]      | 数据低字节 | 含义依命令不同                  |
| [5]      | 帧尾       | 固定 `0xFF`                     |

接收缓冲与标志：

- 接收缓冲：`usart0_receiver_buffer[16]`
- 有效帧缓存：`host_receiver_buffer[6]`
- 帧长度常量：`HOST_RECEIVE_COUNT = 6`
- 处理标志：`extern uint8_t need_process_command;`

`USART0_IRQHandler` 中断接收逻辑（要点）：

1. 逐字节接收到 `usart0_receiver_buffer[usart0_rxcount++]`。
2. 若第一个字节不是 `0xAA`，清空缓冲重新对齐。
3. 若第二个字节不是 `0x55`，且不是 `0xAA 0xAA` 的同步情况，则清空缓冲。
4. 收满 6 字节后：
   - 若 `usart0_receiver_buffer[5] == 0xFF`，则复制到 `host_receiver_buffer`，并置位 `need_process_command = 1`。
   - 否则丢弃。

`process_command()` 在主循环中被调用，完成命令解析和应答。

### 1.3 命令列表与应答格式

命令定义：`User/tool.c`

```c
#define HOST_RECEIVE_COUNT                 6
#define HOST_COMMAND_HEART_BEAT            0x00
#define HOST_COMMAND_GET_POWER_STATUS      0x01
#define HOST_COMMAND_SET_POWER_STATUS      0x02
#define HOST_COMMAND_GET_ADC_VALUE         0x03
#define HOST_COMMAND_GET_BAT_CAPACITY      0x04
#define HOST_COMMAND_GET_BAT_TEMPEARATURE  0x05
#define HOST_COMMAND_GET_BAT_VOLTAGE       0x06
#define HOST_COMMAND_BOOTUP_SUCCESS        0x10
```

`process_command()` 中对完整帧的检查：

```c
if ((host_receiver_buffer[0] == 0xaa) &&
    (host_receiver_buffer[1] == 0x55) &&
    (host_receiver_buffer[5] == 0xff)) {
    // 根据 host_receiver_buffer[2] 进入 switch
}
```

错误回复缓冲区：

```c
uint8_t host_reply_error_status_buffer[6] = {0x55, 0xaa, 0xff, 0xff, 0xff, 0xff};
```

下面是各命令说明：

#### 1.3.1 心跳命令（0x00，HOST_COMMAND_HEART_BEAT）

- 上位机请求：

  ```text
  AA 55 00 00 00 FF
  ```

- 固件应答（`host_reply_heart_beat_buffer`）：

  ```text
  55 AA 00 00 00 FF
  ```

用于链路保活与简单连通性测试。

#### 1.3.2 查询电源输出状态（0x01，HOST_COMMAND_GET_POWER_STATUS）

- 上位机请求：

  ```text
  AA 55 01 <通道索引> 00 FF
  ```

  通道索引 `[3]`（0~4）对应 GPIOA 的几路电源：

  | 索引 | 含义       | 引脚         |
  |------|------------|--------------|
  | 0    | 3V3 ULP    | GPIOA PIN 0  |
  | 1    | 5V0 SYS    | GPIOA PIN 1  |
  | 2    | 5V0 CORE   | GPIOA PIN 4  |
  | 3    | 3V3 SYS    | GPIOA PIN 5  |
  | 4    | 12V SYS    | GPIOA PIN 6  |

- 固件成功应答：

  ```text
  55 AA 01 <通道索引> <当前状态0/1> FF
  ```

  其中 `[4] = gpio_output_bit_get(GPIOA, pin_power_map[idx])`。

- 若索引无效（>=5），回复错误帧：

  ```text
  55 AA FF FF FF FF
  ```

#### 1.3.3 设置电源输出状态（0x02，HOST_COMMAND_SET_POWER_STATUS）

- 上位机请求：

  ```text
  AA 55 02 <通道索引> <值> FF
  ```

  - `[3]`：通道索引 0~4，含义同上。
  - `[4]`：目标状态：
    - `0` -> 关闭（RESET）
    - `非0` -> 打开（SET）

- 固件操作：

  ```c
  gpio_bit_write(GPIOA,
                 pin_power_map[host_receiver_buffer[3]],
                 (host_receiver_buffer[4] > 0) ? SET : RESET);
  ```

- 成功应答：

  ```text
  55 AA 02 <通道索引> <当前状态0/1> FF
  ```

- 索引无效时返回错误帧 `55 AA FF FF FF FF`。

#### 1.3.4 读取 ADC 数组（0x03，HOST_COMMAND_GET_ADC_VALUE）

- 上位机请求：

  ```text
  AA 55 03 00 00 FF
  ```

- 固件应答缓冲：`host_reply_get_adc_value_buffer[24]`：

  ```c
  uint8_t host_reply_get_adc_value_buffer[24] = {
      0x55, 0xaa, HOST_COMMAND_GET_ADC_VALUE,
      0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00,
      0xff
  };
  ```

发送前会执行：

```c
memcpy(&host_reply_get_adc_value_buffer[3], adc_value, 20);
```

**实际应答格式：**

```text
[0]  55
[1]  AA
[2]  03   // 命令码
[3]~[22]  20 字节 ADC 通道数据 adc_value[0..19]
[23] FF
```

这 20 字节来自于另一个 MCU 通过 USART1 发来的采样数据（详见第 2 节）。

#### 1.3.5 读取电池 SOC（0x04，HOST_COMMAND_GET_BAT_CAPACITY）

- 上位机请求：

  ```text
  AA 55 04 00 00 FF
  ```

- 固件内部调用：

  ```c
  battery_temp_data = BQ40Z50_Read_SOC();  // 读取 BQ40Z50 0x0D 寄存器
  ```

- 应答格式：

  ```text
  55 AA 04 <SOC高字节> <SOC低字节> FF
  ```

- SOC 原始值含义：BQ40Z50 的状态寄存器 `0x0D`，通常代表电量百分比（以电量百分数或其它单位编码，上位机应参考 BQ40Z50 手册做最终解释）。

#### 1.3.6 读取电池温度（0x05，HOST_COMMAND_GET_BAT_TEMPEARATURE）

- 上位机请求：

  ```text
  AA 55 05 00 00 FF
  ```

- 固件内部：

  ```c
  battery_temp_data = BQ40Z50_Read_Temp(); // 读取寄存器 0x08
  ```

- 应答格式：

  ```text
  55 AA 05 <Temp高字节> <Temp低字节> FF
  ```

- 注意：固件仅透传 BQ40Z50 `0x08` 寄存器原始 16bit 数值，不做物理单位换算，上位机需参照 BQ40Z50 手册自行换算为 ℃ 等单位。

#### 1.3.7 读取电池电压（0x06，HOST_COMMAND_GET_BAT_VOLTAGE）

- 上位机请求：

  ```text
  AA 55 06 00 00 FF
  ```

- 固件内部：

  ```c
  battery_temp_data = BQ40Z50_Read_Vol(); // 读取寄存器 0x09
  ```

- 应答格式：

  ```text
  55 AA 06 <Volt高字节> <Volt低字节> FF
  ```

- 同样，电压值为原始寄存器值，上位机根据 BQ40Z50 协议折算为 mV 等。

#### 1.3.8 Bootup 成功通知（0x10，HOST_COMMAND_BOOTUP_SUCCESS）

- 上位机请求：

  ```text
  AA 55 10 00 00 FF
  ```

- 固件：

  ```c
  host_bootup_ok = 1;
  ```

  并回复：

  ```text
  55 AA 10 00 00 FF
  ```

- `host_bootup_ok` 作为上电后「主控已启动完成」的信号，用于允许某些周期上报逻辑（比如旧的 `periodic_report()` 等）。

---

## 2. 下位 MCU ADC 数据包协议（USART1）

这一部分描述「另一个 MCU -> 本板」的 ADC 数据帧格式，本板通过该协议获取 20 路 ADC 采样。

### 2.1 物理连接与初始化

- 串口：`USART1`
- 引脚：`PA15(RX)`（本项目中仅用 RX）
- 波特率：`115200`
- 数据位：`8 bit`
- 校验：`无`
- 停止位：`1`

初始化函数：

- `User/tool.c` -> `void com_init1(void)`

接收中断服务：

- `User/gd32f1x0_it.c` -> `void USART1_IRQHandler(void)`

### 2.2 帧长度与基本结构

宏：

```c
#define MCU_RECEIVE_COUNT 24
```

固件认为来自下位 MCU 的每一帧为 **24 字节**。当前版本的关键字段：

- 起始：`0xAA 0xAA`
- 中间：20 字节 ADC 数据
- 结束：最后一个字节为 `0xFF`（中间的倒数第二字节预留，可扩展作校验等）

`USART1_IRQHandler` 中逻辑概述：

1. 逐字节接收填入 `usart1_receiver_buffer[usart1_rxcount++]`。
2. 对二字节启动同步做纠错：
   - 如果 `buf[0] != 0xAA` 且 `buf[1] == 0xAA`，则将第二字节前移，以免发生半字节错位。
3. 至少两个字节时，若 `buf[0] == 0xAA && buf[1] == 0xAA`，认为帧开始有效，否则清空计数。
4. 当 `usart1_rxcount == 24` 时：
   - 若 `usart1_receiver_buffer[23] == 0xFF`，则：
     ```c
     memcpy(adc_value, &usart1_receiver_buffer[2], 20);
     need_report_adc = 1;
     ```
   - 否则打印 `"Invalid end byte: Expected 0xFF\n"` 并丢弃。
5. 重置计数和 `message1_begin` 标志，继续接收。

### 2.3 推荐下位 MCU 打包格式

固件当前只严格检查头和尾，以及中间 20 字节 ADC 的位置。推荐完整 24 字节布局：

| 字节索引 | 建议含义                 |
|----------|--------------------------|
| [0]      | 0xAA                     |
| [1]      | 0xAA                     |
| [2]~[21] | 20 路 ADC 数据           |
| [22]     | 预留/校验字节（可选）   |
| [23]     | 0xFF                     |

其中 `[2]~[21]` 会被复制到全局 `adc_value[20]`，随后用于发往上位机的上报协议（详见第 3 节）。

一旦 `need_report_adc` 被置位，主循环中会触发：

```c
if (need_report_adc) {
    periodic_adc_report();      // 新协议：ADC 周期上报（SubCmd=0x80）
    periodic_battery_report();  // 新协议：电池信息周期上报（SubCmd=0x81）
    need_report_adc = 0;
}
```

---

## 3. 周期性上报给上位机的扩展协议（AA AA AA）

这一部分是通过 `USART0` 向上位机周期性推送的「扩展数据帧」，头部统一为 `AA AA AA`，包含：

- `periodic_adc_report()`：ADC 周期上报（子命令 `0x80`）
- `periodic_battery_report()`：电池信息周期上报（子命令 `0x81`）

### 3.1 通用帧格式

发送端口：`USART0`（与上位机同一串口）

通用格式：

```text
Header : 0xAA 0xAA 0xAA                (3 字节)
Len    : 2 字节，大端，高字节在前     (Len_H, Len_L)
Type   : 1 字节                       (本项目定义为 0xFE)
Data   : N 字节                       (第一字节为子命令，后为有效载荷)
CRC    : 1 字节                       (普通字节累加和的低 8bit)
Tail   : 0xFF 0xFF 0xFF               (3 字节)
```

**Len 字段含义（非常重要）：**

代码中：

```c
/* 以 ADC 周期上报为例 */
const uint16_t payload_len = 1 + 20; /* 子命令 1B + 20B 数据 */
len_field = 1 + payload_len + 2;     /* Type(1) + Data(1+N) + Len(2) */
```

即：

```text
Len = Type(1) + Data(1+N) + Len自身(2)
```

不包括：3 字节 Header、1 字节 CRC、3 字节 Tail。

**CRC 计算：**

函数：`static uint8_t calc_add_crc(uint8_t *data, uint16_t length)`

- 从 `buf[3]`（Len_H）开始到最后一个 Data 字节。
- 计算方式：所有字节累加，取低 8 位。

上位机解析时应按照相同规则校验。

### 3.2 周期上报 ADC（子命令 0x80）

函数位置：`User/tool.c` -> `void periodic_adc_report(void)`

使用的外部变量：

- `extern uint8_t adc_value[20];` 由 USART1 从下位 MCU 采集得到。

帧构造关键步骤：

1. 填入 Header：`AA AA AA`
2. 设置 `payload_len = 1 + 20`（子命令 + 20 字节 ADC）
3. 计算 `len_field = 1 (Type) + payload_len + 2 (Len自身)`
4. 依次填入：
   - Len 高/低字节
   - Type = `0xFE`
   - Data[0] = SubCmd = `0x80`
   - Data[1..20] = `adc_value[0..19]`
5. 从 `Len_H` 起累加直至最后一个 ADC 字节，得到 CRC。
6. 填入 CRC 和尾部 `FF FF FF`。
7. 调用 `usart_transmit(USART0, buf, idx)` 发送。

**子命令 0x80，Data 区定义：**

| Data 字节索引 | 含义         |
|---------------|--------------|
| Data[0]       | SubCmd=0x80  |
| Data[1..20]   | 20 路 ADC 值 |

### 3.3 周期上报电池信息（子命令 0x81）

函数位置：`User/tool.c` -> `void periodic_battery_report(void)`

此协议帧打包了 **9 个 BQ40Z50 电池寄存器** 的原始数值，每个寄存器为 16bit，大端顺序输出。

#### 3.3.1 电池寄存器说明与健康度/电量计算

本项目中使用的 BQ40Z50 电池相关寄存器如下（寄存器地址均为 16 进制）：

1. **充电次数（Cycle Count）– `0x17`**
   - 统计电池经历的充放电循环次数。
2. **电池满电容量（Full Charge Capacity）– `0x10`**
   - 当前估算的「满充容量」，单位和缩放请参考 BQ40Z50 数据手册（一般为 mAh 或类似单位）。
3. **电池设计容量（Design Capacity）– `0x18`**
   - 电池设计时的额定容量（标称容量）。
4. **电池健康度（SoH）**
   - 建议由上位机根据下面关系计算：
   - **健康度(%) ≈ FullChargeCapacity / DesignCapacity**
     ```text
     健康度(%) ≈ 满电容量(0x10) / 设计容量(0x18)
     ```
     例如：
     ```text
     SoH(%) = FullChargeCapacity_raw / DesignCapacity_raw * 100
     ```
5. **剩余容量（Remaining Capacity）– `0x0F`**
   - 当前剩余容量的原始值。
6. **电池电量百分比（SOC）**
   - 上位机可根据以下关系计算：
   - **电量百分比(%) ≈ 剩余容量 / 满电容量**
     ```text
     电量(%) ≈ 剩余容量(0x0F) / 满电容量(0x10)
     ```
     即：
     ```text
     SOC(%) = RemainingCapacity_raw / FullChargeCapacity_raw * 100
     ```
7. **是否在充电（Charging Status）– `0x16`**
   - 一个 16bit 状态寄存器，包含充电/放电相关标志位。
   - 固件仅透传原始值，由上位机按 BQ40Z50 文档解析每一位的含义（例如「正在充电、正在放电、充电完成」等）。
8. **预计充满时间（Time to Full）– `0x13`**
   - 剩余充满时间的估算值，单位和缩放由 BQ40Z50 定义。
9. **电池温度（Temperature）– `0x08`**
   - 一般为 0.1K 单位的温度，固件不做换算，由上位机根据协议转换为摄氏度。
10. **电池电压（Voltage）– `0x09`**
    - 电池组电压，单位和缩放参考 BQ40Z50（常见为 mV）。
11. **电池电流（Current）– `0x0A`**
    - 电池电流（充电为正/放电为负的编码方式由 BQ40Z50 决定）。

> 说明：  
> - 固件在 I2C 读取层只返回寄存器的原始 16bit 数值，不做单位换算；  
> - 上位机应统一使用上述寄存器值，基于 BQ40Z50 官方协议完成单位转换与进一步的统计（SoC、SoH等）。

#### 3.3.2 周期帧 Data 区字段定义（子命令 0x81）

`periodic_battery_report()` 会一次性读取上述 9 个寄存器并按以下顺序打包：

| 数据顺序 | 寄存器地址 | 字段含义                 | 说明                         |
|----------|------------|--------------------------|------------------------------|
| 1        | `0x17`     | Cycle Count              | 充电循环次数                 |
| 2        | `0x10`     | Full Charge Capacity     | 当前估算满充容量             |
| 3        | `0x18`     | Design Capacity          | 电池设计容量                 |
| 4        | `0x0F`     | Remaining Capacity       | 剩余容量                     |
| 5        | `0x16`     | Charging Status          | 充放电状态标志（原始 16bit）|
| 6        | `0x13`     | Time to Full             | 预计充满时间                 |
| 7        | `0x08`     | Temperature              | 电池温度                     |
| 8        | `0x09`     | Voltage                  | 电池电压                     |
| 9        | `0x0A`     | Current                  | 电池电流                     |

**Data 区布局：**

| Data 字节索引 | 内容                                      |
|---------------|-------------------------------------------|
| Data[0]       | SubCmd = `0x81`                          |
| Data[1..2]    | Cycle Count (0x17)，高字节在前           |
| Data[3..4]    | Full Charge Capacity (0x10)，高字节在前  |
| Data[5..6]    | Design Capacity (0x18)，高字节在前       |
| Data[7..8]    | Remaining Capacity (0x0F)，高字节在前    |
| Data[9..10]   | Charging Status (0x16)，高字节在前       |
| Data[11..12]  | Time to Full (0x13)，高字节在前          |
| Data[13..14]  | Temperature (0x08)，高字节在前           |
| Data[15..16]  | Voltage (0x09)，高字节在前               |
| Data[17..18]  | Current (0x0A)，高字节在前               |

每个 16bit 值均通过宏 `PUT_16BE_FROM_VAL` 写入：

- 读取成功：写入实际 16bit 数值（高字节在前）。
- 读取失败（返回 -1）：写入 `0xFFFF` 作为无效占位。

#### 3.3.3 整帧格式总结（0x81）

- Header：`AA AA AA`
- Len：`len_field = 1(Type) + (1+18)(Data) + 2(Len) = 22`
- Type：`0xFE`
- Data：`[0]=0x81` + 9×(高字节 + 低字节) 的 16bit 寄存器值
- CRC：从 Len_H 起算至 Data 最后一个字节
- Tail：`FF FF FF`

**上位机处理建议：**

1. 同步 `AA AA AA` 帧头。
2. 读取 Len，确定整帧长度是否合理。
3. 检查 Type 是否等于 `0xFE`。
4. 确认子命令 `Data[0] == 0x81`。
5. 按上述顺序解析每个 16bit 值（大端），为 0xFFFF 的字段视为无效。
6. 在上位机层：
   - 以 `FullChargeCapacity(0x10)` 和 `DesignCapacity(0x18)` 计算电池健康度 SoH；
   - 以 `RemainingCapacity(0x0F)` 和 `FullChargeCapacity(0x10)` 计算电量百分比 SoC；
   - 按 BQ40Z50 手册解析 ChargingStatus 每一位，以判断是否在充电；
   - 结合 Temp/Voltage/Current 做曲线或告警。

---

## 4. 主循环与三类协议关系

主函数 `User/main.c`：

```c
int main(void)
{
    nvic_irq_enable(USART0_IRQn, 0, 0);
    nvic_irq_enable(USART1_IRQn, 0, 0);

    ...
    com_init0();
    com_init1();
    power_init();
    BQ40Z50_Init();

    while (1) {
        // 1. 处理来自上位机的 6 字节命令协议 (USART0)
        if (need_process_command) {
            process_command();
            need_process_command = 0;
        }

        // 2. 旧版简易周期上报（如有启用）
        if (need_periodic_report && host_bootup_ok) {
            periodic_report();   // 0x55 0xAA 形式的 ADC 周期上报
            need_periodic_report = 0;
        }

        // 3. 收到下位 MCU 的 ADC 帧后，触发新扩展协议上报 (USART1 -> USART0)
        if (need_report_adc) {
            periodic_adc_report();      // 子命令 0x80, 上报 20 路 ADC
            periodic_battery_report();  // 子命令 0x81, 上报 9 个电池寄存器
            need_report_adc = 0;
        }
    }
}
```

关系总结：

- **USART0 + 6 字节协议**：上位机主动发命令（控制电源、查询状态、电池基础信息、通知 Bootup）。
- **USART1 + 24 字节 ADC 帧**：下位 MCU 周期性推送 20 路 ADC，驱动 `need_report_adc`。
- **USART0 + AA AA AA 扩展协议**：本板将最新 ADC 和电池寄存器信息按 0x80/0x81 两种子命令打包给上位机。

---

## 5. 上位机对接要点（简要）

1. **串口参数**
   - 波特率 `115200`，8N1。
   - 建议开启接收缓冲和帧同步逻辑（寻找 `AA 55` 或 `AA AA AA`）。

2. **控制命令**
   - 使用 6 字节短帧 (`AA 55 cmd hi lo FF`) 控制电源和查询。
   - 避免连续错位，可在上位机解码时同样做帧头校正。

3. **周期数据**
   - 监听 `AA AA AA` 头的扩展帧：
     - `SubCmd=0x80`：20 路 ADC，直接可视化或用于阈值判断。
     - AA AA AA 00 18 FE 80 09 B2 0B EB 0D 64 0E 4B 0E E3 0F 2A 0F 55 0F 79 00 36 00 5C B9 FF FF FF
     - `SubCmd=0x81`：9 个电池寄存器，结合 BQ40Z50 文档还原电压、电流、温度、SoC、SoH、充电状态和预计充满时间等。
     - AA AA AA 00 16 FE 81 00 01 0B F8 0C E4 00 00 02 D0 FF FF 0B C7 2E 20 00 00 79 FF FF FF
   - 利用：
     - 满电容量(0x10) / 设计容量(0x18) -> 估算电池健康度；
     - 剩余容量(0x0F) / 满电容量(0x10) -> 实时电池电量百分比。


如需新增协议命令或拓展 Data 字段，只需在现有命令/子命令框架下增加对应处理逻辑，并与上位机解析程序同步即可。
| 名字             | 寄存器值 | 说明 |
|------------------|----------|------|
| 充电次数         | 0x17     | 电池经历的完整充放电循环次数，用于评估电池老化程度 |
| 电池满电容量     | 0x10     | 当前状态下电池可充到的最大容量 |
| 电池设计容量     | 0x18     | 电池出厂时的标称设计容量 |
| 电池健康度       | —        | 满电容量 / 设计容量，用于衡量电池健康状况 |
| 剩余容量         | 0x0F     | 当前电池中剩余的实际容量 |
| 电池电量百分比   | —        | 剩余容量 / 满电容量，表示当前电量百分比 |
| 是否在充电       | 0x16     | 指示电池当前是否处于充电状态 |
| 预计充满时间     | 0x13     | 根据当前充电状态估算的剩余充满时间 |
| 电池温度         | 0x08     | 电池当前温度，通常用于安全与寿命监测 |
| 电池电压         | 0x09     | 电池当前输出电压 |
| 电池电流         | 0x0A     | 电池当前充放电电流，正负表示充电或放电 |

注意充电状态下灯板黄灯闪烁
