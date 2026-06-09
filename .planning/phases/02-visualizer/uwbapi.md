UWB 驱动API 指南
联系方式:
如该文档无法解决您的问题，请通过以下方式联系我们：
联系电话：0755-29162996
联系邮箱：support@aithinker.com

如需商务合作，请联系：sales@aithinker.com
如对该文档有修改建议，请联系：mosh@tech-now.com
期待您的建议，来帮助我们完善此文档，让它能帮助更多的开发者，Thanks~
获取 API 版本
说明：
此函数返回由 DW3000_DRIVER_VERSION 定义的 API 版本。
语法：
int32_t dwt_apiversion(void);
参数说明：
无
返回值：
返回 API 版本（DW3000_DRIVER_VERSION）。
示例代码：
无

---
设置本地数据结构指针
说明：
此函数将本地数据结构指针设置为指向由索引指定的本地数组中的元素。
语法：
int dwt_setlocaldataptr(unsigned int index);
参数说明：
- index：选择要指向的数组元素。必须在数组范围内，即 < DWT_NUM_DW_DEV。
返回值：
- 返回 DWT_SUCCESS 表示成功。
- 返回 DWT_ERROR 表示失败。
示例代码：
无

---
获取 TX 的 PG 延迟值
说明：
此函数返回 TX 的 PG 延迟值。
语法：
uint8_t dwt_readpgdelay(void);
参数说明：
无
返回值：
返回 uint8_t 类型的 PG 延迟值。
示例代码：
无

---
获取 VBAT 电压值
说明：
此函数返回 OTP 地址 0x8（VBAT_ADDRESS）中记录的测量电压值。必须在调用此函数之前先调用 dwt_initialise()。
语法：
uint8_t dwt_geticrefvolt(void);
参数说明：
无
返回值：
返回 8 位的 VBAT 电压值，按工厂设置。
示例代码：
无

---
获取温度值
说明：
此函数返回 OTP 地址 0x9（VTEMP_ADDRESS）中记录的温度值。必须在调用此函数之前先调用 dwt_initialise()。
语法：
uint8_t dwt_geticreftemp(void);
参数说明：
无
返回值：
返回 8 位的温度值，按工厂设置。
示例代码：
无

---
获取部件 ID
说明：
此函数返回设备的部件 ID。必须在调用此函数之前先调用 dwt_initialise()。
语法：
uint32_t dwt_getpartid(void);
参数说明：
无
返回值：
返回 32 位的部件 ID，按工厂设置。
示例代码：
无

---
获取批次 ID
说明：
此函数返回设备的批次 ID。必须在调用此函数之前先调用 dwt_initialise()。
语法：
uint32_t dwt_getpartid(void);
参数说明：
无
返回值：
返回 32 位的批次 ID，按工厂设置。
示例代码：
无

---
获取设备 ID
说明：
此函数返回设备类型和修订信息。对于 DW3000，返回值为 0xDECA0130。
语法：
uint32_t dwt_readdevid(void);
参数说明：
无
返回值：
返回 DW3000 的设备 ID，值为 0xDECA0130。
示例代码：
无

---
获取 OTP 修订版
说明：
此函数返回读取的 OTP 修订版。必须在调用此函数之前先调用 dwt_initialise()。
语法：
uint8_t dwt_otprevision(void);
参数说明：
无
返回值：
返回读取的 OTP 修订版值。
示例代码：
无

---
启用/禁用细粒度 TX 顺序
说明：
此函数用于启用或禁用细粒度 TX 顺序（默认启用）。
语法：
void dwt_setfinegraintxseq(int enable);
参数说明：
- enable：设置为 1 启用细粒度 TX 顺序，设置为 0 禁用。
返回值：
无
示例代码：
无

---
启用 LNA/PA 模式
说明：
此函数用于启用 GPIO 以支持外部 LNA（低噪声放大器）或 PA（功率放大器）功能，具体功能依赖于硬件，更多详情请参阅 DW3000 用户手册。该功能也可以用于调试，因为启用 TX 和 RX GPIO 可以帮助监控 DW3000 的活动。
注意：
启用 PA 功能时需要禁用细粒度 TX 顺序。可以使用 dwt_setfinegraintxseq() 禁用此功能。
语法：
void dwt_setlnapamode(int lna_pa);
参数说明：
- lna_pa：位域：
  - 位 0 设置为 1 时启用 LNA 功能；
  - 位 1 设置为 1 时启用 PA 功能；
  - 要禁用 LNA/PA，将这些位设置为 0。
返回值：
无
示例代码：
无

---
初始化 DW3000
说明：
此函数初始化 DW3000 收发器：读取其 DEV_ID 寄存器（地址 0x00）以验证该 IC 是否被本软件支持（例如 DW3000 32 位设备 ID 值为 0xDECA03xx）。然后执行必要的初始化操作，包括读取 OTP 存储中的 LDO、BIAS 调整和晶体修正值。
注意：
1. 在调用 dwt_configuresleep 前需要调用此函数，且 SPI 频率必须小于 3MHz。
2. 调用此函数时会读取并应用 LDO、BIAS 调整和晶体修正值。
3. 假定在设备复位或电源开启后调用此函数。
语法：
int dwt_initialise(int mode);
参数说明：
- mode：掩码，定义要读取的 OTP 值。
返回值：
- 返回 DWT_SUCCESS 表示成功。
- 返回 DWT_ERROR 表示失败。
示例代码：
无

---
设置 DW3000 状态
说明：
此函数可将 DW3000 设置为 IDLE（空闲）/IDLE_PLL 或 IDLE_RC 模式，当设备不处于 TX 或 RX 状态时使用。
语法：
void dwt_setdwstate(int state);
参数说明：
- state：
  - DWT_DW_IDLE (1)：将 DW3000 设置为 IDLE/IDLE_PLL 状态；
  - DWT_DW_INIT (0)：将 DW3000 设置为 INIT_RC 状态；
  - DWT_DE_IDLE_RC (2)：将 DW3000 设置为 IDLE_RC 状态。
返回值：
无
示例代码：
无

---
启用 GPIO 时钟
说明：
此函数用于启用 GPIO 时钟，确保 GPIO 正常工作。
语法：
void dwt_enablegpioclocks(void);
参数说明：
无
返回值：
无
示例代码：
无

---
恢复配置
说明：
此函数在设备从 DEEPSLEEP/SLEEP 状态唤醒后调用，用于恢复未自动从 AON 恢复的配置。
语法：
void dwt_restoreconfig(void);
参数说明：
无
返回值：
返回 DWT_SUCCESS 表示成功。
示例代码：
无

---
配置 STS 模式
说明：
此函数用于配置 STS 模式，例如 DWT_STS_MODE_OFF、DWT_STS_MODE_1 等。调用 dwt_configure 函数配置其他参数后，再调用此函数配置 STS 模式。
语法：
void dwt_configurestsmode(uint8_t stsMode);
参数说明：
- stsMode：例如 DWT_STS_MODE_OFF、DWT_STS_MODE_1 等。
返回值：
无
示例代码：
无

---
配置 DW3000（BU03/BU04）
说明：
此函数是 DW3000 配置的主要 API，用于配置 DW3000 和低级驱动程序。输入参数是指向 dwt_config_t 类型数据结构的指针，包含所有可配置项。dwt_config_t 结构体显示哪些项是支持的。
语法：
int dwt_configure(dwt_config_t *config);
参数说明：
- config：指向配置结构体的指针，包含设备的配置数据。
返回值：
- 返回 DWT_SUCCESS 表示成功。
- 返回 DWT_ERROR 表示失败（例如 PLL 校准失败，PLL 无法锁定）。
示例代码：
无

---
配置 TX 无线频率 (RF) 参数
说明：
此函数提供配置 TX 频谱的 API，包括功率和脉冲生成器延迟。输入参数是指向 dwt_txconfig_t 类型的数据结构指针，包含所有可配置项。
语法：
void dwt_configuretxrf(dwt_txconfig_t *config);
参数说明：
- config：指向 txrf 配置结构的指针，包含 TX 无线频率配置数据。
返回值：
无
示例代码：
无

---
重新加载 STS AES 初始值
说明：
此函数重新加载 STS AES 初始值。
语法：
void dwt_configuretxrf(dwt_txconfig_t *config);
参数说明：
无
返回值：
无
示例代码：
无

---
设置接收通道的查找表默认值
说明：
此函数根据选择的通道设置查找表的默认值。
语法：
void dwt_configmrxlut(int channel);
参数说明：
- channel：设备将要进行传输/接收的通道。
返回值：
无
示例代码：
无

---
配置 STS AES 128 位密钥
说明：
此函数配置 STS AES 128 位密钥值。默认值如下：
- [31:00] c9a375fa
- [63:32] 8df43a20
- [95:64] b5e5a4ed
- [127:96] 0738123b
语法：
void dwt_configurestskey(dwt_sts_cp_key_tpStsKey);
参数说明：
- pStsKey：指向 dwt_sts_cp_key_t 类型的结构体指针，包含生成 STS 的 AES128 密钥值。
返回值：
无
示例代码：
无

---
配置 STS AES 128 位初始值
说明：
此函数配置 STS AES 128 位初始值。默认值为 1，即 DW3000 重置时的值为 1。
语法：
void dwt_configurestsiv(dwt_sts_cp_iv_tpStsIv);
参数说明：
- pStsIv：指向 dwt_sts_cp_iv_t 类型的结构体指针，包含生成 STS 的 AES128 初始值。
返回值：
无
示例代码：
无

---
设置接收天线延迟
说明：
此函数将天线延迟（时间单位）写入 RX 寄存器。
语法：
void dwt_setrxantennadelay(uint16_t antennaDly);
参数说明：
- antennaDly：总 RX 天线延迟值，将写入 RX 寄存器。
返回值：
无
示例代码：
无

---
设置发送天线延迟
说明：
此函数将天线延迟（时间单位）写入 TX 寄存器。
语法：
void dwt_settxantennadelay(uint16_t antennaDly);
参数说明：
- antennaDly：总 TX 天线延迟值，将写入 TX 延迟寄存器。
返回值：
无
示例代码：
无

---
写入 TX 数据
说明：
此函数将用户提供的 TX 数据写入 DW3000 的 TX 缓冲区。输入参数包括数据长度（字节数）和数据字节的指针。
语法：
int dwt_writetxdata(uint16_t txDataLength, uint8_t *txDataBytes, uint16_t txBufferOffset);
参数说明：
- txDataLength：要写入 TX 缓冲区的数据总长度（字节）。注意：TX 缓冲区大小为 1024 字节。标准 PHR 模式允许传输最多 127 字节的数据（包括 2 字节 CRC）。扩展 PHR 模式允许传输最多 1023 字节的数据（包括 2 字节 CRC）。如果数据长度大于 127 字节，需设置 DWT_PHRMODE_EXT。
- txDataBytes：指向包含要发送数据的用户缓冲区的指针。
- txBufferOffset：指定 TX 缓冲区中的偏移位置，数据将从该位置开始写入。
返回值：
- 返回 DWT_SUCCESS 表示成功。
- 返回 DWT_ERROR 表示失败。
示例代码：
无

---
配置 TX 帧控制寄存器
说明：
此 API 函数在传输帧之前配置 TX 帧控制寄存器。
语法：
void dwt_writetxfctrl(uint16_t txFrameLength, uint16_t txBufferOffset, uint8_t ranging);
参数说明：
- txFrameLength：TX 消息的长度（包括 2 字节 CRC），最大为 1023。标准 PHR 模式最大支持 127 字节。如果配置大于 127 字节，则需要在 phrMode 配置中设置 DWT_PHRMODE_EXT。
- txBufferOffset：TX 缓冲区的偏移位置，从此位置开始写入数据。
- ranging：如果这是一个测距帧，则为 1，否则为 0。
返回值：
无
示例代码：
无

---
配置帧前导码长度
说明：
此函数用于配置帧的前导码长度。前导码长度可以按 8 的倍数进行配置，从 16 到 2048 符号。如果配置了非零值，则会忽略 TXPSR_PE 设置。
语法：
void dwt_setplenfine(uint8_t preambleLength);
参数说明：
- preambleLength：设置前导码的长度。值为 0 时禁用此设置，帧的长度将由 TXPSR_PE 设置决定，且需要通过 dwt_configure 函数进行配置。
返回值：
无
示例代码：
无

---
启动 TX 传输
说明：
此函数用于启动传输，输入参数指定使用的 TX 模式。
语法：
int dwt_starttx(uint8_t mode);
参数说明：
- mode：指定传输模式，可能的值如下：
  - DWT_START_TX_IMMEDIATE：立即传输（不期望响应）
  - DWT_START_TX_DELAYED：延迟传输（不期望响应），传输时间由 DX_TIME 寄存器指定
  - DWT_START_TX_DLY_REF：延迟传输（不期望响应），传输时间由 DREF_TIME 寄存器及 DX_TIME 寄存器指定
  - DWT_START_TX_DLY_RS：延迟传输（不期望响应），传输时间由 RX_TIME_0 寄存器及 DX_TIME 寄存器指定
  - DWT_START_TX_DLY_TS：延迟传输（不期望响应），传输时间由 TX_TIME_LO 寄存器及 DX_TIME 寄存器指定
  - DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED：立即传输（期望响应），传输后接收器会自动开启
  - DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED：延迟传输（期望响应），传输后接收器会自动开启
  - DWT_START_TX_CCA：如果在 PTO 时间内未检测到前导码，则发送帧
  - DWT_START_TX_CCA | DWT_RESPONSE_EXPECTED：如果在 PTO 时间内未检测到前导码，则发送帧并开启接收器
返回值：
- DWT_SUCCESS：成功
- DWT_ERROR：失败（例如延迟传输在超时后会被取消）
示例代码：
无

---
配置参考时间用于相对时序的延迟发送和接收
说明：
此函数配置用于延迟发送和接收的参考时间，分辨率为 8ns。
语法：
void dwt_setreferencetrxtime(uint32_t reftime);
参数说明：
- reftime：参考时间（与 DX_TIME 或 TX 时间戳或 RX 时间戳时间结合使用，用于定义传输时间或延迟 RX 开启时间）
返回值：
无
示例代码：
无

---
配置延迟传输时间或延迟接收开启时间
说明：
此函数配置延迟传输时间或延迟接收开启时间。
语法：
void dwt_setdelayedtrxtime(uint32_t starttime);
参数说明：
- starttime：TX/RX 开始时间，32 位值应为系统时间的高 32 位，表示发送消息的时间或开启接收器的时间。
返回值：
无
示例代码：
无

---
读取 TX 时间戳
说明：
此函数用于读取 TX 时间戳（已根据配置的天线延迟进行调整）。
语法：
void dwt_readtxtimestamp(uint8_t *timestamp);
参数说明：
- timestamp：指向一个 5 字节缓冲区的指针，用于存储读取到的 TX 时间戳。
返回值：
无
示例代码：
无

---
读取 TX 时间戳的高 32 位
说明：
此函数用于读取 TX 时间戳的高 32 位，已根据编程的天线延迟进行调整。
语法：
uint32_t dwt_readtxtimestamphi32(void);
返回值：
- 返回 TX 时间戳的高 32 位。
示例代码：
无

---
读取 TX 时间戳的低 32 位
说明：
此函数用于读取 TX 时间戳的低 32 位，已根据编程的天线延迟进行调整。
语法：
uint32_t dwt_readtxtimestamplo32(void);
返回值：
- 返回 TX 时间戳的低 32 位。
示例代码：
无

---
读取 PDOA 结果（到达相位差）
说明：
此函数用于读取 PDOA（到达相位差），它是 Ipatov 和 STS POA 之间，或者两个 STS POA 之间的相位差，具体取决于 PDOA 的操作模式。
计算公式：
要将结果转换为度数，可以使用以下公式：
float pdoa_deg = ((float)pdoa / (1 << 11)) * 180 / M_PI;
语法：
int16_t dwt_readpdoa(void);
返回值：
- 返回 PDOA 结果，单位是 [1:-11] 弧度的有符号值。
示例代码：
无

---
读取 TDOA（到达时间差）
说明：
此函数用于读取 TDOA（到达时间差）。TDOA 值从寄存器读取时为 41 位长度。实际读取的是 6 字节（48 位）。剩余的 7 位（即高位）不属于 TDOA 值，应设置为零。尽管如此，返回值仍然可以通过掩码处理来确保无误。
语法：
void dwt_readtdoa(uint8_ttdoa);
参数说明：
- tdoa：指向一个 6 字节缓冲区的指针，在调用此函数后将填充该缓冲区与 TDOA 值。
返回值：
无
示例代码：
无

---
读取 RX 时间戳（调整后的到达时间）
说明：
此函数用于读取 RX 时间戳（已根据到达时间进行调整）。
语法：
void dwt_readrxtimestamp(uint8_ttimestamp);
void dwt_readrxtimestampunadj(uint8_ttimestamp);
参数说明：
- timestamp：指向一个 5 字节缓冲区的指针，该缓冲区将在函数调用后填充 RX 时间戳。
返回值：
无
示例代码：
无

---
读取 RX 时间戳（相对于 Ipatov CIR）
说明：
此函数用于读取 RX 时间戳（已根据到达时间进行调整），与 Ipatov CIR 相关。
语法：
void dwt_readrxtimestamp_ipatov(uint8_ttimestamp);
参数说明：
- timestamp：指向一个 5 字节缓冲区的指针，该缓冲区将在函数调用后填充 RX 时间戳。
返回值：
无
示例代码：
无

---
读取 RX 时间戳（相对于 STS CIR）
说明：
此函数用于读取 RX 时间戳（已根据到达时间进行调整），与 STS CIR 相关。
语法：
void dwt_readrxtimestamp_sts(uint8_ttimestamp);
参数说明：
- timestamp：指向一个 5 字节缓冲区的指针，该缓冲区将在函数调用后填充 RX 时间戳。
返回值：
无
示例代码：
无

---
读取 RX 时间戳的高 32 位
说明：
此函数用于读取 RX 时间戳的高 32 位，已根据编程的天线延迟进行调整。
语法：
uint32_t dwt_readrxtimestamphi32(void);
参数说明：
- 无
返回值：
- 返回 RX 时间戳的高 32 位。
示例代码：
无

---
读取 RX 时间戳的低 32 位
说明：
此函数用于读取 RX 时间戳的低 32 位，已根据编程的天线延迟进行调整。
语法：
uint32_t dwt_readrxtimestamplo32(void);
参数说明：
- 无
返回值：
- 返回 RX 时间戳的低 32 位。
示例代码：
无

---
读取系统时间的高 32 位
说明：
此函数用于读取系统时间的高 32 位时间戳。
语法：
uint32_t dwt_readsystimestamphi32(void);
参数说明：
- 无
返回值：
- 返回系统时间的高 32 位。
示例代码：
无

---
关闭收发器
说明：
此函数用于关闭收发器，停止传输和接收数据。
语法：
void dwt_forcetrxoff(void);
参数说明：
- 无
返回值：
无
示例代码：
无

---
启动接收器
说明：
此函数用于启动接收器，可以立即启动或延迟启动（取决于模式参数）。如果出现“延迟”错误，只有在没有设置 DWT_IDLE_ON_DLY_ERR 标志时才会启动接收器。接收器将在启动后一直处于开启状态，监听任何消息，直到接收到有效帧、发生错误（CRC、PHY头、Reed Solomon）或超时（SFD、前导码或帧）。
语法：
int dwt_rxenable(int mode);
参数说明：
- mode: 此参数可以是以下允许的值之一：
  - DWT_START_RX_IMMEDIATE (0x00): 立即启用接收器。
  - DWT_START_RX_DELAYED (0x01): 设置延迟接收模式，如果触发“延迟”错误，则接收器将立即启用。
  - DWT_IDLE_ON_DLY_ERR (0x02): 如果由于“延迟”错误导致延迟接收失败，则如果设置了此标志，接收器不会立即重新启用，且设备将在函数退出时处于空闲状态。
  - DWT_START_RX_DLY_REF (0x04): 在指定时间启用接收器（时间在 DREF_TIME 寄存器中 + 任何 DX_TIME 寄存器中的时间）。
  - DWT_START_RX_DLY_RS (0x08): 在指定时间启用接收器（时间在 RX_TIME_0 寄存器中 + 任何 DX_TIME 寄存器中的时间）。
  - DWT_START_RX_DLY_TS (0x10): 在指定时间启用接收器（时间在 TX_TIME_LO 寄存器中 + 任何 DX_TIME 寄存器中的时间）。
返回值：
- DWT_SUCCESS: 启动接收器成功。
- DWT_ERROR: 错误（例如，如果延迟时间已经过去，则延迟接收启用将失败）。
示例代码：
无

---
启用/禁用并配置 SNIFF 模式
说明：
SNIFF 模式是一种低功耗接收模式，接收器按顺序开启和关闭，而不是始终开启。每个状态（开启/关闭）持续的时间通过以下参数指定。有关更多详细信息，请参阅 DW3000 用户手册第 4.5 节“低功耗 SNIFF 模式”。
语法：
void dwt_setsniffmode(int enable, uint8_t timeOn, uint8_t timeOff);
参数说明：
- enable: 设置为 1 启用 SNIFF 模式，设置为 0 禁用 SNIFF 模式。如果设置为 0，则其他所有参数都不生效。
- timeOn: 接收器开启阶段的持续时间，单位为 PAC 大小的倍数。计数器会自动加上 1 个 PAC 大小。最小值为 1（即开启时间为 2 PAC 大小），最大值为 15。
- timeOff: 接收器关闭阶段的持续时间，单位为 128/125 微秒（约 1 微秒）的倍数。最大值为 255。
返回值：
- 无返回值。
示例代码：
无

---
启用双接收缓冲模式
说明：
此函数启用或禁用双接收缓冲模式。
语法：
void dwt_setdblrxbuffmode(dwt_dbl_buff_state_e dbl_buff_state, dwt_dbl_buff_mode_e dbl_buff_mode);
参数说明：
- dbl_buff_state: 枚举变量，用于启用或禁用双缓冲模式。
- dbl_buff_mode: 枚举变量，用于设置接收器的自动重新启用模式。
返回值：
- 无返回值。
示例代码：
无

---
信号接收缓冲区已空
说明：
此函数通知芯片指定的接收缓冲区已空，可以用新数据填充。
语法：
void dwt_signal_rx_buff_free(void);
参数说明：
- 无
返回值：
- 无返回值。
示例代码：
无

---
启用接收超时 (SY_STAT_RFTO 事件)
说明：
此函数用于启用接收超时功能。如果接收器在启用接收后未接收到数据，则会触发超时事件。
语法：
void dwt_setrxtimeout(uint32_t time);
参数说明：
- time: 接收器在启用接收命令后保持开启的时长。该参数的单位是 1.0256 微秒（512/499.2MHz）。
如果设置为 0，则禁用超时。
返回值：
- 无返回值。
示例代码：
无

---
启用前导码超时 (SY_STAT_RXPTO 事件)
说明：
此函数用于启用前导码超时功能。如果接收器在接收期间未检测到前导码，则会触发前导码超时事件。
语法：
void dwt_setpreambledetecttimeout(uint16_t timeout);
参数说明：
- timeout: 前导码检测超时，单位为 PAC 大小的倍数。计数器会自动加上 1 PAC 大小。
最小值为 1（即超时为 2 PAC 大小）。
返回值：
- 无返回值。
示例代码：
无

---
校准本地振荡器
说明：
此函数用于校准本地振荡器的频率，因为振荡器的频率会随温度和电压变化而有所不同。
注意： 必须在调用 dwt_configuresleepcnt 之前运行此函数，以便了解计数器的单位。
语法：
uint16_t dwt_calibratesleepcnt(void);
参数说明：
- 无
返回值：
- 返回值是低功耗振荡器周期的 XTAL 周期数。低功耗振荡器的频率 = 38.4 MHz / 返回值。
示例代码：
无

---
设置睡眠计数器
说明：
此函数设置新的睡眠计数器值，程序会编程 28 位计数器的高 16 位。
注意： 必须在调用 dwt_configuresleep 之前运行此函数，且 SPI 频率必须小于 3 MHz。
语法：
void dwt_configuresleepcnt(uint16_t sleepcnt);
参数说明：
- sleepcnt: 需要设置的睡眠计数器值。
返回值：
- 无返回值。
示例代码：
无

---
配置睡眠模式 (深度睡眠和普通睡眠模式)
说明：
此函数用于配置设备进入深度睡眠（DEEP_SLEEP）和普通睡眠（SLEEP）模式，以及唤醒后的操作模式。在进入睡眠之前，设备应根据需要设置为 TX 或 RX 模式，然后在唤醒时，设备的 TX/RX 设置将被保留，设备可以立即执行所需的 TX/RX 操作。
注意：
例如，在标签操作中，深度睡眠后，设备只需加载 TX 缓冲区并发送数据帧。
语法：
void dwt_configuresleep(uint16_t mode, uint8_t wake);
参数说明：
- mode：配置设备的唤醒模式，以下为常见的配置标志：
  - DWT_PGFCAL 0x0800
  - DWT_GOTORX 0x0200
  - DWT_GOTOIDLE 0x0100
  - DWT_SEL_OPS 0x0040 | 0x0080
  - DWT_LOADOPS 0x0020
  - DWT_LOADLDO 0x0010
  - DWT_LOADDGC 0x0008
  - DWT_LOADBIAS 0x0004
  - DWT_RUNSAR 0x0002
  - DWT_CONFIG 0x0001 - 下载 AON 配置到 HIF (配置下载)
- wake：配置唤醒参数，以下为常见的唤醒标志：
  - DWT_SLP_CNT_RPT 0x40 - 睡眠计数器循环
  - DWT_PRESRVE_SLP 0x20 - 保持 SLEEP_EN 位状态，虽然在唤醒时会自动清除
  - DWT_WAKE_WK 0x10 - 从 WAKEUP PIN 唤醒
  - DWT_WAKE_CS 0x8 - 从芯片选择引脚唤醒
  - DWT_BR_DET 0x4 - 在睡眠/深度睡眠期间启用欠压检测
  - DWT_SLEEP 0x2 - 启用睡眠模式
  - DWT_SLP_EN 0x1 - 启用睡眠/深度睡眠功能
返回值：
- 无返回值。
示例代码：
无

---
清除 AON 配置
说明：
此函数用于清除 DW3000 中的 AON 配置。
语法：
void dwt_clearaonconfig(void);
参数说明：
- 无
返回值：
- 无返回值。
示例代码：
无

---
进入睡眠模式
说明：
此函数将设备置于深度睡眠或普通睡眠模式。在调用 dwt_configuresleep 配置好睡眠及唤醒参数后，才能调用此函数。
语法：
void dwt_entersleep(int idle_rc);
参数说明：
- idle_rc：如果设置为 DWT_DW_IDLE_RC，则在进入睡眠之前会清除自动 INIT2IDLE 位，唤醒后设备将保持在 IDLE_RC 状态。
返回值：
- 无返回值。
示例代码：
无

---
配置在 TX 后进入深度睡眠
说明：
此函数用于设置自动 TX 后进入深度睡眠模式的位。即在传输完成后，设备将进入深度睡眠模式。在调用 dwt_configuresleep 配置好唤醒设置后，才可使用此功能。
注意：
在此过程中，IRQ 线必须为低电平（即没有待处理的事件）。
语法：
void dwt_entersleepaftertx(int enable);
参数说明：
- enable：设置为 1 表示配置设备在 TX 后进入深度睡眠，设置为 0 禁用该功能。
返回值：
- 无返回值。
示例代码：
无

---
注册回调函数
说明：
此函数用于注册当某个事件发生时调用的回调函数。
注意：
回调函数可以为空（设置为 NULL）。在这种情况下，dwt_isr() 会正常处理事件，但不会调用对应的回调函数。
语法：
void dwt_setcallbacks(dwt_cb_t cbTxDone, dwt_cb_t cbRxOk, dwt_cb_t cbRxTo, dwt_cb_t cbRxErr, dwt_cb_t cbSPIErr, dwt_cb_t cbSPIRdy);
参数说明：
- cbTxDone：TX 确认事件回调函数。
- cbRxOk：接收成功帧事件回调函数。
- cbRxTo：接收超时事件回调函数。
- cbRxErr：接收错误事件回调函数。
- cbSPIErr：SPI 错误事件回调函数。
- cbSPIRdy：SPI 就绪事件回调函数。
返回值：
- 无返回值。
示例代码：
无

---
检查 IRQ 线状态
说明：
此函数检查 IRQ 线是否处于活动状态，通常用于替代中断处理程序。
语法：
uint8_t dwt_checkirq(void);
参数说明：
- 无
返回值：
- 返回 1 表示 IRQ 线处于活动状态，返回 0 表示未处于活动状态。
示例代码：
无

---
检查是否处于 IDLE_RC 状态
说明：
此函数用于检查 DW3000 是否处于 IDLE_RC 状态。
语法：
uint8_t dwt_checkidlerc(void);
参数说明：
- 无
返回值：
- 返回 1 表示处于 IDLE_RC 状态，返回 0 表示未处于 IDLE_RC 状态。
示例代码：
无

---
中断服务例程
说明：
这是 DW3000 的通用中断服务例程（ISR）。它会处理和报告以下事件：
- RXFCG（通过 cbRxOk 回调函数）
- TXFRS（通过 cbTxDone 回调函数）
- RXRFTO/RXPTO（通过 cbRxTo 回调函数）
- RXPHE/RXFCE/RXRFSL/RXSFDTO/AFFREJ/LDEERR（通过 cbRxErr 回调函数）
对于所有事件，相应的中断会被清除并执行必要的复位操作。另外，在 RXFCG 情况下，接收到的帧信息和帧控制会在调用回调函数前读取。如果启用了双缓冲，接收回调处理结束后，它还会在接收缓冲区之间进行切换。
注意：
此版本的 ISR 支持双缓冲，但不支持自动重新启用 RX！
在基于 PC 的系统（如使用 Cheetah 或 ARM 的 USB 到 SPI 转换器）中可能没有中断发生，但我们仍然需要以轮询的方式来处理此操作。在嵌入式系统中，应配置此函数以便在上述中断发生时触发。
语法：
void dwt_isr(void);
参数说明：
- 无
返回值：
- 无返回值。
示例代码：
无

---
设置中断
说明：
此函数用于启用指定的事件触发中断。以下事件可以在 SYS_ENABLE_LO 和 SYS_ENABLE_HI 寄存器中找到。
语法：
void dwt_setinterrupt(uint32_t bitmask_lo, uint32_t bitmask_hi, dwt_INT_options_e INT_options);
参数说明：
- bitmask_lo：设置 SYS_ENABLE_LO_ID 寄存器中将触发中断的事件位掩码。
- bitmask_hi：设置 SYS_ENABLE_HI_ID 寄存器中将触发中断的事件位掩码。
- INT_options：中断选项，指定启用中断的行为：
  - DWT_ENABLE_INT：启用位掩码中选择的中断。
  - DWT_ENABLE_INT_ONLY：强制设置位掩码中的中断状态（即直接将掩码写入寄存器）。
  - DWT_DISABLE_INT：禁用位掩码中选择的中断。
返回值：
- 无返回值。
示例代码：
无

---
设置 PAN ID
说明：
此函数用于设置 PAN ID。
语法：
void dwt_setpanid(uint16_t panID);
参数说明：
- panID：指定 PAN ID。
返回值：
- 无返回值。
示例代码：
无

---
设置 16 位短地址
说明：
此函数用于设置 16 位短地址。
语法：
void dwt_setaddress16(uint16_t shortAddress);
参数说明：
- shortAddress：设置的 16 位短地址。
返回值：
- 无返回值。
示例代码：
无

---
设置 EUI 64 位地址
说明：
此函数用于设置 64 位（长）EUI 地址。
语法：
void dwt_seteui(uint8_t *eui64);
参数说明：
- eui64：指向包含 64 位地址的缓冲区的指针。
返回值：
- 无返回值。
示例代码：
无

---
获取 DW3000 的 EUI 64 位地址
说明：
此函数用于从 DW3000 获取 64 位（长）EUI 地址。
语法：
void dwt_geteui(uint8_t *eui64);
参数说明：
- eui64：指向将存放读取到的 64 位 EUI 地址的缓冲区的指针。
返回值：
- 无返回值。
示例代码：
无

---
读取 AON 内存
说明：
此函数用于从 AON 内存读取数据。
语法：
uint8_t dwt_aon_read(uint16_t aon_address);
参数说明：
- aon_address：要读取的 AON 内存地址。
返回值：
- 返回从给定 AON 内存地址读取的 8 位数据。
示例代码：
无

---
写入 AON 内存
说明：
此函数用于向 AON 内存写入数据。
语法：
void dwt_aon_write(uint16_t aon_address, uint8_t aon_write_data);
参数说明：
- aon_address：要写入的 AON 内存地址。
- aon_write_data：要写入的 8 位数据。
返回值：
- 无返回值。
示例代码：
无

---
读取 OTP 数据
说明：
此函数用于从指定地址读取 OTP 数据，并将其存入提供的数组中。
语法：
void dwt_otpread(uint16_t address, uint32_t *array, uint8_t length);
参数说明：
- address：OTP 地址，从此地址开始读取数据。
- array：指向目标数组的指针，数据将被读取到该数组中。
- length：要读取的 32 位字的数量（数组至少需要此长度）。
返回值：
- 无返回值。
示例代码：
无

---
配置帧过滤
说明：
此函数用于启用帧过滤（默认情况下，接受任何数据和 ACK 帧，并确保目标地址正确）。
语法：
void dwt_configureframefilter(uint16_t enabletype, uint16_t filtermode);
参数说明：
- enabletype（位掩码）：启用/禁用帧过滤并配置 802.15.4 类型：
  - DWT_FF_ENABLE_802_15_4：使用 802.15.4 过滤规则
  - DWT_FF_DISABLE：禁用帧过滤
- filtermode（位掩码）：配置帧过滤选项：
  - DWT_FF_BEACON_EN：允许信标帧
  - DWT_FF_DATA_EN：允许数据帧
  - DWT_FF_ACK_EN：允许 ACK 帧
  - DWT_FF_MAC_EN：允许 MAC 控制帧
  - DWT_FF_RSVD_EN：允许保留帧类型
  - DWT_FF_MULTI_EN：允许多用途帧
  - DWT_FF_FRAG_EN：允许分片帧类型
  - DWT_FF_EXTEND_EN：允许扩展帧类型
  - DWT_FF_COORD_EN：充当协调器（可以接收无目标地址的帧，PAN ID 必须匹配）
  - DWT_FF_IMPBRCAST_EN：允许 MAC 隐式广播
返回值：
- 无返回值。
示例代码：
无

---
计算 8 位 CRC
说明：
此函数用于计算 8 位 CRC，使用多项式 P(x) = x^8 + x^2 + x^1 + x^0。
语法：
uint8_t dwt_generatecrc8(const uint8_tbyteArray, int flen, uint8_t crcInit);
参数说明：
- byteArray：要计算 CRC 的数据。
- flen：byteArray 的长度。
- crcInit：CRC 计算的初始值。
返回值：
- 返回计算得到的 8 位 CRC 值。
示例代码：
无

---
启用 SPI CRC 校验
说明：
此函数用于启用 DW3000 的 SPI CRC 校验。
语法：
void dwt_enablespicrccheck(dwt_spi_crc_mode_e crc_mode, dwt_spierrcb_t spireaderr_cb);
参数说明：
- crc_mode：SPI CRC 校验模式：
  - DWT_SPI_CRC_MODE_WR：在每次 SPI 写入时执行 CRC 校验，SPI 写入的最后一个字节需要是 8 位 CRC。如果 CRC 不匹配，将在状态寄存器中设置 SPI CRC 错误事件（SYS_STATUS_SPICRC）。
- spireaderr_cb：SPI 读取错误的回调函数指针，当 SPI 读取的 CRC 与计算的 CRC 不匹配时调用该函数。
返回值：
- 无返回值。
示例代码：
无

---
启用自动 ACK
说明：
此函数启用自动 ACK 功能。如果 responseDelayTime 参数为 0，则 ACK 会尽快发送；否则，它将在指定的延迟后（以符号为单位）发送，最大值为 255。
注意：此功能需要启用帧过滤。
语法：
void dwt_enableautoack(uint8_t responseDelayTime, int enable);
参数说明：
- responseDelayTime：ACK 发送的延迟时间（以符号为单位）。如果为非零值，ACK 会在该延迟后发送，最大值为 255。
- enable：启用或禁用自动 ACK 功能。
返回值：
- 无返回值。
示例代码：
无

---
设置接收器开机延迟
说明：
此函数设置在发送帧后接收器开启的延迟时间。
语法：
void dwt_setrxaftertxdelay(uint32_t rxDelayTime);
参数说明：
- rxDelayTime：延迟时间（20 位），单位为 UWB 微秒。
返回值：
- 无返回值。
示例代码：
无

---
软件复位 DW3000
说明：
此函数用于重置 DW3000。
注意：在调用此函数之前，SPI 速率必须小于等于 7 MHz，因为设备将在复位过程中使用 FOSC/4 作为内部复位的一部分。
语法：
void dwt_softreset(void);
参数说明：
- 无
返回值：
- 无返回值。
示例代码：
无

---
读取 RX 缓冲区数据
说明：
此函数用于从 RX 缓冲区的指定偏移位置读取数据。
语法：
void dwt_readrxdata(uint8_t *buffer, uint16_t length, uint16_t rxBufferOffset);
参数说明：
- buffer：将数据读取到的缓冲区。
- length：要读取的数据长度（单位：字节）。
- rxBufferOffset：RX 缓冲区中的偏移位置，从该位置开始读取数据。
返回值：
- 无返回值。
示例代码：
无

---
读取 RX 临时缓冲区数据
说明：
此函数用于从 RX 临时缓冲区的指定偏移位置读取数据。
语法：
void dwt_read_rx_scratch_data(uint8_t *buffer, uint16_t length, uint16_t rxBufferOffset);
参数说明：
- buffer：将数据读取到的缓冲区。
- length：要读取的数据长度（单位：字节）。
- rxBufferOffset：RX 临时缓冲区中的偏移位置，从该位置开始读取数据。
返回值：
- 无返回值。
示例代码：
无

---
读取累加器数据
说明：
此函数用于从累加器缓冲区的指定偏移位置读取 18 位数据。对于 18 位复数样本，每个样本占 6 字节（3 字节实部和 3 字节虚部）。
注意：由于内部内存访问延迟，读取累加器时，输出的第一个字节是一个虚拟字节，应当丢弃。无论从哪个子索引开始读取，这一点始终成立。
语法：
void dwt_readaccdata(uint8_t *buffer, uint16_t len, uint16_t accOffset);
参数说明：
- buffer：将数据读取到的缓冲区。
- len：要读取的数据长度（单位：字节）。
- accOffset：累加器缓冲区中的偏移位置，此值对应复数样本的索引。例如，要从第 100 个样本开始读取 10 个样本，buffer 需要至少 61 字节（每个样本 6 字节，外加 1 字节虚拟字节），accOffset 为 100。
返回值：
- 无返回值。
示例代码：
无

---
读取时钟偏移
说明：
此函数用于读取时钟偏移（与远程 DW3000 设备的频率偏移相关）。返回的带符号 16 位值应除以 16 以得到 ppm 偏移。
语法：
int16_t dwt_readclockoffset(void);
参数说明：
- 无
返回值：
- 返回带符号 12 位的时钟偏移值（s[6:-4]）。如果返回值为正，表示本地 RX 时钟运行比远程 TX 设备更快。
示例代码：
无

---
读取载波积分器值
说明：
此函数用于读取 RX 载波积分器值（与 TX 节点的频率偏移相关）。
语法：
int32_t dwt_readcarrierintegrator(void);
参数说明：
- 无
返回值：
- 返回带符号 32 位载波积分器值。如果返回值为正，表示本地 RX 时钟运行比远程 TX 设备更快。
示例代码：
无

---
启用 CIA 诊断数据
说明：
此函数用于启用 CIA 诊断数据。当启用时，以下寄存器将被记录：
IP_TOA_LO, IP_TOA_HI, STS_TOA_LO, STS_TOA_HI, STS1_TOA_LO, STS1_TOA_HI, CIA_TDOA_0, CIA_TDOA_1_PDOA, CIA_DIAG_0, CIA_DIAG_1。
语法：
void dwt_configciadiag(uint8_t enable_mask);
参数说明：
- enable_mask：启用 CIA 诊断数据的掩码：
  - DW_CIA_DIAG_LOG_MAX（0x8）：启用最大诊断寄存器集（双缓冲模式）
  - DW_CIA_DIAG_LOG_MID（0x4）：启用中等诊断寄存器集（双缓冲模式）
  - DW_CIA_DIAG_LOG_MIN（0x2）：启用最小诊断寄存器集（双缓冲模式）
  - DW_CIA_DIAG_LOG_ALL（0x1）：启用所有诊断寄存器
  - DW_CIA_DIAG_LOG_MIN（0x0）：启用简化诊断寄存器集
返回值：
- 无返回值。
示例代码：
无

---
读取 STS 信号质量
说明：
此函数用于读取 STS 信号质量指数。
语法：
int dwt_readstsquality(int16_trxStsQualityIndex);
参数说明：
- rxStsQualityIndex：输出参数，返回带符号的 STS 信号质量指数。
返回值：
- 返回值 >=0 表示信号质量良好，<0 表示信号质量差。
备注：对于 64 MHz PRF，如果值大于或等于 STS 长度的 90%，则可以认为 STS 接收良好。否则，STS 时间戳可能不准确。
示例代码：
无

---
读取 STS 状态
说明：
此函数用于读取 STS 状态。
语法：
int dwt_readstsstatus(uint16_tstsStatus, int sts_num);
参数说明：
- stsStatus：输出参数，返回 STS 状态值。
- sts_num：STS 的索引值，0 表示第一个 STS，1 表示第二个 STS（当 PDOA 模式为 3 时，第二个 STS 可用）。
返回值：
- 返回 0 表示 STS 状态良好/有效，返回值 <0 表示 STS 状态差。
示例代码：
无

---
读取 RX 信号质量诊断数据
说明：
此函数用于读取 RX 信号质量的诊断数据。
语法：
void dwt_readdiagnostics(dwt_rxdiag_t *diagnostics);
参数说明：
- diagnostics：指向诊断结构体的指针，返回从 DW3000 读取的诊断数据。
返回值：
- 无返回值。
示例代码：
无

---
启用/禁用事件计数器
说明：
此函数用于启用或禁用 IC 中的事件计数器。
语法：
void dwt_configeventcounters(int enable);
参数说明：
- enable：1 启用事件计数器并重置，0 禁用事件计数器。
返回值：
- 无返回值。
示例代码：
无

---
读取事件计数器
说明：
此函数用于读取 IC 中的事件计数器。
语法：
void dwt_readeventcounters(dwt_deviceentcnts_t *counters);
参数说明：
- counters：指向 dwt_deviceentcnts_t 结构体的指针，用于存放读取到的事件计数数据。
返回值：
- 无返回值。
示例代码：
无

---
编程 32 位值到 DW3000 OTP 存储器
说明：
此函数用于将 32 位值编程到 DW3000 的 OTP 存储器中。
语法：
int dwt_otpwriteandverify(uint32_t value, uint16_t address);
参数说明：
- value：要编程到 OTP 的 32 位值。
- address：编程到 OTP 的 16 位地址。
返回值：
- 成功时返回 DWT_SUCCESS，错误时返回 DWT_ERROR。
示例代码：
无

---
设置 Tx/Rx GPIOs 控制 LEDs
说明：
此函数用于设置 Tx/Rx GPIOs，这些 GPIOs 可用于控制 LEDs。请注意，这并不完全依赖于 IC，还需要在适当的 I/O 引脚上安装了 LED 的板子。此函数启用 GPIOs 2 和 3，连接到 EVB1000 上的 LED3 和 LED4。
语法：
void dwt_setleds(uint8_t mode);
参数说明：
- mode：这是一个位字段，解释如下：
  - 位 0：1 启用 LEDs，0 禁用 LEDs。
  - 位 1：1 初始化时让 LEDs 闪烁一次。仅在位 0 设置为启用 LEDs 时有效。
  - 位 2 到 7：保留。
返回值：
- 无返回值。
示例代码：
无

---
调整晶体频率
说明：
此函数用于调整晶体的频率。
语法：
void dwt_setxtaltrim(uint8_t value);
参数说明：
- value：晶体调节值（范围为 0x0 到 0x3F），每步约为 1.65ppm。
返回值：
- 无返回值。
示例代码：
无

---
获取晶体调节值
说明：
此函数返回在初始化过程中（dwt_init）应用的晶体调节值。此值可以是从 OTP 存储器中读取的值，也可以是默认值。
注意：此函数返回的是初始化时的值！在调用 dwt_setxtaltrim 后，此值不会更新。
语法：
uint8_t dwt_getxtaltrim(void);
参数说明：
- 无
返回值：
- 返回初始化时设置的晶体调节值。
示例代码：
无

---
启用重复帧生成
说明：
此函数用于根据帧重复率启用重复帧的生成。
语法：
void dwt_repeated_frames(uint32_t framerepetitionrate);
参数说明：
- framerepetitionrate：指定帧重复的速率。如果该值小于帧的持续时间，帧将会连续发送。
返回值：
- 无返回值。
示例代码：
无

---
启用连续重复的 CW 波形
说明：
此函数将启用设备的重复连续 CW 波形。
语法：
void dwt_repeated_cw(int cw_enable, int cw_mode_config);
参数说明：
- cw_enable：CW 模式启用标志，设置为 1 启用 CW 模式，设置为 0 禁用。
- cw_mode_config：CW 配置模式。
返回值：
- 无返回值。
示例代码：
无

---
设置 DW3000 在特定频道频率下发送 CW 信号
说明：
此函数将 DW3000 设置为在指定的频道频率下发送 CW 信号。
语法：
void dwt_configcwmode(uint8_t channel);
参数说明：
- channel：频道编号，取值为 5 或 9。
返回值：
- 无返回值。
示例代码：
无

---
配置 DW3000 进入连续发送帧模式进行合规性测试
说明：
此函数将 DW3000 配置为连续 TX 帧模式，用于合规性测试。
语法：
void dwt_configcontinuousframemode(uint32_t framerepetitionrate, uint8_t channel);
参数说明：
- framerepetitionrate：32 位值，用于设置发送间隔。最小值为 4，单位为大约 8 纳秒。
- channel：频道编号，取值为 5 或 9。
返回值：
- 无返回值。
示例代码：
无

---
读取 DW IC 的原始电池电压和温度值
说明：
此函数读取 DW IC 的原始电池电压和温度值，数据来自 DW IC 的 A/D 转换器。
语法：
uint16_t dwt_readtempvbat(void);
参数说明：
- 无
返回值：
- 返回合并的原始温度和电池电压数据，格式为 (temp_raw << 8) | (vbat_raw)。
示例代码：
无

---
转换原始温度值为实际温度
说明：
此函数将读取的原始温度值转换为实际温度。调用 dwt_initialise 后，确保 pdw3000local->tempP 包含来自 OTP 的 SAR_LTEMP 值。
语法：
float dwt_convertrawtemperature(uint8_t raw_temp);
参数说明：
- raw_temp：原始温度值，由 dwt_readtempvbat 函数读取。
返回值：
- 返回转换后的温度传感器值。
示例代码：
无

---
转换原始电压值为实际电压
说明：
此函数将读取的原始电压值转换为实际电压。调用 dwt_initialise 后，确保 pdw3000local->vBatP 包含来自 OTP 的 SAR_LVBAT 值。
语法：
float dwt_convertrawvoltage(uint8_t raw_voltage);
参数说明：
- raw_voltage：原始电压值，由 dwt_readtempvbat 函数读取。
返回值：
- 返回转换后的电压传感器值。
示例代码：
无

---
读取 DW3000 唤醒时的温度
说明：
此函数读取从睡眠/深度睡眠唤醒后采样的 DW3000 温度值。这些值不是当前值，而是上次唤醒时的值（仅在 DWT_TANDV 位设置在 dwt_configuresleep 的模式参数中时有效）。
语法：
uint8_t dwt_readwakeuptemp(void);
参数说明：
- 无
返回值：
- 返回 8 位原始温度传感器值。
示例代码：
无

---
读取 DW3000 唤醒时的电池电压
说明：
此函数读取从睡眠/深度睡眠唤醒后采样的 DW3000 电池电压值。这些值不是当前值，而是上次唤醒时的值（仅在 DWT_TANDV 位设置在 dwt_configuresleep 的模式参数中时有效）。
语法：
uint8_t dwt_readwakeupvbat(void);
参数说明：
- 无
返回值：
- 返回 8 位原始电池电压传感器值。
示例代码：
无

---
计算 DW3000 调整后的带宽设置
说明：
此函数计算 DW3000 的调整后的带宽设置（PG_DELAY 位字段设置）。该调整是 DW3000 内部 PG 校准例程的结果，根据目标计数值，试图找到最接近该计数值的 PG 延迟。
注意：手动序列化 TX 块和 TX 时钟需要为频道 5 或 9 启用。此函数假设 PLL 已处于空闲状态，在调用此函数之前，请通过调用 dwt_configure 将 PLL 配置为空闲状态。
语法：
uint8_t dwt_calcbandwidthadj(uint16_t target_count, int channel);
参数说明：
- target_count：目标 PG 计数值，用于校正带宽。
- channel：配置带宽的频道，取值为 5 或 9。
返回值：
- 返回校准完成后写入 PG_DELAY 寄存器的值。
示例代码：
无

---
计算脉冲生成器计数寄存器（PGC_STATUS）的值
说明：
此函数计算给定 PG_DELAY 对应的脉冲生成器计数器寄存器（PGC_STATUS）中的值。此值用作参考测量，并在温度变化时用于调整设备的带宽。此函数假设 PLL 已经处于空闲状态。
语法：
uint16_t dwt_calcpgcount(uint8_t pgdly, int channel);
参数说明：
- pgdly：PG_DELAY 值，范围为 0 到 63。用于控制带宽，并查找相应的计数值。
- channel：配置带宽的频道，取值为 5 或 9。
返回值：
- 返回通过提供的 PG_DELAY 值计算出的计数值（来自 PGC_STATUS）。该值作为参考，用于后续的带宽调整。
示例代码：
无

---
读取 32 位值
说明
此函数用于从 DW3000 设备的寄存器读取 32 位值。
语法
uint32_t dwt_read32bitoffsetreg(int regFileID, int regOffset);
参数说明
- regFileID：被访问的寄存器文件或缓冲区的 ID。
- regOffset：寄存器文件或缓冲区中的索引。
返回值
返回 32 位的寄存器值。
示例代码
无

---
写入 32 位值
说明
此函数用于向 DW3000 设备的寄存器写入 32 位值。
语法
void dwt_write32bitoffsetreg(int regFileID, int regOffset, uint32_t regval);
参数说明
- regFileID：被访问的寄存器文件或缓冲区的 ID。
- regOffset：寄存器文件或缓冲区中的索引。
- regval：要写入的 32 位值。
返回值
无返回值。
示例代码
无

---
读取 16 位值
说明
此函数用于从 DW3000 设备的寄存器读取 16 位值。
语法
uint16_t dwt_read16bitoffsetreg(int regFileID, int regOffset);
参数说明
- regFileID：被访问的寄存器文件或缓冲区的 ID。
- regOffset：寄存器文件或缓冲区中的索引。
返回值
返回 16 位的寄存器值。
示例代码
无

---
写入 16 位值
说明
此函数用于向 DW3000 设备的寄存器写入 16 位值。
语法
void dwt_write16bitoffsetreg(int regFileID, int regOffset, uint16_t regval);
参数说明
- regFileID：被访问的寄存器文件或缓冲区的 ID。
- regOffset：寄存器文件或缓冲区中的索引。
- regval：要写入的 16 位值。
返回值
无返回值。
示例代码
无

---
读取 8 位值
说明
此函数用于从 DW3000 设备的寄存器读取 8 位值。
语法
uint8_t dwt_read8bitoffsetreg(int regFileID, int regOffset);
参数说明
- regFileID：被访问的寄存器文件或缓冲区的 ID。
- regOffset：寄存器文件或缓冲区中的索引。
返回值
返回 8 位的寄存器值。
示例代码
无

---
写入 8 位值
说明
此函数用于向 DW3000 设备的寄存器写入 8 位值。
语法
void dwt_write8bitoffsetreg(int regFileID, int regOffset, uint8_t regval);
参数说明
- regFileID：被访问的寄存器文件或缓冲区的 ID。
- regOffset：寄存器文件或缓冲区中的索引。
- regval：要写入的 8 位值。
返回值
无返回值。
示例代码
无

---
修改 32 位寄存器
说明
此函数用于修改 DW3000 的 32 位寄存器。它通过位与（AND）和位或（OR）操作来优化 SPI 传输，只清除或设置已知的寄存器位。
语法
void dwt_modify32bitoffsetreg(const int regFileID, const int regOffset, const uint32_t _and, const uint32_t _or);
参数说明
- regFileID：被访问的寄存器文件或缓冲区的 ID。
- regOffset：寄存器文件或缓冲区中的索引。
- _and：用于逻辑与操作的 32 位值。
- _or：在逻辑与操作后，逻辑或操作的 32 位值。
返回值
无返回值。
示例代码
无

---
修改 16 位寄存器
说明
此函数用于修改 DW3000 的 16 位寄存器。它通过位与（AND）和位或（OR）操作来优化 SPI 传输，只清除或设置已知的寄存器位。
语法
void dwt_modify16bitoffsetreg(const int regFileID, const int regOffset, const uint16_t _and, const uint16_t _or);
参数说明
- regFileID：被访问的寄存器文件或缓冲区的 ID。
- regOffset：寄存器文件或缓冲区中的索引。
- _and：用于逻辑与操作的 16 位值。
- _or：在逻辑与操作后，逻辑或操作的 16 位值。
返回值
无返回值。
示例代码
无

---
修改 8 位寄存器
说明
此函数用于修改 DW3000 的 8 位寄存器。它通过位与（AND）和位或（OR）操作来优化 SPI 传输，仅清除或设置已知的寄存器位。
语法
void dwt_modify8bitoffsetreg(const int regFileID, const int regOffset, const uint8_t _and, const uint8_t _or);
参数说明
- regFileID：被访问的寄存器文件或缓冲区的 ID。
- regOffset：寄存器文件或缓冲区中的索引。
- _and：用于逻辑与操作的 8 位值。
- _or：在逻辑与操作后，逻辑或操作的 8 位值。
返回值
无返回值。
示例代码
无

---
配置 AES 密钥
说明
此函数用于在第一次使用 AES 前配置 AES 密钥。密钥会被编程到密钥寄存器。请注意，密钥寄存器仅支持 128 位密钥。
语法
void dwt_set_keyreg_128(const dwt_aes_key_t  *key);
输入参数
- key: 指向 dwt_aes_key_t 类型的指针，该指针指向将被编程到密钥寄存器的 128 位密钥。
输出参数
无返回值。
示例代码
无

---
配置 AES 块
说明
此函数用于在第一次使用之前配置 AES 块。配置数据包含 AES 的各项设置，如加密模式等。
语法
void dwt_configure_aes(const dwt_aes_config_t *pCfg);
输入参数
- pCfg: 指向 dwt_aes_config_t 类型的指针，该指针包含 AES 配置的数据。
输出参数
无返回值。
示例代码
无

---
获取 MIC 大小并转换为 AES_CFG 寄存器值
说明
此函数根据输入的 MIC 大小（以字节为单位）计算出相应的值，并转换为可以写入 AES_CFG 寄存器的值。
语法
dwt_mic_size_e dwt_mic_size_from_bytes(uint8_t mic_size_in_bytes);
输入参数
- mic_size_in_bytes: MIC 的大小（单位为字节），常见值为 4 或 8 字节。
输出参数
返回一个 dwt_mic_size_e 类型的值，表示 AES_CFG 寄存器中要写入的值。
示例代码
无

---
执行 AES 加密/解密
说明
此函数用于执行 AES 加密或解密操作。加密时，数据块根据预先配置的密钥进行加密；解密时，密文会被解密成原始数据。此函数仅支持 AES_KEY_Src_Register 模式，并且支持的数据包大小应小于 127 字节。
语法
int8_t dwt_do_aes(dwt_aes_job_t *job, dwt_aes_core_type_e core_type);
输入参数
- job: 指向 dwt_aes_job_t 类型的指针，包含待加密或解密的数据和相关的加密/解密信息。
- core_type: 指定使用的核心类型，表示加密/解密操作所使用的核心模块。
输出参数
返回一个 int8_t 类型的值，表示 AES 加密/解密操作的状态。AES_STS_ID 中的状态位标识操作是否成功。
示例代码
无

---
通过 IO 引脚唤醒设备
说明
此函数通过 IO 引脚唤醒设备。
语法
void wakeup_device_with_io(void);
输入参数
无
输出参数
无返回值。
示例代码
无

---
唤醒 IC
说明
此函数通过 IO 引脚唤醒 DW3000 集成电路（IC）。
语法
void dwt_wakeup_ic(void);
输入参数
无
输出参数
无返回值。
示例代码
无

---
写入带 CRC 校验的 SPI
说明
此函数是一个低级抽象函数，用于向 SPI 写入数据，并且在 DW3000 SPI CRC 模式下使用。此函数会将数据分为两个独立的字节缓冲区：一个用于写入头部数据，另一个用于写入主体数据。写入的头部和主体数据将一同进行 CRC 校验。
注意：此函数的实现体应定义在 deca_spi.c 文件中，且为平台特定。
语法
extern int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodylength, const uint8_t *bodyBuffer, uint8_t crc8);
输入参数
- headerLength: 要写入的头部数据的字节数。
- headerBuffer: 指向包含 headerLength 字节头部数据的缓冲区指针。
- bodylength: 要写入的数据主体的字节数。
- bodyBuffer: 指向包含 bodylength 字节数据主体的缓冲区指针。
- crc8: 计算出的 8 位 CRC 校验值，针对头部和数据主体字节。
输出参数
无
返回值
- 返回 DWT_SUCCESS 表示成功。
- 返回 DWT_ERROR 表示错误。
示例代码
无

---
写入 SPI
说明
此函数是一个低级抽象函数，用于向 SPI 写入数据。它将数据分为两个独立的字节缓冲区：一个用于写入头部数据，另一个用于写入主体数据。
注意：此函数的实现体应定义在 deca_spi.c 文件中，且为平台特定。
语法
extern int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodylength, const uint8_t *bodyBuffer);
输入参数
- headerLength: 要写入的头部数据的字节数。
- headerBuffer: 指向包含 headerLength 字节头部数据的缓冲区指针。
- bodylength: 要写入的数据主体的字节数。
- bodyBuffer: 指向包含 bodylength 字节数据主体的缓冲区指针。
输出参数
无
返回值
- 返回 DWT_SUCCESS 表示成功。
- 返回 DWT_ERROR 表示错误。
示例代码
无

---
从 SPI 读取数据
说明
此函数是一个低级抽象函数，用于从 SPI 读取数据。它将数据分为两个独立的字节缓冲区：一个用于读取头部数据，另一个用于读取主体数据。返回的数据将存储在提供的读取缓冲区中。
注意：此函数的实现体应定义在 deca_spi.c 文件中，且为平台特定。
语法
extern int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readlength, uint8_t *readBuffer);
输入参数
- headerLength: 要读取的头部数据的字节数。
- headerBuffer: 指向缓冲区的指针，用于接收头部数据。
- readlength: 要读取的主体数据的字节数。
- readBuffer: 指向缓冲区的指针，用于接收读取到的数据（请注意，缓冲区大小应为 headerLength + readlength）。
输出参数
无
返回值
- 返回 DWT_SUCCESS 表示成功，并且返回数据开始的位置。
- 返回 DWT_ERROR 表示错误。
示例代码
无

---
进入临界区并禁用中断
说明
此函数应禁用中断，并在临界区开始时调用。该函数返回禁用前的 IRQ 状态，该值将在 decamutexoff 调用时用于重新启用中断。
注意：此函数的实现体应定义在 deca_mutex.c 文件中，且为平台特定。
语法
decaIrqStatus_t decamutexon(void);
输入参数
无
输出参数
无
返回值
返回 DW3000 中断的状态。
示例代码
无

---
离开临界区并恢复中断
说明
此函数应重新启用中断，或者至少恢复其状态（该状态是由 decamutexon 返回并保存的）。该函数在临界区结束时调用。
注意：此函数的实现体应定义在 deca_mutex.c 文件中，且为平台特定。
语法
void decamutexoff(decaIrqStatus_t s);
输入参数
- s: 由 decamutexon 返回的 DW3000 中断状态。
输出参数
无
返回值
无
示例代码
无

---
等待指定时间（毫秒）
说明
此函数用于等待指定的时间（毫秒）。此函数的实现体应定义在 deca_sleep.c 文件中，且为平台特定。
语法
void deca_sleep(unsigned int time_ms);
输入参数
- time_ms: 等待的时间，以毫秒为单位。
输出参数
无
返回值
无
示例代码
无

---
等待指定时间（微秒）
说明
此函数用于等待指定的时间（微秒）。此函数的实现体应定义在 deca_sleep.c 文件中，且为平台特定。
语法
void deca_usleep(unsigned long time_us);
输入参数
- time_us: 等待的时间，以微秒为单位。
输出参数
无
返回值
无
示例代码
无

---
检查设备 ID
说明
此函数用于读取设备 ID 并检查是否为正确的设备 ID。
语法
int dwt_check_dev_id(void);
输入参数
无
输出参数
无
返回值
- 返回 DWT_SUCCESS 表示成功。
- 返回 DWT_ERROR 表示错误。
示例代码
无

---
运行 PGF 校准
说明
此函数运行 PGF 校准，这是在接收前所必需的。如果 RX 校准例程失败，设备接收性能将受到严重影响，应用程序应重置设备并再次尝试。
语法
int dwt_run_pgfcal(void);
输入参数
- ldoen: 如果设置为 1，函数将在校准前启用 LDO，并在校准后禁用 LDO。
输出参数
无
返回值
- 返回 PGF 校准结果。DWT_ERROR（-1）表示错误。
示例代码
无

---
运行 PGF 校准（另一版本）
说明
此函数也是运行 PGF 校准，适用于需要接收之前的校准。如果 RX 校准失败，设备接收性能将受到严重影响，应用程序应重置设备并再次尝试。
语法
int dwt_pgf_cal(int ldoen);
输入参数
- ldoen: 如果设置为 1，函数将在校准前启用 LDO，并在校准后禁用 LDO。
输出参数
无
返回值
- 返回 PGF 校准结果，0 表示错误。
示例代码
无

---
配置低功耗设备地址
说明
此函数用于将一个 16 位地址写入目标低功耗设备（LE）的地址。当帧过滤配置中的正确位被设置时，帧待处理功能将生效。有关更多详细信息，请参见 dwt_configureframefilter。
语法
void dwt_configure_le_address(uint16_t addr, int leIndex);
输入参数
- addr: 要写入所选 LE 寄存器的地址值。
- leIndex: 要写入的低功耗设备（LE）地址。
输出参数
无
返回值
无
示例代码
无

---
配置 SFD 类型
说明
此函数仅配置 SFD 类型，例如 IEEE 4a - 8、DW-8、DW-16 或 IEEE 4z - 8（以二进制格式）。应在此之前调用 dwt_configure 以配置其他参数。
语法
void dwt_configuresfdtype(uint8_t sfdType);
输入参数
- sfdType: 要配置的 SFD 类型。例如：DWT_SFD_IEEE_4A、DWT_SFD_DW_8、DWT_SFD_DW_16、DWT_SFD_IEEE_4Z。
输出参数
无
返回值
无
示例代码
无

---