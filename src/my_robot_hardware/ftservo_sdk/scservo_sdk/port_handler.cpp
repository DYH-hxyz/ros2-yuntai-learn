// ============================================================================
// port_handler.cpp —— 串口句柄类(实现)
// ----------------------------------------------------------------------------
// 对应 Python port_handler.py。串口底层的差异点是：
//   Python 用 pyserial(serial.Serial)；C++ 用 wjwwood/serial(serial::Serial)。
// 两者在"配置项、读写语义"上几乎一一对应，逐行对照即可看懂。
// ============================================================================

#include <chrono>   // std::chrono 高精度时钟
#include <thread>   // (本文件暂未直接使用，保留为后续扩展)

// 引入 wjwwood serial 库(真正的串口实现)
#include <serial/serial.h>

#include "port_handler.h"

namespace scs {

// ============================================================================
// 构造函数：Python __init__ 里做的事，这里逐一对应
// ============================================================================
PortHandler::PortHandler(const std::string& port_name)
    : port_name(port_name)   // 只把端口路径记下来，此时还没打开串口
{
    // 其余成员(baudrate=1M、is_open=false、is_using=false 等)在头文件里已有
    // 默认值(花括号初始化)，无需在构造函数体里再赋值。
    // 对应 Python：self.packet_start_time = 0.0; 等那几行。
}

// 析构函数：C++ RAII 思想——对象销毁时自动关串口，防止程序退出后设备被占用
PortHandler::~PortHandler()
{
    if (is_open) {
        closePort();   // 串口还开着就先关掉
    }
}

// ============================================================================
// openPort()：用"当前设置的波特率"打开串口
//   Python: return self.setBaudRate(self.baudrate)
//   打开的过程其实就是"按波特率重新配置一次串口"，所以直接复用 setBaudRate
// ============================================================================
bool PortHandler::openPort()
{
    return setBaudRate(baudrate);
}

// 关闭串口
void PortHandler::closePort()
{
    if (ser_ && ser_->isOpen()) {
        ser_->close();       // 调用 serial 库关闭底层文件描述符
    }
    is_open = false;         // 同步标志位
}

// ============================================================================
// clearPort()：在"发一帧之前"调用，把接收缓冲里上一次可能残留的字节清掉，
// 避免上一帧没读完的脏数据混进本次应答的解析里(防"串包")。
//   Python: self.ser.flush()   (这里语义上要的是"丢弃输入缓冲"，故用 flushInput)
// ============================================================================
void PortHandler::clearPort()
{
    if (ser_ && ser_->isOpen()) {
        ser_->flushInput();   // 只冲刷"接收"缓冲(丢弃已到但未读的字节)
    }
}

void PortHandler::setPortName(const std::string& name)
{
    port_name = name;
}

std::string PortHandler::getPortName() const
{
    return port_name;
}

// ============================================================================
// setBaudRate(baudrate)：设置波特率
//   1) 先判断这个波特率是不是底层支持的(非标波特率直接拒绝)
//   2) 支持就用它去(重新)配置并打开串口
//   Python: 通过 getCFlagBaud 检查后 setupPort
// ============================================================================
bool PortHandler::setBaudRate(int baud)
{
    int cflag = getCFlagBaud(baud);   // 校验并换算波特率

    if (cflag <= 0) {
        // 不支持的波特率：Python 原版这里留了 TODO(自定义波特率未实现)，直接失败
        return false;
    }
    baudrate = baud;                  // 记住用户要的波特率(成员变量更新)
    return setupPort(cflag);          // 真正去配置并打开串口
}

int PortHandler::getBaudRate() const
{
    return baudrate;
}

// 接收缓冲里现有多少字节(对应 Python self.ser.in_waiting)
int PortHandler::getBytesAvailable() const
{
    if (ser_ && ser_->isOpen()) {
        return static_cast<int>(ser_->available());
    }
    return 0;
}

// ============================================================================
// readPort：非阻塞读取。ser 在 setupPort 时被配成"全零超时"= 非阻塞，
// 等价 pyserial 的 timeout=0：能读到多少返回多少(可能 0)，绝不卡住。
// 上层协议层自己用忙等+超时时钟来凑齐一整个包，这正是我们要的行为。
//   Python: return self.ser.read(length)
// ============================================================================
size_t PortHandler::readPort(uint8_t* data, size_t length)
{
    if (!ser_ || !ser_->isOpen() || length == 0) {
        return 0;                     // 没开/要 0 字节：直接返回
    }
    try {
        // serial 库 read 返回"实际读到的字节数"(0 ~ length)
        return ser_->read(data, length);
    } catch (...) {
        // 读出错(如设备被拔)就按没读到处理，把异常吞掉交给上层超时逻辑兜底
        return 0;
    }
}

// 重载：读进 std::vector<uint8_t>。先保证缓冲够大，再委托给上面的指针版本
size_t PortHandler::readPort(std::vector<uint8_t>& data, size_t length)
{
    if (data.size() < length) {
        data.resize(length, 0);       // 不够就扩容
    }
    return readPort(data.data(), length);
}

// 写一帧出去(对应 Python self.ser.write(packet))
size_t PortHandler::writePort(const uint8_t* packet, size_t length)
{
    if (!ser_ || !ser_->isOpen() || length == 0) {
        return 0;
    }
    try {
        return ser_->write(packet, length);   // 返回实际写入字节数
    } catch (...) {
        return 0;
    }
}

// 重载：写 std::vector<uint8_t>
size_t PortHandler::writePort(const std::vector<uint8_t>& packet)
{
    if (!ser_ || !ser_->isOpen() || packet.empty()) {
        return 0;
    }
    try {
        return ser_->write(packet.data(), packet.size());
    } catch (...) {
        return 0;
    }
}

// ============================================================================
// setPacketTimeout(packet_length)：根据"预计要收 packet_length 个数据字节"，
// 自动推算一个合理的总超时时间并启动计时。
//   估算依据 = 每字节耗时 x (长度 + 3) + 额外容差
//              ^^^^^^^^^^^^^^^^^   ^^         ^^^^
//              (LEN+ERR+CHK 多3字节)          (系统/转换器延迟)
//   每字节耗时 tx_time_per_byte 在 setupPort 里按波特率算好。
//   Python 原式：tx_time_per_byte*packet_length + tx_time_per_byte*3.0 + LATENCY_TIMER
// ============================================================================
void PortHandler::setPacketTimeout(int packet_length)
{
    packet_start_time = getCurrentTime();   // 记下"现在"作为起跑线
    packet_timeout =
        (tx_time_per_byte * packet_length) +   // 收数据字节所需时间
        (tx_time_per_byte * 3.0) +             // 加上 LEN/ERR/CHK 这 3 个固定字节
        LATENCY_TIMER;                         // 再加系统容差
}

// 直接指定一个固定的超时毫秒数(某些场景用不上字节耗时估算，如同步读整串)
void PortHandler::setPacketTimeoutMillis(int msec)
{
    packet_start_time = getCurrentTime();   // 重置起跑线
    packet_timeout = msec;                  // 直接用给定的毫秒数
}

// 判断是否已经超过设定的超时时间
bool PortHandler::isPacketTimeout() const
{
    if (getTimeSinceStart() > packet_timeout) {   // 已等待时间 > 允许时间 => 超时
        // 注意：这里不清成员(返回 const)。超时判断是"纯查询"，由调用方决定怎么处理
        return true;
    }
    return false;
}

// 取当前时刻(毫秒)。用 steady_clock(单调时钟)，避免系统改时间导致计时错乱
double PortHandler::getCurrentTime() const
{
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    // 转成微秒再除以 1000 => 毫秒(带小数)，精度足够应付亚毫秒级串口时序
    return std::chrono::duration_cast<std::chrono::microseconds>(now).count() / 1000.0;
}

// 距上次 setPacketTimeout 过去了多少毫秒
double PortHandler::getTimeSinceStart() const
{
    double elapsed = getCurrentTime() - packet_start_time;
    if (elapsed < 0.0) {
        // 理论上单调时钟不会回退；这里仿照 Python 加一道保险：
        // 若出现负值(异常)，就把起点重置为当前，返回 0
        return 0.0;
    }
    return elapsed;
}

// ============================================================================
// setupPort(cflag_baud)：真正按波特率"配置串口参数 + 打开"。
// 与 Python 的不同点：Python 每次用 pyserial 新建 Serial 对象；
// C++ 这里也是"先关旧、再新建/重配 serial::Serial"。
//   关键配置：
//     波特率     = baudrate        (如 1000000)
//     数据位     = 8
//     校验位     = None (无校验)
//     停止位     = 1
//     流控       = None
//     超时       = Timeout() 全零 => 非阻塞(等价 pyserial timeout=0)
// SCS 舵机总线就是标准 UART 8N1，这些是必须对齐的物理层参数。
// ============================================================================
bool PortHandler::setupPort(int cflag_baud)
{
    if (is_open) {
        closePort();   // 已打开就先关，保证下面从干净状态重新配置
    }

    try {
        // 构造一个默认(未打开)的串口对象。unique_ptr 重新分配 = 丢弃旧状态
        ser_ = std::make_unique<serial::Serial>();

        ser_->setPort(port_name);          // 指定设备路径，如 /dev/ttyUSB0
        ser_->setBaudrate(baudrate);       // 存波特率(打开时生效)

        // 全零 Timeout = 非阻塞模式：read/write 立即返回，不等满指定字节
        // 这就是 pyserial 里 timeout=0 的含义，逐字对应 Python 的 timeout=0
        serial::Timeout nonblocking;
        ser_->setTimeout(nonblocking);

        ser_->setBytesize(serial::eightbits);   // 8 数据位
        ser_->setParity(serial::parity_none);   // 无校验
        ser_->setStopbits(serial::stopbits_one); // 1 停止位
        ser_->setFlowcontrol(serial::flowcontrol_none);  // 无硬件/软件流控

        ser_->open();                       // 真正 open 设备文件
        ser_->setBaudrate(baudrate);        // 保险起见打开后再设一次，确保 termios 生效

        // 打开成功后立刻丢弃接收缓冲里的历史残留(防上电瞬间的乱码)
        ser_->flushInput();
    } catch (...) {
        // 打开失败(设备不存在/被占用/权限不足)会抛异常，这里统一转成 false 返回
        is_open = false;
        return false;
    }

    is_open = true;

    // 计算"传 1 字节需要多少毫秒"：
    //   波特率 = 每秒传多少 bit；UART 一字节实际占 10 bit(1起始+8数据+1停止)
    //   所以 1 字节耗时(ms) = 10 / (baudrate/1000) = (1000.0/baudrate)*10.0
    // 这个值后面用来估算"收一整个包要多长时间"，从而设定合理超时
    tx_time_per_byte = (1000.0 / baudrate) * 10.0;

    return true;
}

// ============================================================================
// getCFlagBaud(baudrate)：检查波特率是否被支持。
// 只有出现在白名单里的才放行(Feetech 官方支持这些常见速率)；
// 其余(如 1.5M、非标值)返回 -1，让上层判定"不支持"。
//   注意：Linux 下其实能设任意自定义波特率，但 Python 版 SDK 就没实现，
//         这里保持一致，先只支持标准速率。
// ============================================================================
int PortHandler::getCFlagBaud(int baud) const
{
    // 白名单里的标准波特率
    static const int supported[] = {
        4800, 9600, 14400, 19200, 38400, 57600,
        115200, 128000, 250000, 500000, 1000000
    };
    for (int b : supported) {
        if (b == baud) {
            return baud;   // 支持：直接返回该波特率数值
        }
    }
    return -1;             // 不支持
}

}  // namespace scs
