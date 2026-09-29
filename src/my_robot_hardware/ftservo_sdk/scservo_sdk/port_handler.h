// ============================================================================
// port_handler.h —— 串口句柄类(声明)
// ----------------------------------------------------------------------------
// 对应 Python SDK 的 port_handler.py。它的职责只有一个：把"串口"包装成一个
// 好用的对象，向上层协议层提供开/关/读写/超时控制。类比 Python 的 pyserial：
//     Python:  import serial;  ser = serial.Serial(...);  ser.read/write
//     C++:    #include <serial/serial.h>; serial::Serial(...);  .read/.write
// wjwwood 的 serial 库(pip 名对应 pyserial)API 和 pyserial 几乎一一对应，
// 所以整个翻译过程非常"直译"。
//
// 【为什么超时计时要自己写，不用 serial 库的阻塞超时？】
// 因为 SCS 协议是"一问一答"，收状态包时舵机可能回得慢，也可能干脆不回。
// SDK 需要"带超时的忙等轮询"：每次只读走当前已到的字节，用本地时钟判断是否超时，
// 以便超时后能干净地放弃本次通信，不让程序卡死。所以这里保留 Python 版一整套
// packet_start_time / packet_timeout / tx_time_per_byte 计时逻辑。
// ============================================================================
#ifndef PORT_HANDLER_H
#define PORT_HANDLER_H

#include <cstdint>   // uint8_t
#include <string>    // std::string
#include <vector>    // std::vector
#include <memory>    // std::unique_ptr

// 只做前置声明(前向声明)，不在头文件里 include <serial/serial.h>。
// 好处：任何 include 本头文件的代码都不用被迫引入 serial 库，编译更快、耦合更低。
// 真正要用 ser_ 成员的地方只有 port_handler.cpp，在那里 include 即可。
namespace serial { class Serial; }

namespace scs {

// 默认波特率：1000000 (1Mbps)，Feetech SCS 总线最常见的速率
constexpr int DEFAULT_BAUDRATE = 1000000;

// 额外容差(毫秒)：实际收包时，在理论字节耗时基础上多等这么久，
// 用来吸收系统调度抖动、USB 转串口延迟等不确定性
constexpr double LATENCY_TIMER = 50.0;

class PortHandler {
public:
    // 显式构造函数：只记下端口名，真正打开串口要等 openPort()/setupPort()
    explicit PortHandler(const std::string& port_name);

    // 析构：确保退出时把串口关掉，防止占用设备文件
    ~PortHandler();

    // ---- 生命周期控制 -------------------------------------------------------
    bool openPort();                   // 用当前波特率把串口真正打开(成功返回 true)
    void closePort();                  // 关闭串口
    void clearPort();                  // 清空接收缓冲(丢掉历史残留字节，防串包)

    void setPortName(const std::string& port_name);  // 改端口名
    std::string getPortName() const;                 // 取端口名

    bool setBaudRate(int baudrate);    // 设置波特率(非法波特率返回 false)
    int getBaudRate() const;           // 取当前波特率

    // ---- 底层读写(协议层会频繁调用) ----------------------------------------
    int getBytesAvailable() const;              // 接收缓冲里现有多少字节可读
    size_t readPort(uint8_t* data, size_t length);   // 非阻塞读，返回实际读到字节数
    size_t readPort(std::vector<uint8_t>& data, size_t length);  // 读到 vector 的便捷重载
    size_t writePort(const uint8_t* packet, size_t length);      // 写一帧出去，返回实际写入数
    size_t writePort(const std::vector<uint8_t>& packet);        // 写 vector 的便捷重载

    // ---- 本地超时计时(毫秒级) ----------------------------------------------
    void setPacketTimeout(int packet_length);   // 按"预计收 n 字节"反推一个合理超时
    void setPacketTimeoutMillis(int msec);      // 直接指定超时毫秒数
    bool isPacketTimeout() const;               // 距上次 setPacketTimeout 是否已超时
    double getCurrentTime() const;              // 当前时刻(ms)
    double getTimeSinceStart() const;           // 距上次 setPacketTimeout 过了多少 ms

    // =========================================================================
    // 下面这些"公有成员变量"是对 Python 版公有属性的 1:1 直译：
    //   Python: self.is_open / self.baudrate / self.is_using / self.port_name
    // C++ 里为教学起见同样设为公有，方便对照。生产代码一般会封装成私有+访问器。
    // =========================================================================
    bool is_open = false;         // 串口是否已打开
    int baudrate = DEFAULT_BAUDRATE;   // 当前波特率
    bool is_using = false;        // "正在忙"标志：收发进行中置 true，防止并发调用
                                  // 协议层靠它判断 COMM_PORT_BUSY
    std::string port_name;        // 端口路径(如 /dev/ttyUSB0)

private:
    // ---- 由 setBaudRate 调用的真正"配置并打开串口" --------------------------
    bool setupPort(int cflag_baud);

    // 把用户给的波特率换算成底层可用值：支持则返回该值，不支持返回 -1
    int getCFlagBaud(int baudrate) const;

    // 计时相关私有成员(对应 Python self.packet_start_time 等)
    double packet_start_time = 0.0;   // 上一次 setPacketTimeout 的时刻(ms)
    double packet_timeout = 0.0;      // 允许等待的毫秒数
    double tx_time_per_byte = 0.0;    // 波特率决定的"传 1 字节耗时(ms)"，用于估算超时

    // 真正的串口对象。用 unique_ptr 是因为 serial::Serial 禁止拷贝，且想延迟到
    // setupPort 里再构造(开/关/换波特率时直接重建一个最省心)
    std::unique_ptr<serial::Serial> ser_;
};

}  // namespace scs

#endif  // PORT_HANDLER_H
