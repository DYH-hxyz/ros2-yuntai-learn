// ============================================================================
// protocol_packet_handler.h —— SCS 总线协议处理器(声明)
// ----------------------------------------------------------------------------
// 对应 Python SDK 的 protocol_packet_handler.py。这是整个 SDK 的心脏：
// 它知道"SCS 总线协议"长什么样——怎么组一帧、怎么加校验、怎么收发、
// 怎么把"读/写/ping/同步读写"翻译成一帧帧字节。更上层的 sms_sts/scscl/hls
// 只负责"填参数、调这里"，真正的协议细节全部封装在下面这些函数里。
//
// 【SCS 总线帧格式】(务必先背下这个图，后面所有代码都是围绕它转的)
//   发送帧(指令包)：
//       字节0     字节1     字节2     字节3      字节4        字节5...
//     +--------+--------+--------+--------+----------+-----------+
//     |  0xFF  |  0xFF  |  ID    |  LEN   | INSTRUCTION | 参数1..n  |
//     +--------+--------+--------+--------+----------+-----------+
//                                        ... 参数区后最后 1 字节是校验和 CHKSUM
//     - 0xFF 0xFF      : 帧头，连续两个 FF 表示一帧开始(物理层字节对齐的定界)
//     - ID             : 目标舵机号；0xFE=广播
//     - LEN            : 包长 = 1(指令字节) + 参数个数
//     - INSTRUCTION    : 指令码(PING/READ/WRITE/...)
//     - 参数            : 依指令而定
//     - CHKSUM         : 校验和 = ~(ID+LEN+INSTRUCTION+所有参数) & 0xFF
//                        (即：从 ID 到校验和前一字节全部相加，取反，截断到 1 字节)
//   接收帧(状态包)：格式同上，只是字节4 从 INSTRUCTION 换成 ERROR(错误位)
//
// 【返回值的 C++ 约定】为贴近 Python 元组又符合 C++ 习惯，统一为：
//   返回值 = 通信结果(COMM_SUCCESS/COMM_xxx)，对应 Python 元组里的 comm_result
//   数据/错误 用"引用传出"(函数尾部带 & 的参数)，对应 Python 元组里其它元素。
//   (对照表在讲解 Markdown 里会给全)
// ============================================================================
#ifndef PROTOCOL_PACKET_HANDLER_H
#define PROTOCOL_PACKET_HANDLER_H

#include <cstdint>   // uint8_t
#include <string>    // std::string
#include <vector>    // std::vector

#include "port_handler.h"

namespace scs {

// 收发包最大长度限制(字节)。超过说明协议被用错了
constexpr int TXPACKET_MAX_LEN = 250;
constexpr int RXPACKET_MAX_LEN = 250;

class protocol_packet_handler {
public:
    // 构造函数：绑定一个串口句柄 + 指定字节序(protocol_end)
    //   protocol_end==0 => 小端(Low byte first)，SMS/STS 用这个
    //   protocol_end==1 => 大端(High byte first)，SCS 系列用
    // (Python __init__: self.portHandler = portHandler; self.scs_end = protocol_end)
    protocol_packet_handler(PortHandler* portHandler, int protocol_end);

    // ---- 字节序存取 --------------------------------------------------------
    int scs_getend() const;              // 查当前字节序
    void scs_setend(int e);              // 改字节序

    // 有符号数换算(SCS 的 16 位数据用最高位(bit15)当符号位，见下注释)
    int scs_tohost(int a, int b) const;  // 舵机原始值 -> 带正负的整数(解读)
    int scs_toscs(int a, int b) const;   // 带正负的整数 -> 舵机原始值(编码)

    // 多字节拼/拆(字节序感知)
    int scs_makeword(int a, int b) const;    // 两个字节拼成一个 16 位字
    int scs_makedword(int a, int b) const;   // 两个 16 位字拼成一个 32 位双字
    int scs_loword(int l) const;             // 取 32 位数的低 16 位
    int scs_hiword(int h) const;             // 取 32 位数的高 16 位
    int scs_lobyte(int w) const;             // 取 16 位数的低字节
    int scs_hibyte(int w) const;             // 取 16 位数的高字节

    double getProtocolVersion() const;       // 返回协议版本(1.0)

    // ---- 错误/结果转成人类可读字符串(打印用) -------------------------------
    std::string getTxRxResult(int result) const;   // 通信结果码 -> 描述
    std::string getRxPacketError(int error) const; // 舵机错误位 -> 描述

    // =========================================================================
    // 一、底层收发(所有上层功能的基石)
    // =========================================================================
    int txPacket(std::vector<uint8_t>& txpacket);       // 组好帧头发出去(填校验和)
    int rxPacket(std::vector<uint8_t>& rxpacket);       // 收一帧并解析校验，成功返回 COMM_SUCCESS
    // 完整的"发指令+收应答"一问一答(失败/广播则不等应答)
    int txRxPacket(std::vector<uint8_t>& txpacket,
                   std::vector<uint8_t>& rxpacket, int& error);

    // =========================================================================
    // 二、指令级 API(Python 同名函数逐个对应)
    // =========================================================================
    int ping(int scs_id, int& model_number, int& error);  // 探测舵机并读回型号
    int action(int scs_id);                               // 触发 REG_WRITE 暂存动作

    int readTx(int scs_id, int address, int length);      // 只发"读"指令(不等应答)
    int readRx(int scs_id, int length,
               std::vector<uint8_t>& data, int& error);   // 只收"读"应答
    int readTxRx(int scs_id, int address, int length,
                 std::vector<uint8_t>& data, int& error); // 发"读"+收应答(最常用)

    // 便捷版：按数据宽度读 1/2/4 字节
    int read1ByteTxRx(int scs_id, int address, uint8_t& data_read, int& error);
    int read2ByteTxRx(int scs_id, int address, int& data_read, int& error);
    int read4ByteTxRx(int scs_id, int address, int& data_read, int& error);

    // 写：TxOnly = 只发不等应答(快)；TxRx = 发+等舵机回执(能拿到错误)
    int writeTxOnly(int scs_id, int address, int length, const std::vector<uint8_t>& data);
    int writeTxRx(int scs_id, int address, int length,
                  const std::vector<uint8_t>& data, int& error);

    int write1ByteTxOnly(int scs_id, int address, uint8_t data);
    int write1ByteTxRx(int scs_id, int address, uint8_t data, int& error);
    int write2ByteTxOnly(int scs_id, int address, int data);
    int write2ByteTxRx(int scs_id, int address, int data, int& error);
    int write4ByteTxOnly(int scs_id, int address, int data);
    int write4ByteTxRx(int scs_id, int address, int data, int& error);

    // 异步写(REG_WRITE)：先暂存，收到 ACTION 才一起执行
    int regWriteTxOnly(int scs_id, int address, int length, const std::vector<uint8_t>& data);
    int regWriteTxRx(int scs_id, int address, int length,
                     const std::vector<uint8_t>& data, int& error);

    // 同步读/同步写(一条总线命令操作多个舵机)
    int syncReadTx(int start_address, int data_length,
                   const std::vector<uint8_t>& param, int param_length);
    int syncReadRx(int data_length, int param_length, std::vector<uint8_t>& rxpacket);
    int syncWriteTxOnly(int start_address, int data_length,
                        const std::vector<uint8_t>& param, int param_length);

    // 特殊指令：零点标定 / 恢复出厂(核心层保留，STS 普通使用不碰)
    int reOfsCal(int scs_id, int position, int& error);
    int reSet(int scs_id, int& error);

private:
    PortHandler* portHandler;   // 底层串口(对应 Python self.portHandler)
    int scs_end;                // 字节序 0=小端 1=大端(对应 Python self.scs_end)
};

}  // namespace scs

#endif  // PROTOCOL_PACKET_HANDLER_H
