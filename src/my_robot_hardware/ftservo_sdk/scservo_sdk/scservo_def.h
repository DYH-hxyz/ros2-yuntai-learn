// ============================================================================
// scservo_def.h —— SCS 总线协议全局常量定义
// ----------------------------------------------------------------------------
// 对应 Python SDK 里的 scservo_def.py。这个文件里没有"函数"，只有一堆常数，
// 它们是整个 SDK 的地基：指令码、通信结果码、错误位、ID 边界。
//
// 为什么把这些"魔法数字"抽出来起名字？
//   因为 SCS 总线协议是字节流协议，所有通信最终都要拼成:
//       FF FF | ID | 长度 | 指令 | 参数... | 校验和
//   如果我们到处写裸数字 0x83、0xFE，代码读起来就像天书；起了名字后，
//   "我要同步写" 就写 INST_SYNC_WRITE，一眼看懂。这就是"见名知义"。
//
// 【命名空间 scs】C++ 里用 namespace 把这一堆全局常量圈起来，避免污染全局。
//   使用方写 using namespace scs; 后，用法就跟 Python 的 from xxx import * 一样了。
// ============================================================================
#ifndef SCSERVO_DEF_H
#define SCSERVO_DEF_H

namespace scs {

// ============================================================================
// 一、总线 ID 相关
// ============================================================================
constexpr int BROADCAST_ID = 0xFE;  // 254，广播 ID：发给"总线上所有舵机"
                                     // 收到广播的舵机【不回复】状态包（否则一堆舵机同时回，
                                     // 串口立刻打架）。所以 SDK 里凡是 ID==BROADCAST_ID 的包
                                     // 都只发不收、不等应答。
constexpr int MAX_ID       = 0xFC;  // 252，合法 ID 上限。>252 的 ID 非法
constexpr int SCS_END      = 0;     // 字节序标记：0 = 小端(Little-Endian, 低字节在前)。
                                     // SMS/STS 系列用 0(小端)；SCS 系列用 1(大端)。

// ============================================================================
// 二、SCS 协议指令码（Instruction）
//    每个指令包的第 4 字节(PKT_INSTRUCTION)填这些值，告诉舵机"你想让我干啥"
// ============================================================================
constexpr int INST_PING       = 1;    // 0x01 Ping：问舵机在不在(顺便拿回错误状态)
constexpr int INST_READ       = 2;    // 0x02 读：从某个寄存器地址读若干字节
constexpr int INST_WRITE      = 3;    // 0x03 写：把若干字节写到某个寄存器地址
constexpr int INST_REG_WRITE  = 4;    // 0x04 异步写(RegWrite)：先暂存到舵机内部寄存器，
                                       //   等收到 ACTION(0x05) 再统一生效
constexpr int INST_ACTION     = 5;    // 0x05 触发：让之前 REG_WRITE 暂存的数据一起执行
constexpr int INST_SYNC_WRITE = 131;  // 0x83 同步写：一条指令同时给多个舵机写同段寄存器
constexpr int INST_SYNC_READ  = 130;  // 0x82 同步读：一条指令让多个舵机依次上报同段寄存器
constexpr int INST_RESET      = 10;   // 0x0A 复位：恢复出厂设置(慎用！会把 ID/波特率也重置)
constexpr int INST_OFSCAL     = 11;   // 0x0B 零点标定：把当前位置标定为 0(中位)

// ============================================================================
// 三、通信结果码（CommResult）—— 每个收发函数返回的"这单干成没干成"
//    这些是负数。0 表示成功；负数越低档越惨。最后打印时靠它查表给人类看
// ============================================================================
constexpr int COMM_SUCCESS        = 0;   // 通信成功
constexpr int COMM_PORT_BUSY      = -1;  // 串口正忙(被上一次收发占用，防止并发踩踏)
constexpr int COMM_TX_FAIL        = -2;  // 发送失败(写进串口的字节数不对)
constexpr int COMM_RX_FAIL        = -3;  // 接收失败
constexpr int COMM_TX_ERROR       = -4;  // 指令包本身不对(比如超长)
constexpr int COMM_RX_WAITING     = -5;  // 正在等状态包(还没等完)
constexpr int COMM_RX_TIMEOUT     = -6;  // 等超时了(舵机没回应/回应太慢)
constexpr int COMM_RX_CORRUPT     = -7;  // 收到的包校验和不对(线路上有干扰/波特率错)
constexpr int COMM_NOT_AVAILABLE  = -9;  // 当前协议/参数不支持这个操作

// ============================================================================
// 四、协议包内字节位置（PKT = PacKeT）
//    发送/接收包都是字节数组，这些常量是"数组下标"，标出每一段在包里的位置。
//    配合上面的帧格式理解：
//       下标:  0     1     2    3       4          5...
//       内容: [FF]  [FF]  [ID] [LEN]  [INST/ERR]  [参数0]...
// ============================================================================
constexpr int PKT_HEADER0     = 0;   // 包头第 0 字节：恒为 0xFF(帧头标志)
constexpr int PKT_HEADER1     = 1;   // 包头第 1 字节：恒为 0xFF(连续两个 FF = 一帧开始)
constexpr int PKT_ID          = 2;   // 第 2 字节：目标舵机 ID
constexpr int PKT_LENGTH      = 3;   // 第 3 字节：包长(LEN)——从"长度"自己之后到校验和之前
                                       //   一共有几个字节 = 1(INST) + 参数个数
constexpr int PKT_INSTRUCTION = 4;   // 第 4 字节：指令码【发送包用】
constexpr int PKT_ERROR       = 4;   // 第 4 字节：错误位【接收包用】(和 INSTRUCTION 同一位置！)
constexpr int PKT_PARAMETER0  = 5;   // 第 5 字节起：参数区起点。具体参数下标 = PKT_PARAMETER0+n

// ============================================================================
// 五、状态包(舵机回包)里"错误字节"的各个错误位 (每个位代表一种异常)
//    舵机回的状态包第 4 字节是 Error，把下面某一位拉高 = 发生对应错误
//    比如 Error == 0x02(bit1) 就是角度传感器错误
// ============================================================================
constexpr int ERRBIT_VOLTAGE  = 1;   // bit0：电压异常(供电过高/过低)
constexpr int ERRBIT_ANGLE    = 2;   // bit1：角度传感器异常(电位器/磁编码器故障)
constexpr int ERRBIT_OVERHEAT = 4;   // bit2：过热(超温保护)
constexpr int ERRBIT_OVERELE  = 8;   // bit3：过流(电流超限)
constexpr int ERRBIT_OVERLOAD = 32;  // bit5：过载(堵转/负载过大)

}  // namespace scs

#endif  // SCSERVO_DEF_H
