// ============================================================================
// protocol_packet_handler.cpp —— SCS 总线协议处理器(实现)
// ----------------------------------------------------------------------------
// 逐函数对照 Python protocol_packet_handler.py 翻译。
// 阅读前请先看头文件顶部的【SCS 总线帧格式】图。
// ============================================================================

#include <cstdint>
#include <string>
#include <vector>

#include "scservo_def.h"              // 协议常量(指令码/错误位/COMM_*)
#include "protocol_packet_handler.h"  // 本类声明

namespace scs {

// ============================================================================
// 构造函数：等价 Python __init__
//   self.portHandler = portHandler
//   self.scs_end = protocol_end
// ============================================================================
protocol_packet_handler::protocol_packet_handler(PortHandler* ph, int protocol_end)
    : portHandler(ph),      // 记住底层串口(后面所有收发都走它)
      scs_end(protocol_end) // 记住字节序(0 小端 / 1 大端)
{
}

// ---- 字节序读写(直译) ------------------------------------------------------
int protocol_packet_handler::scs_getend() const { return scs_end; }
void protocol_packet_handler::scs_setend(int e) { scs_end = e; }

// ============================================================================
// scs_tohost(a, b)：把舵机原始读数转成"带正负的整数"。
//   a 是 16 位原始值，b 是符号位所在的位置(通常 15，即最高位)。
//   思路：如果第 b 位为 1(负数标志)，就返回 -(去掉符号位后的值)；否则原样返回。
//   举例：角度/速度这类数据，舵机用 bit15 表示方向(1=负)。
//         原始值 0x83E8 (bit15=1) -> tohost => -(0x03E8) = -1000
//   Python: if (a & (1<<b)): return -(a & ~(1<<b)) else: return a
// ============================================================================
int protocol_packet_handler::scs_tohost(int a, int b) const
{
    if (a & (1 << b)) {
        return -(a & ~(1 << b));   // 负值：保留除符号位外的数值并取负
    }
    return a;                      // 正值：原样
}

// ============================================================================
// scs_toscs(a, b)：scs_tohost 的反操作。把带符号整数编码成舵机认识的 16 位原始值
//   若为负，则置第 b 位为 1：(-a) | (1<<b)
//   例：-1000 -> (-(-1000))|0x8000 = 1000|0x8000 = 0x83E8
// ============================================================================
int protocol_packet_handler::scs_toscs(int a, int b) const
{
    if (a < 0) {
        return (-a) | (1 << b);    // 负数：绝对值 + 符号位置 1
    }
    return a;
}

// ============================================================================
// scs_makeword(a, b)：把两个字节 a(低)、b(高) 按"当前字节序"拼成一个 16 位整数
//   小端(scs_end==0，SMS/STS)：结果 = a | (b<<8)   (a 是低字节在前)
//   大端(scs_end==1，SCS)     ：结果 = b | (a<<8)   (a 是高字节在前)
//   注意最终数值无关端序(组合出的数值一定正确)，端序只影响"谁在低位"。
// ============================================================================
int protocol_packet_handler::scs_makeword(int a, int b) const
{
    if (scs_end == 0) {
        return (a & 0xFF) | ((b & 0xFF) << 8);   // 小端：a=低字节
    }
    return (b & 0xFF) | ((a & 0xFF) << 8);       // 大端：a=高字节
}

// 两个 16 位字(a=低字, b=高字)拼成 32 位(双字数据不用管端序，直接拼)
int protocol_packet_handler::scs_makedword(int a, int b) const
{
    return (a & 0xFFFF) | ((b & 0xFFFF) << 16);
}

// 取 32 位整数的低 16 位
int protocol_packet_handler::scs_loword(int l) const { return l & 0xFFFF; }

// 取 32 位整数的高 16 位
int protocol_packet_handler::scs_hiword(int h) const { return (h >> 16) & 0xFFFF; }

// 取 16 位整数的"低字节"(按字节序解释：小端 = 真正低 8 位；大端 = 高 8 位)
int protocol_packet_handler::scs_lobyte(int w) const
{
    if (scs_end == 0) {
        return w & 0xFF;
    }
    return (w >> 8) & 0xFF;
}

// 取 16 位整数的"高字节"
int protocol_packet_handler::scs_hibyte(int w) const
{
    if (scs_end == 0) {
        return (w >> 8) & 0xFF;
    }
    return w & 0xFF;
}

double protocol_packet_handler::getProtocolVersion() const
{
    return 1.0;   // SCS 协议版本 1.0
}

// ============================================================================
// getTxRxResult / getRxPacketError：把数字结果翻译成人话(用于打日志)
// ============================================================================
std::string protocol_packet_handler::getTxRxResult(int result) const
{
    if (result == COMM_SUCCESS)       return "[TxRxResult] Communication success!";
    if (result == COMM_PORT_BUSY)     return "[TxRxResult] Port is in use!";
    if (result == COMM_TX_FAIL)       return "[TxRxResult] Failed transmit instruction packet!";
    if (result == COMM_RX_FAIL)       return "[TxRxResult] Failed get status packet from device!";
    if (result == COMM_TX_ERROR)      return "[TxRxResult] Incorrect instruction packet!";
    if (result == COMM_RX_WAITING)    return "[TxRxResult] Now receiving status packet!";
    if (result == COMM_RX_TIMEOUT)    return "[TxRxResult] There is no status packet!";
    if (result == COMM_RX_CORRUPT)    return "[TxRxResult] Incorrect status packet!";
    if (result == COMM_NOT_AVAILABLE) return "[TxRxResult] Protocol does not support this function!";
    return "";
}

std::string protocol_packet_handler::getRxPacketError(int error) const
{
    if (error & ERRBIT_VOLTAGE)  return "[ServoStatus] Input voltage error!";
    if (error & ERRBIT_ANGLE)    return "[ServoStatus] Angle sen error!";
    if (error & ERRBIT_OVERHEAT) return "[ServoStatus] Overheat error!";
    if (error & ERRBIT_OVERELE)  return "[ServoStatus] OverEle error!";
    if (error & ERRBIT_OVERLOAD) return "[ServoStatus] Overload error!";
    return "";   // 无错误位
}

// ============================================================================
// txPacket(txpacket)：发送一帧指令包
//   入参 txpacket 是一个"只填了 ID/LEN/INST/参数"的字节数组，
//   本函数负责补上：帧头 FF FF、按协议算好并写入最后 1 字节校验和，然后发出去。
//   对应 Python txPacket 里 checksum 那一整段。
// ============================================================================
int protocol_packet_handler::txPacket(std::vector<uint8_t>& txpacket)
{
    int checksum = 0;
    // 帧总长 = LEN + 4(固定开销：HEADER0 HEADER1 ID LENGTH 这 4 字节)
    int total_packet_length = txpacket[PKT_LENGTH] + 4;

    // ---- 互斥保护：串口正忙就不发，避免两线程同时操作串口 ----------------
    if (portHandler->is_using) {
        return COMM_PORT_BUSY;
    }
    portHandler->is_using = true;   // 占用串口(直到收完或函数结束释放)

    // ---- 长度检查：超长直接拒绝(协议最多 250) ------------------------------
    if (total_packet_length > TXPACKET_MAX_LEN) {
        portHandler->is_using = false;   // 记得释放占用
        return COMM_TX_ERROR;
    }

    // ---- 补帧头：连续两个 0xFF 标志一帧开始 ---------------------------------
    txpacket[PKT_HEADER0] = 0xFF;
    txpacket[PKT_HEADER1] = 0xFF;

    // ---- 计算校验和：累加 [ID .. 校验和前一字节] ----------------------------
    // 下标 2(第一个 ID) 起，到 total-2(校验和前一个)止
    for (int idx = 2; idx < total_packet_length - 1; ++idx) {
        checksum += txpacket[idx];
    }
    // 校验和 = 总和的"按位取反"再截断到低 8 位
    // (这样 舵机侧 把整包[除帧头外]累加 + 校验和 == 0xFF，可自检)
    txpacket[total_packet_length - 1] = static_cast<uint8_t>(~checksum & 0xFF);

    // ---- 发送前清一下接收缓冲，丢掉可能残留的旧字节 ------------------------
    portHandler->clearPort();

    // ---- 真正写进串口；必须只写 total_packet_length 个字节(数组可能更大) ---
    size_t written = portHandler->writePort(txpacket.data(), total_packet_length);

    // ---- 写入字节数不对 = 发送失败 ------------------------------------------
    if (static_cast<int>(written) != total_packet_length) {
        portHandler->is_using = false;
        return COMM_TX_FAIL;
    }
    // 注意：这里不释放 is_using——因为调用方随后通常要等应答(rxPacket 里释放)。
    // 发完不等应答的路径(如广播)由各自函数负责复位。
    return COMM_SUCCESS;
}

// ============================================================================
// rxPacket(rxpacket)：接收一帧状态包(舵机应答)，校验通过后返回 COMM_SUCCESS，
// 并把完整帧字节放进 rxpacket 输出。
//   这是一段"边收边找帧头 + 边收边校验和 + 带超时"的状态机逻辑，Python 版用
//   列表切片实现，这里用 std::vector + erase 等价直译(逐行对照注释)。
// ============================================================================
int protocol_packet_handler::rxPacket(std::vector<uint8_t>& rxpacket)
{
    rxpacket.clear();                    // 清空输出缓冲
    int result = COMM_TX_FAIL;           // 默认结果(还没成功)
    int checksum = 0;
    int rx_length = 0;                   // 当前已累计收到的字节数
    int wait_length = 6;                 // 最小一帧长度 = FF FF ID LEN ERR CHK

    // Python 里那个不断 extend 的 rxpacket 列表；这里用本地缓冲等价实现
    std::vector<uint8_t> buf;

    while (true) {
        // ---- 尝试再读 (wait_length - rx_length) 个字节并追加进缓冲 ----------
        // 串口是非阻塞的，所以可能一次只读到 0~N 个；多轮循环慢慢凑齐
        int need = wait_length - rx_length;
        if (need > 0) {
            uint8_t tmp[64];
            size_t got = portHandler->readPort(tmp, static_cast<size_t>(need));
            buf.insert(buf.end(), tmp, tmp + got);   // 追加到缓冲尾
        }
        rx_length = static_cast<int>(buf.size());    // 更新累计字节数

        if (rx_length >= wait_length) {
            // ---- 已凑齐最小长度，开始"找帧头 FF FF" -------------------------
            int idx = 0;
            for (idx = 0; idx < (rx_length - 1); ++idx) {
                if (buf[idx] == 0xFF && buf[idx + 1] == 0xFF) {
                    break;   // 找到连续两个 FF
                }
            }

            if (idx == 0) {
                // ---- 帧头就在开头：这是理想情况，进一步校验头部字段 --------
                if (buf[PKT_ID] > 0xFD ||          // ID 非法(>253)
                    buf[PKT_LENGTH] > RXPACKET_MAX_LEN ||  // 长度超限
                    buf[PKT_ERROR] > 0x7F) {              // 错误字节异常(最高位应=0)
                    // 头部字段可疑：扔掉第 1 个字节再重新找帧头(可能错位了)
                    buf.erase(buf.begin());
                    rx_length -= 1;
                    continue;
                }

                // ---- 根据 LEN 重新精确计算"完整一帧应有几字节" ---------------
                if (wait_length != (buf[PKT_LENGTH] + PKT_LENGTH + 1)) {
                    wait_length = buf[PKT_LENGTH] + PKT_LENGTH + 1;  // LEN+4
                    continue;   // 目标长度变了，回去继续收剩下的
                }

                if (rx_length < wait_length) {
                    // ---- 还没收满一帧：看超时没超时 --------------------------
                    if (portHandler->isPacketTimeout()) {
                        result = (rx_length == 0) ? COMM_RX_TIMEOUT  // 一个字节都没等到
                                                  : COMM_RX_CORRUPT; // 等到一半断了
                        break;
                    }
                    continue;   // 没超时就继续等
                }

                // ---- 收满一帧，验证校验和 ------------------------------------
                checksum = 0;
                for (int i = 2; i < wait_length - 1; ++i) {   // 从 ID 累加到 CHK 前
                    checksum += buf[i];
                }
                checksum = (~checksum) & 0xFF;                 // 取反截断

                if (buf[wait_length - 1] == static_cast<uint8_t>(checksum)) {
                    result = COMM_SUCCESS;                     // 校验通过！
                } else {
                    result = COMM_RX_CORRUPT;                  // 校验失败(干扰/波特率错)
                }
                break;
            } else {
                // ---- 帧头不在开头：说明开头是脏数据，把 FF FF 之前的都丢掉 ----
                // Python: del rxpacket[0: idx]  (丢 [0,idx) 区间)
                buf.erase(buf.begin(), buf.begin() + idx);
                rx_length -= idx;
                // 循环回去重新从头判断(此时帧头已在 0 位)
            }
        } else {
            // ---- 连最小长度都没凑够：超时判断 --------------------------------
            if (portHandler->isPacketTimeout()) {
                result = (rx_length == 0) ? COMM_RX_TIMEOUT : COMM_RX_CORRUPT;
                break;
            }
            // 没超时继续收
        }
    }

    portHandler->is_using = false;   // 收完一帧，释放串口占用
    rxpacket = buf;                  // 把完整帧(或已收的残帧)交给调用方
    return result;
}

// ============================================================================
// txRxPacket：完整的"发指令 + 等应答"一问一答流程，是绝大多数 API 的公共骨架
//   1) 发送；2) 若目标=广播 则不等应答直接返回；3) 否则带超时收应答，直到
//      收到"属于我这个 ID"的状态包或超时；4) 抽出错误字节。
//   Python: txRxPacket 方法
// ============================================================================
int protocol_packet_handler::txRxPacket(std::vector<uint8_t>& txpacket,
                                        std::vector<uint8_t>& rxpacket,
                                        int& error)
{
    rxpacket.clear();
    error = 0;

    // ---- 发送指令包；失败就不必等了 ----------------------------------------
    int result = txPacket(txpacket);
    if (result != COMM_SUCCESS) {
        return result;
    }

    // ---- 广播包：舵机们都不会回，直接结束(省去白等超时) ---------------------
    if (txpacket[PKT_ID] == BROADCAST_ID) {
        portHandler->is_using = false;   // txPacket 占用了串口，这里要释放
        return result;                   // result == COMM_SUCCESS
    }

    // ---- 设定接收超时 --------------------------------------------------------
    // 若发的是 READ 指令，应答里会带 [length] 个数据字节，按需多等；
    // 其它指令应答只有固定 6 字节(FF FF ID LEN ERR CHK)。
    if (txpacket[PKT_INSTRUCTION] == INST_READ) {
        portHandler->setPacketTimeout(txpacket[PKT_PARAMETER0 + 1] + 6);
    } else {
        portHandler->setPacketTimeout(6);
    }

    // ---- 循环收应答：只认"ID 和自己发出的 ID 一致"的那一帧 ------------------
    // (总线上可能混着其它舵机的残留应答，必须过滤)
    while (true) {
        rxpacket.clear();
        result = rxPacket(rxpacket);
        // 失败就停；成功但 ID 不匹配就继续收下一帧
        if (result != COMM_SUCCESS) {
            break;
        }
        if (!rxpacket.empty() && txpacket[PKT_ID] == rxpacket[PKT_ID]) {
            break;
        }
    }

    // ---- 成功且 ID 匹配时，把状态包的错误字节取出来 --------------------------
    if (result == COMM_SUCCESS && !rxpacket.empty() &&
        txpacket[PKT_ID] == rxpacket[PKT_ID]) {
        error = rxpacket[PKT_ERROR];   // PKT_ERROR 与 PKT_INSTRUCTION 同下标=4
    }

    return result;
}

// ============================================================================
// ping：探测舵机是否在线。流程：发 PING -> 等应答 -> 若成功再读 0x03 地址的
// "型号"寄存器(2 字节)拼出型号。型号用来确认舵机型号(如 STS3215 有自己专属的型号值，
// 具体数值以实际 ping 读回的为准)。
//   地址 3 = Model Number Low, 地址 4 = Model Number High
// ============================================================================
int protocol_packet_handler::ping(int scs_id, int& model_number, int& error)
{
    model_number = 0;
    error = 0;

    if (scs_id > BROADCAST_ID) {
        return COMM_NOT_AVAILABLE;   // 非法 ID
    }

    // PING 包：ID + LEN=2(无参数) + INST_PING，共 6 字节
    std::vector<uint8_t> txpacket(6, 0);
    txpacket[PKT_ID] = scs_id;
    txpacket[PKT_LENGTH] = 2;
    txpacket[PKT_INSTRUCTION] = INST_PING;

    std::vector<uint8_t> rxpacket;
    int result = txRxPacket(txpacket, rxpacket, error);

    if (result == COMM_SUCCESS) {
        // ---- PING 通了：再读型号寄存器(0x03 起 2 字节) -----------------------
        std::vector<uint8_t> data;
        int read_result = readTxRx(scs_id, 3, 2, data, error);   // 覆盖 result/error
        result = read_result;
        if (result == COMM_SUCCESS && data.size() >= 2) {
            model_number = scs_makeword(data[0], data[1]);       // 拼成型号
        }
    }
    return result;
}

// ============================================================================
// action：触发"所有之前 REG_WRITE 暂存的舵机"一起执行。ID 传 BROADCAST(0xFE)
// 让总线上所有舵机都执行暂存动作。
// ============================================================================
int protocol_packet_handler::action(int scs_id)
{
    std::vector<uint8_t> txpacket(6, 0);
    txpacket[PKT_ID] = scs_id;
    txpacket[PKT_LENGTH] = 2;
    txpacket[PKT_INSTRUCTION] = INST_ACTION;

    std::vector<uint8_t> rxpacket;
    int error = 0;
    int result = txRxPacket(txpacket, rxpacket, error);
    return result;
}

// ============================================================================
// readTx：只把"读指令"发出去，不等应答(应答留给稍后的 readRx 收)。
// 用于拆成两半的异步读(一般用户直接读用 readTxRx 即可)。
//   指令包：ID + LEN=4 + INST_READ + [地址 address] + [长度 length]
// ============================================================================
int protocol_packet_handler::readTx(int scs_id, int address, int length)
{
    if (scs_id > BROADCAST_ID) {
        return COMM_NOT_AVAILABLE;
    }
    std::vector<uint8_t> txpacket(8, 0);
    txpacket[PKT_ID] = scs_id;
    txpacket[PKT_LENGTH] = 4;
    txpacket[PKT_INSTRUCTION] = INST_READ;
    txpacket[PKT_PARAMETER0 + 0] = static_cast<uint8_t>(address);
    txpacket[PKT_PARAMETER0 + 1] = static_cast<uint8_t>(length);

    int result = txPacket(txpacket);   // 发完不等

    // 若发送成功，按"将收到 length 个数据字节"预设定时
    if (result == COMM_SUCCESS) {
        portHandler->setPacketTimeout(length + 6);
    }
    return result;
}

// ============================================================================
// readRx：接收 readTx 发出后的应答(和 readTx 配对使用)
//   收一帧，校验 ID 匹配后，把数据区 [参数起点 .. 起点+length) 拷给调用方
// ============================================================================
int protocol_packet_handler::readRx(int scs_id, int length,
                                    std::vector<uint8_t>& data, int& error)
{
    int result = COMM_TX_FAIL;
    error = 0;
    data.clear();

    std::vector<uint8_t> rxpacket;
    while (true) {
        rxpacket.clear();
        result = rxPacket(rxpacket);
        // 失败停；成功但 ID 不匹配继续收
        if (result != COMM_SUCCESS) {
            break;
        }
        if (!rxpacket.empty() && rxpacket[PKT_ID] == scs_id) {
            break;
        }
    }

    // 成功：取错误字节 + 把数据区拷贝出来
    if (result == COMM_SUCCESS && rxpacket.size() >= PKT_PARAMETER0 + length) {
        error = rxpacket[PKT_ERROR];
        data.assign(rxpacket.begin() + PKT_PARAMETER0,
                    rxpacket.begin() + PKT_PARAMETER0 + length);
    }
    return result;
}

// ============================================================================
// readTxRx：最常用的"读"入口 = 发读指令 + 收应答 + 抽数据，一步到位
// ============================================================================
int protocol_packet_handler::readTxRx(int scs_id, int address, int length,
                                      std::vector<uint8_t>& data, int& error)
{
    data.clear();
    error = 0;
    if (scs_id > BROADCAST_ID) {
        return COMM_NOT_AVAILABLE;
    }

    std::vector<uint8_t> txpacket(8, 0);
    txpacket[PKT_ID] = scs_id;
    txpacket[PKT_LENGTH] = 4;
    txpacket[PKT_INSTRUCTION] = INST_READ;
    txpacket[PKT_PARAMETER0 + 0] = static_cast<uint8_t>(address);
    txpacket[PKT_PARAMETER0 + 1] = static_cast<uint8_t>(length);

    std::vector<uint8_t> rxpacket;
    int result = txRxPacket(txpacket, rxpacket, error);

    if (result == COMM_SUCCESS && rxpacket.size() >= PKT_PARAMETER0 + length) {
        error = rxpacket[PKT_ERROR];
        data.assign(rxpacket.begin() + PKT_PARAMETER0,
                    rxpacket.begin() + PKT_PARAMETER0 + length);
    }
    return result;
}

// ---- 读 1 字节便捷封装：把 data 向量里的唯一字节变成整数 -------------------
int protocol_packet_handler::read1ByteTxRx(int scs_id, int address,
                                           uint8_t& data_read, int& error)
{
    std::vector<uint8_t> data;
    int result = readTxRx(scs_id, address, 1, data, error);
    data_read = (result == COMM_SUCCESS && !data.empty()) ? data[0] : 0;
    return result;
}

// ---- 读 2 字节：拼成 16 位整数(字节序感知) ---------------------------------
int protocol_packet_handler::read2ByteTxRx(int scs_id, int address,
                                           int& data_read, int& error)
{
    std::vector<uint8_t> data;
    int result = readTxRx(scs_id, address, 2, data, error);
    data_read = (result == COMM_SUCCESS && data.size() >= 2)
                    ? scs_makeword(data[0], data[1])
                    : 0;
    return result;
}

// ---- 读 4 字节：拼成 32 位整数(两个 16 位字拼) ------------------------------
int protocol_packet_handler::read4ByteTxRx(int scs_id, int address,
                                           int& data_read, int& error)
{
    std::vector<uint8_t> data;
    int result = readTxRx(scs_id, address, 4, data, error);
    if (result == COMM_SUCCESS && data.size() >= 4) {
        data_read = scs_makedword(scs_makeword(data[0], data[1]),
                                  scs_makeword(data[2], data[3]));
    } else {
        data_read = 0;
    }
    return result;
}

// ============================================================================
// writeTxOnly：只发"写"指令不等应答(最常用的高速写，广播也用它)
//   指令包结构：
//     ID + LEN=(length+3) + INST_WRITE + [起始地址] + [数据...length 字节]
// ============================================================================
int protocol_packet_handler::writeTxOnly(int scs_id, int address, int length,
                                         const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> txpacket(length + 7, 0);   // 预留整帧空间
    txpacket[PKT_ID] = static_cast<uint8_t>(scs_id);
    txpacket[PKT_LENGTH] = length + 3;              // LEN=INST(1)+地址(1)+数据
    txpacket[PKT_INSTRUCTION] = INST_WRITE;
    txpacket[PKT_PARAMETER0] = static_cast<uint8_t>(address);   // 起始地址

    // 拷贝 length 个数据字节到参数区(从下标 PKT_PARAMETER0+1 开始)
    for (int i = 0; i < length; ++i) {
        txpacket[PKT_PARAMETER0 + 1 + i] = data[i];
    }

    int result = txPacket(txpacket);
    portHandler->is_using = false;   // TxOnly：发完即释放(不等应答)
    return result;
}

// ============================================================================
// writeTxRx：发"写"指令 + 等舵机回执(能拿到舵机侧错误码，适合慢速/关键配置)
// ============================================================================
int protocol_packet_handler::writeTxRx(int scs_id, int address, int length,
                                       const std::vector<uint8_t>& data, int& error)
{
    std::vector<uint8_t> txpacket(length + 7, 0);
    txpacket[PKT_ID] = static_cast<uint8_t>(scs_id);
    txpacket[PKT_LENGTH] = length + 3;
    txpacket[PKT_INSTRUCTION] = INST_WRITE;
    txpacket[PKT_PARAMETER0] = static_cast<uint8_t>(address);
    for (int i = 0; i < length; ++i) {
        txpacket[PKT_PARAMETER0 + 1 + i] = data[i];
    }

    std::vector<uint8_t> rxpacket;
    int result = txRxPacket(txpacket, rxpacket, error);
    return result;
}

// ---- 写 1 字节(不等应答) ----------------------------------------------------
int protocol_packet_handler::write1ByteTxOnly(int scs_id, int address, uint8_t data)
{
    std::vector<uint8_t> d = { data };
    return writeTxOnly(scs_id, address, 1, d);
}

// ---- 写 1 字节(等应答) ------------------------------------------------------
int protocol_packet_handler::write1ByteTxRx(int scs_id, int address, uint8_t data, int& error)
{
    std::vector<uint8_t> d = { data };
    return writeTxRx(scs_id, address, 1, d, error);
}

// ---- 写 2 字节：把 16 位整数拆成 低/高字节(按字节序) 再写 ------------------
int protocol_packet_handler::write2ByteTxOnly(int scs_id, int address, int data)
{
    std::vector<uint8_t> d = {
        static_cast<uint8_t>(scs_lobyte(data)),
        static_cast<uint8_t>(scs_hibyte(data))
    };
    return writeTxOnly(scs_id, address, 2, d);
}

int protocol_packet_handler::write2ByteTxRx(int scs_id, int address, int data, int& error)
{
    std::vector<uint8_t> d = {
        static_cast<uint8_t>(scs_lobyte(data)),
        static_cast<uint8_t>(scs_hibyte(data))
    };
    return writeTxRx(scs_id, address, 2, d, error);
}

// ---- 写 4 字节：32 位整数拆成 4 个字节(低字低/高字节、高字低/高字节) -------
int protocol_packet_handler::write4ByteTxOnly(int scs_id, int address, int data)
{
    std::vector<uint8_t> d = {
        static_cast<uint8_t>(scs_lobyte(scs_loword(data))),
        static_cast<uint8_t>(scs_hibyte(scs_loword(data))),
        static_cast<uint8_t>(scs_lobyte(scs_hiword(data))),
        static_cast<uint8_t>(scs_hibyte(scs_hiword(data)))
    };
    return writeTxOnly(scs_id, address, 4, d);
}

int protocol_packet_handler::write4ByteTxRx(int scs_id, int address, int data, int& error)
{
    std::vector<uint8_t> d = {
        static_cast<uint8_t>(scs_lobyte(scs_loword(data))),
        static_cast<uint8_t>(scs_hibyte(scs_loword(data))),
        static_cast<uint8_t>(scs_lobyte(scs_hiword(data))),
        static_cast<uint8_t>(scs_hibyte(scs_hiword(data)))
    };
    return writeTxRx(scs_id, address, 4, d, error);
}

// ============================================================================
// regWriteTxOnly / regWriteTxRx：异步写(REG_WRITE)
//   与普通写的差别：舵机收到后只"暂存"到内部寄存器，不动；
//   直到收到 ACTION(0x05) 才真正执行。常用于需要"多个舵机同一时刻一起动"的场合。
// ============================================================================
int protocol_packet_handler::regWriteTxOnly(int scs_id, int address, int length,
                                            const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> txpacket(length + 7, 0);
    txpacket[PKT_ID] = static_cast<uint8_t>(scs_id);
    txpacket[PKT_LENGTH] = length + 3;
    txpacket[PKT_INSTRUCTION] = INST_REG_WRITE;
    txpacket[PKT_PARAMETER0] = static_cast<uint8_t>(address);
    for (int i = 0; i < length; ++i) {
        txpacket[PKT_PARAMETER0 + 1 + i] = data[i];
    }
    int result = txPacket(txpacket);
    portHandler->is_using = false;
    return result;
}

int protocol_packet_handler::regWriteTxRx(int scs_id, int address, int length,
                                          const std::vector<uint8_t>& data, int& error)
{
    std::vector<uint8_t> txpacket(length + 7, 0);
    txpacket[PKT_ID] = static_cast<uint8_t>(scs_id);
    txpacket[PKT_LENGTH] = length + 3;
    txpacket[PKT_INSTRUCTION] = INST_REG_WRITE;
    txpacket[PKT_PARAMETER0] = static_cast<uint8_t>(address);
    for (int i = 0; i < length; ++i) {
        txpacket[PKT_PARAMETER0 + 1 + i] = data[i];
    }
    std::vector<uint8_t> rxpacket;
    int result = txRxPacket(txpacket, rxpacket, error);
    return result;
}

// ============================================================================
// syncReadTx：同步读——只发"让多个舵机依次上报"的广播指令，之后用 syncReadRx 收
//   参数区 = [起始地址] [数据长度] [ID1] [ID2] ... [IDn]
//   舵机们收到后按 ID 从小到大依次把自己的数据回上来(这就是"同步"的含义)
// ============================================================================
int protocol_packet_handler::syncReadTx(int start_address, int data_length,
                                        const std::vector<uint8_t>& param,
                                        int param_length)
{
    std::vector<uint8_t> txpacket(param_length + 8, 0);
    // 固定开销 8 = FF FF ID LEN INST START_ADDR DATA_LEN CHK
    txpacket[PKT_ID] = BROADCAST_ID;             // 发给所有人
    txpacket[PKT_LENGTH] = param_length + 4;     // 4 = INST+START+DATA_LEN+... 
    txpacket[PKT_INSTRUCTION] = INST_SYNC_READ;
    txpacket[PKT_PARAMETER0 + 0] = static_cast<uint8_t>(start_address);
    txpacket[PKT_PARAMETER0 + 1] = static_cast<uint8_t>(data_length);
    for (int i = 0; i < param_length; ++i) {
        txpacket[PKT_PARAMETER0 + 2 + i] = param[i];   // 依次是各舵机 ID
    }
    int result = txPacket(txpacket);   // 只发(不等)
    return result;
}

// ============================================================================
// syncReadRx：收同步读的一长串应答。
//   每个舵机会回 (6 + data_length) 字节(FF FF ID LEN ERR DATA.. CHK)，
//   一共 param_length 个舵机 => 总长 = (6+data_length) * param_length。
//   这里按"总长"一次性等满(不等满就超时重来)，原始字节原样交给上层分组。
// ============================================================================
int protocol_packet_handler::syncReadRx(int data_length, int param_length,
                                        std::vector<uint8_t>& rxpacket)
{
    int wait_length = (6 + data_length) * param_length;  // 预期总字节数
    portHandler->setPacketTimeoutMillis(wait_length);    // 超时跟总长挂钩
    rxpacket.clear();
    int rx_length = 0;

    while (true) {
        // 一次最多读 (wait_length - rx_length) 字节追加进去
        uint8_t tmp[64];
        size_t got = portHandler->readPort(tmp, static_cast<size_t>(wait_length - rx_length));
        rxpacket.insert(rxpacket.end(), tmp, tmp + got);
        rx_length = static_cast<int>(rxpacket.size());

        if (rx_length >= wait_length) {
            portHandler->is_using = false;   // 收齐：释放串口
            return COMM_SUCCESS;
        }
        if (portHandler->isPacketTimeout()) {
            portHandler->is_using = false;
            return (rx_length == 0) ? COMM_RX_TIMEOUT : COMM_RX_CORRUPT;
        }
    }
}

// ============================================================================
// syncWriteTxOnly：同步写——一条广播指令给多个舵机写"同一段寄存器、不同数据"
//   参数区 = [起始地址] [数据长度] [ID1][数据1...] [ID2][数据2...] ...
//   数据长度固定，所以舵机能从参数流里"各取所需"。
//   注意：给的是广播 ID，舵机不回，txRxPacket 内部会自动跳过等待。
// ============================================================================
int protocol_packet_handler::syncWriteTxOnly(int start_address, int data_length,
                                             const std::vector<uint8_t>& param,
                                             int param_length)
{
    std::vector<uint8_t> txpacket(param_length + 8, 0);
    txpacket[PKT_ID] = BROADCAST_ID;
    txpacket[PKT_LENGTH] = param_length + 4;
    txpacket[PKT_INSTRUCTION] = INST_SYNC_WRITE;
    txpacket[PKT_PARAMETER0 + 0] = static_cast<uint8_t>(start_address);
    txpacket[PKT_PARAMETER0 + 1] = static_cast<uint8_t>(data_length);
    for (int i = 0; i < param_length; ++i) {
        txpacket[PKT_PARAMETER0 + 2 + i] = param[i];
    }

    std::vector<uint8_t> rxpacket;
    int error = 0;
    int result = txRxPacket(txpacket, rxpacket, error);   // 广播：发完即返
    return result;
}

// ============================================================================
// reOfsCal：零点标定(INST_OFSCAL)。把 position 写入，舵机会把"当前物理位置"
// 标定成你给的这个数值。通常用于装舵机时把机械中位标成 0/2048 等。
// ============================================================================
int protocol_packet_handler::reOfsCal(int scs_id, int position, int& error)
{
    error = 0;
    if (scs_id > BROADCAST_ID) {
        return COMM_NOT_AVAILABLE;
    }
    std::vector<uint8_t> txpacket(8, 0);
    txpacket[PKT_ID] = static_cast<uint8_t>(scs_id);
    txpacket[PKT_LENGTH] = 4;
    txpacket[PKT_INSTRUCTION] = INST_OFSCAL;
    txpacket[PKT_PARAMETER0] = static_cast<uint8_t>(scs_lobyte(position));
    txpacket[PKT_PARAMETER0 + 1] = static_cast<uint8_t>(scs_hibyte(position));

    std::vector<uint8_t> rxpacket;
    int result = txRxPacket(txpacket, rxpacket, error);
    return result;
}

// ============================================================================
// reSet：恢复出厂设置(INST_RESET)。会把 ID、波特率、角度范围等全复位。
// 注意：复位后舵机 ID 会变回 1、波特率回默认，使用时要格外小心！
// ============================================================================
int protocol_packet_handler::reSet(int scs_id, int& error)
{
    error = 0;
    if (scs_id > BROADCAST_ID) {
        return COMM_NOT_AVAILABLE;
    }
    std::vector<uint8_t> txpacket(6, 0);
    txpacket[PKT_ID] = static_cast<uint8_t>(scs_id);
    txpacket[PKT_LENGTH] = 2;
    txpacket[PKT_INSTRUCTION] = INST_RESET;

    std::vector<uint8_t> rxpacket;
    int result = txRxPacket(txpacket, rxpacket, error);
    return result;
}

}  // namespace scs
