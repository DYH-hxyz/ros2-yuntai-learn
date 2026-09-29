// ============================================================================
// group_sync_read.cpp —— 同步读"参数收集器"(实现)
//   逐函数对照 Python group_sync_read.py。这里 readRx 是全场最绕的一段，
//   专门把"多个舵机拼在一起回的一长串字节"逐帧拆开，请务必看注释。
// ============================================================================

#include "scservo_def.h"
#include "group_sync_read.h"

namespace scs {

// 构造函数：对应 Python __init__
GroupSyncRead::GroupSyncRead(protocol_packet_handler* ph_ptr,
                             int start_addr, int data_len)
    : ph(ph_ptr),
      start_address(start_addr),
      data_length(data_len)
{
    // Python __init__ 末尾调用了 self.clearParam()，这里 data_dict 默认即空
}

// 把某舵机加入本轮名单(已在名单里则返回 false)
bool GroupSyncRead::addParam(int scs_id)
{
    if (data_dict.count(scs_id) != 0) {
        return false;            // 重复 -> 拒绝
    }
    // Python: self.data_dict[scs_id] = []  占个位，等收到数据后填入
    data_dict[scs_id] = std::vector<uint8_t>();
    is_param_changed = true;     // 名单变了
    return true;
}

// 移除某舵机
void GroupSyncRead::removeParam(int scs_id)
{
    if (data_dict.count(scs_id) == 0) {
        return;
    }
    data_dict.erase(scs_id);
    is_param_changed = true;
}

// 清空名单
void GroupSyncRead::clearParam()
{
    data_dict.clear();
}

// ============================================================================
// makeParam：把"要读的舵机 ID 列表"展平成协议层要的参数流 [ID1][ID2]...
// ============================================================================
void GroupSyncRead::makeParam()
{
    if (data_dict.empty()) {
        return;
    }
    param.clear();
    for (auto& kv : data_dict) {
        param.push_back(static_cast<uint8_t>(kv.first));   // 每个 ID 占 1 字节
    }
}

// ============================================================================
// txPacket：发同步读指令
// ============================================================================
int GroupSyncRead::txPacket()
{
    if (data_dict.empty()) {
        return COMM_NOT_AVAILABLE;
    }
    if (is_param_changed || param.empty()) {
        makeParam();
        is_param_changed = false;
    }
    // param_length = 舵机个数(每个只占 1 字节 ID)
    int param_length = static_cast<int>(data_dict.size());
    return ph->syncReadTx(start_address, data_length, param, param_length);
}

// ============================================================================
// rxPacket：收一整串同步读应答并逐台拆包
//   先把底层 syncReadRx 返回的原始字节流拿到，然后对名单里的每个 ID
//   调用 readRx 从流里"各找各的帧"存进 data_dict。
//   Python 里 data_dict[scs_id] 在拆包成功后被替换成 [Error, 数据...]
// ============================================================================
int GroupSyncRead::rxPacket()
{
    last_result = true;              // 先乐观假设全成
    if (data_dict.empty()) {
        return COMM_NOT_AVAILABLE;
    }

    std::vector<uint8_t> rxpacket;
    int result = ph->syncReadRx(data_length, static_cast<int>(data_dict.size()), rxpacket);

    // 至少要有 (data_length+6) 字节(最小一个完整舵机帧)才值得拆
    if (static_cast<int>(rxpacket.size()) >= (data_length + 6)) {
        for (auto& kv : data_dict) {
            int scs_id = kv.first;
            int r = readRx(rxpacket, scs_id, data_length);
            if (r != COMM_SUCCESS) {
                last_result = false;   // 有一台拆包失败 -> 整体标记失败
            } else {
                result = r;
            }
        }
    } else {
        last_result = false;
    }
    return result;
}

// 发 + 收 一步到位
int GroupSyncRead::txRxPacket()
{
    int result = txPacket();
    if (result != COMM_SUCCESS) {
        return result;
    }
    return rxPacket();
}

// ============================================================================
// readRx：核心拆包函数。在"多个舵机的应答连在一起的长流"里找指定 ID 的那一帧。
//   SCS 应答帧格式：FF FF [ID] [LEN] [ERR] [数据...n] [CHK]
//   同步读时舵机们按 ID 升序一个接一个回，所以流里是 N 段这样的帧连在一起。
//   本函数用"滚动窗口"逐字节找 FF FF ID，找到后校验 LEN 和 CHK，成功就把
//   [ERR, 数据...] 存进 data_dict[scs_id]。
// ============================================================================
int GroupSyncRead::readRx(const std::vector<uint8_t>& rxpacket,
                          int scs_id, int data_len)
{
    int rx_length = static_cast<int>(rxpacket.size());
    int rx_index = 0;

    // 主循环：只要还剩"至少一帧"的余地就继续找
    while ((rx_index + 6 + data_len) <= rx_length) {
        // ---- 滚动找帧头 FF FF <id> ----
        // 用 3 字节滑动窗口(h0最新 / h1上一个 / h2上上个)逐字节后移，
        // 命中"h2==FF 且 h1==FF 且 h0==目标ID"即找到本舵机的帧起点
        uint8_t h0 = 0, h1 = 0, h2 = 0;
        while (rx_index < rx_length) {
            h2 = h1;                         // 窗口整体后移一格
            h1 = h0;
            h0 = rxpacket[rx_index];         // 读入最新字节
            rx_index++;
            if (h2 == 0xFF && h1 == 0xFF && h0 == static_cast<uint8_t>(scs_id)) {
                break;                       // 找到帧头(此时 rx_index 已越过 ID 字节)
            }
        }

        // ---- 帧头后还需 LEN(1)+ERR(1)+数据(n)+CHK(1) = n+3 字节 ----
        // 若剩余不足(含"整个流都没找到该 ID"导致 rx_index 走到头的情形)则放弃
        if ((rx_index + 3 + data_len) > rx_length) {
            break;
        }

        // ---- 校验 LEN 是否等于 data_len+2 (ERR+数据+CHK) --------------
        if (rxpacket[rx_index] != static_cast<uint8_t>(data_len + 2)) {
            rx_index += 1;    // LEN 不对：往后挪 1 字节再找(可能错位)
            continue;
        }
        rx_index += 1;        // 跳过 LEN

        // ---- 读错误字节 + 数据，同时累加校验和 --------------------------
        uint8_t Error = rxpacket[rx_index];
        rx_index += 1;

        int calSum = scs_id + (data_len + 2) + Error;   // 校验从 ID 开始累加
        std::vector<uint8_t> data;
        data.push_back(Error);                          // data[0] 存错误字节
        for (int i = 0; i < data_len; ++i) {
            calSum += rxpacket[rx_index];               // 累加每个数据字节
            data.push_back(rxpacket[rx_index]);
            rx_index += 1;
        }
        calSum = (~calSum) & 0xFF;                      // 取反截断 = 期望 CHK

        // ---- 校验和与最后一个字节比对 ------------------------------------
        if (static_cast<uint8_t>(calSum) != rxpacket[rx_index]) {
            return COMM_RX_CORRUPT;   // 校验失败：这帧脏了
        }

        data_dict[scs_id] = data;     // 存好: [Error, d0, d1, ...]
        return COMM_SUCCESS;
    }

    // 正常遍历完也没拆出有效帧
    return COMM_RX_CORRUPT;
}

// ============================================================================
// isAvailable：查某舵机、某地址的数据是否已经就绪可用。
//   会校验：ID 是否在名单里、地址是否落在本次读取的区间内、数据是否真的到了。
//   Python 返回 (True/False, data[0]=错误字节)；C++ 返回 bool，错误字节走引用
// ============================================================================
bool GroupSyncRead::isAvailable(int scs_id, int address, int data_len,
                                uint8_t& error)
{
    auto it = data_dict.find(scs_id);
    if (it == data_dict.end()) {
        return false;                // 该 ID 压根没参与本轮
    }
    // 地址必须落在 [start_address, start_address + data_length - data_len]
    if (address < start_address ||
        (start_address + data_length - data_len) < address) {
        return false;                // 越界
    }
    std::vector<uint8_t>& v = it->second;
    if (v.empty()) {
        return false;                // 还没收到数据
    }
    if (static_cast<int>(v.size()) < (data_len + 1)) {
        return false;                // 收到的数据不够
    }
    error = v[0];                    // 错误字节在 data 的[0]
    return true;
}

// ============================================================================
// getData：按地址取某舵机的数据(支持 1/2/4 字节宽度的整数拼装)
//   data_dict[id] 布局 = [错误, d0, d1, ...]，所以数据起点 = 地址差 + 1
// ============================================================================
int GroupSyncRead::getData(int scs_id, int address, int data_len)
{
    auto it = data_dict.find(scs_id);
    if (it == data_dict.end()) {
        return 0;
    }
    std::vector<uint8_t>& v = it->second;
    int off = address - start_address + 1;   // 数据区起点(跳过[0]的错误字节)

    if (data_len == 1) {
        return v[off];
    }
    if (data_len == 2) {
        return ph->scs_makeword(v[off], v[off + 1]);       // 拼 16 位
    }
    if (data_len == 4) {
        return ph->scs_makedword(                            // 拼 32 位
            ph->scs_makeword(v[off], v[off + 1]),
            ph->scs_makeword(v[off + 2], v[off + 3]));
    }
    return 0;                            // 只支持 1/2/4 字节
}

}  // namespace scs
