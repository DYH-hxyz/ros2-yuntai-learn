// ============================================================================
// group_sync_write.cpp —— 同步写"参数分组器"(实现)
//   逐函数对照 Python group_sync_write.py
// ============================================================================

#include "scservo_def.h"
#include "group_sync_write.h"

namespace scs {

// 构造函数：等价 Python __init__ (把 ph/起始地址/数据长度记下来)
GroupSyncWrite::GroupSyncWrite(protocol_packet_handler* ph_ptr,
                               int start_addr, int data_len)
    : ph(ph_ptr),
      start_address(start_addr),
      data_length(data_len)
{
    // is_param_changed/param/data_dict 已由头文件里的默认值初始化(=空)
}

// ============================================================================
// addParam：往"本轮待同步写"的集合里添加一个舵机的数据
//   返回 false 的情形：ID 已在集合里 / 数据长度超过设定的 data_length
// ============================================================================
bool GroupSyncWrite::addParam(int scs_id, const std::vector<uint8_t>& data)
{
    if (data_dict.count(scs_id) != 0) {
        return false;            // 重复添加同一 ID -> 拒绝
    }
    if (static_cast<int>(data.size()) > data_length) {
        return false;            // 数据超长 -> 拒绝
    }
    data_dict[scs_id] = data;    // 存进集合(map 自动按 ID 升序，顺序正好符合总线习惯)
    is_param_changed = true;     // 标记"参数变过"，让下次 txPacket 重建 param
    return true;
}

// 移除某个舵机
void GroupSyncWrite::removeParam(int scs_id)
{
    if (data_dict.count(scs_id) == 0) {
        return;                  // 本来就不在 -> 无事可做
    }
    data_dict.erase(scs_id);
    is_param_changed = true;
}

// 修改某个已存在舵机的数据(不存在或超长返回 false)
bool GroupSyncWrite::changeParam(int scs_id, const std::vector<uint8_t>& data)
{
    if (data_dict.count(scs_id) == 0) {
        return false;
    }
    if (static_cast<int>(data.size()) > data_length) {
        return false;
    }
    data_dict[scs_id] = data;
    is_param_changed = true;
    return true;
}

// 清空整批(每轮发完必须清，否则下一轮会把旧 ID 也带上)
void GroupSyncWrite::clearParam()
{
    data_dict.clear();
}

// ============================================================================
// makeParam：把 data_dict 展平成协议层要的"扁平参数流"
//   Python: self.param = [] ; for id: self.param += [id] + data[id]
//   生成顺序 = map 升序遍历(与 Python 按添加顺序基本一致，且升序是总线推荐)
// ============================================================================
void GroupSyncWrite::makeParam()
{
    if (data_dict.empty()) {
        return;
    }
    param.clear();
    for (auto& kv : data_dict) {
        if (kv.second.empty()) {
            return;   // 有舵机数据为空：属异常，放弃本次重建(保持原样/空)
        }
        param.push_back(static_cast<uint8_t>(kv.first));           // 先放 ID
        param.insert(param.end(), kv.second.begin(), kv.second.end());  // 再放该 ID 的数据
    }
}

// ============================================================================
// txPacket：真正把整批发出去
//   没有数据 -> COMM_NOT_AVAILABLE
//   数据变了(或 param 还是空的) -> 先重建 param
//   然后交给协议层的 syncWriteTxOnly：
//     param_length = 舵机数 x (1 + 每舵机字节数)   (1 是每个 ID 占的 1 字节)
// ============================================================================
int GroupSyncWrite::txPacket()
{
    if (data_dict.empty()) {
        return COMM_NOT_AVAILABLE;
    }

    if (is_param_changed || param.empty()) {
        makeParam();
        is_param_changed = false;   // 重建完成，清标记(避免重复重建)
    }

    int param_length = static_cast<int>(data_dict.size()) * (1 + data_length);
    return ph->syncWriteTxOnly(start_address, data_length, param, param_length);
}

}  // namespace scs
