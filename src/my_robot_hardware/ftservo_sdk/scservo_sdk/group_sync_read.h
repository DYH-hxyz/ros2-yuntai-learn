// ============================================================================
// group_sync_read.h —— 同步读"参数收集器"(声明)
// ----------------------------------------------------------------------------
// 对应 Python group_sync_read.py。与 GroupSyncWrite 相反：它负责
//   1) 记下"要向哪些舵机(ID)同步读取同一段寄存器"
//   2) 发一条 SYNC_READ 广播指令，让这些舵机按 ID 从小到大依次把自己的数据回上来
//   3) 收到一长串混合应答后，按帧头逐台拆包，把每台的数据暂存在 data_dict 里
//   4) 提供 getData()/isAvailable() 让上层按 ID 查询结果
// ============================================================================
#ifndef GROUP_SYNC_READ_H
#define GROUP_SYNC_READ_H

#include <cstdint>
#include <map>
#include <vector>

#include "protocol_packet_handler.h"

namespace scs {

class GroupSyncRead {
public:
    // 构造函数：绑定协议处理器 + 起始寄存器地址 + 每台舵机读几个字节
    GroupSyncRead(protocol_packet_handler* ph, int start_address, int data_length);

    bool addParam(int scs_id);        // 把某舵机 ID 加入本轮同步读名单
    void removeParam(int scs_id);     // 移除某舵机
    void clearParam();                // 清空名单(每轮用完清)

    int txPacket();                   // 只发同步读指令(返回 COMM_*)
    int rxPacket();                   // 只收并解析整串应答
    int txRxPacket();                 // 发+收一步到位

    // 查询某舵机某地址的数据是否已就绪(就绪返回 true, 并把错误字节带出)
    bool isAvailable(int scs_id, int address, int data_length, uint8_t& error);
    int getData(int scs_id, int address, int data_length);   // 取某舵机数据(整数)

private:
    void makeParam();   // 重建"要读的 ID 列表"参数
    // 从混合应答流里按 ID 拆出某个舵机的一帧(核心解析，见实现注释)
    int readRx(const std::vector<uint8_t>& rxpacket, int scs_id, int data_length);

    protocol_packet_handler* ph;      // 上层协议处理器
    int start_address;                // 起始寄存器地址
    int data_length;                  // 每个舵机读的字节数

    bool last_result = false;         // 上一轮是否全部成功(对应 Python self.last_result)
    bool is_param_changed = false;    // 名单是否变过
    std::vector<uint8_t> param;       // 扁平 ID 列表参数
    // 存结果：ID -> [Error字节, 数据0, 数据1, ...] (Python 里 data_dict[id] 结构)
    std::map<int, std::vector<uint8_t>> data_dict;
};

}  // namespace scs

#endif  // GROUP_SYNC_READ_H
