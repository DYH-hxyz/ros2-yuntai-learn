// ============================================================================
// group_sync_write.h —— 同步写"参数分组器"(声明)
// ----------------------------------------------------------------------------
// 对应 Python group_sync_write.py。功能一句话：帮你在内存里攒一批
//   [舵机ID + 各自要写的数据]
// 然后一次性打包成一条 SYNC_WRITE 广播指令发给总线上所有舵机。
//
// 为什么要这么绕？因为 SCS 总线是"半双工串行"，一条指令只能发一次。
// 如果你要 10 个舵机同时动，逐台写要发 10 条指令、而且不同步；
// SYNC_WRITE 让 10 台舵机在【同一帧里】拿到各自目标值，动作才整齐划一。
// ============================================================================
#ifndef GROUP_SYNC_WRITE_H
#define GROUP_SYNC_WRITE_H

#include <cstdint>
#include <map>
#include <vector>

#include "protocol_packet_handler.h"

namespace scs {

class GroupSyncWrite {
public:
    // 构造函数：绑定协议处理器 + 记下"要写的起始寄存器地址"和"每个舵机写几个字节"
    //   Python: GroupSyncWrite(self, SMS_STS_ACC, 7)  -> 从 ACC 地址起每舵机 7 字节
    GroupSyncWrite(protocol_packet_handler* ph, int start_address, int data_length);

    // 把某个舵机要写的数据加进本轮批次(重复 ID 或数据超长返回 false)
    bool addParam(int scs_id, const std::vector<uint8_t>& data);

    void removeParam(int scs_id);   // 移除某舵机
    bool changeParam(int scs_id, const std::vector<uint8_t>& data);  // 改某舵机数据

    void clearParam();              // 清空整批(每轮发完都要清，见示例)

    int txPacket();                 // 把攒好的整批发出去(返回 COMM_*)

private:
    void makeParam();   // 由 is_param_changed 触发，重建协议层要的扁平参数流

    protocol_packet_handler* ph;    // 上层协议处理器(对应 Python self.ph)
    int start_address;              // 起始寄存器地址
    int data_length;                // 每个舵机数据字节数(必须固定，协议要求)
    bool is_param_changed = false;  // 数据是否变过(变过才需重建 param)
    std::vector<uint8_t> param;     // 扁平参数流: [ID1][d..][ID2][d..]...
    std::map<int, std::vector<uint8_t>> data_dict;  // 攒数据的容器: ID -> 数据
};

}  // namespace scs

#endif  // GROUP_SYNC_WRITE_H
