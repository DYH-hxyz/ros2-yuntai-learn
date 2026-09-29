// ============================================================================
// sms_sts.h —— SMS/STS 系列舵机高层 API(声明)
// ----------------------------------------------------------------------------
// 对应 Python sms_sts.py。到这一层你就不用再管"协议帧怎么组"了——
// 协议细节在 protocol_packet_handler，这里只做两件事：
//   1) 定义【内存表】(寄存器地址都起了名字，这就是你想要的"封装库"，避开裸地址)
//   2) 提供人话函数：WritePosEx(带加速度/速度地走到某位置)、ReadPos 等
//
// 【STS 舵机内存表是什么？】
//   舵机内部就像一块小内存，不同"地址"存不同含义的数据(位置/速度/温度...)。
//   你要让舵机动，本质就是"往某个地址写值"；要读状态，就是"从某地址读值"。
//   下表地址是官方数据手册给的，SDK 起好名字后你完全不用背数字。
//   EPROM = 掉电不丢(配置型数据，如 ID/波特率/角度范围)
//   SRAM  = 掉电就丢(运行型数据，如当前位置/目标位置)
//   位置单位：0~4095 对应一整圈(4096 步)；速度/加速度单位见各注释
// ============================================================================
#ifndef SMS_STS_H
#define SMS_STS_H

#include <vector>

#include "scservo_def.h"
#include "protocol_packet_handler.h"
#include "group_sync_write.h"

namespace scs {

// ---- 波特率表(写进地址 6 的"档位值"，不是真正的波特率数字) ----------------
constexpr int SMS_STS_1M      = 0;   // 1Mbps
constexpr int SMS_STS_0_5M    = 1;   // 0.5Mbps
constexpr int SMS_STS_250K    = 2;   // 250kbps
constexpr int SMS_STS_128K    = 3;   // 128kbps
constexpr int SMS_STS_115200  = 4;   // 115200
constexpr int SMS_STS_76800   = 5;   // 76800
constexpr int SMS_STS_57600   = 6;   // 57600
constexpr int SMS_STS_38400   = 7;   // 38400

// ============================================================================
// 内存表地址常量(全部来自 sms_sts.py，一 一对应)
// ============================================================================
// ---------- EPROM 只读 ----------
constexpr int SMS_STS_MODEL_L = 3;     // 型号低字节(读型号用，如 STS3215)
constexpr int SMS_STS_MODEL_H = 4;     // 型号高字节

// ---------- EPROM 读写(配置类) ----------
constexpr int SMS_STS_ID              = 5;   // 舵机 ID(改它=给舵机换号)
constexpr int SMS_STS_BAUD_RATE       = 6;   // 波特率档位
constexpr int SMS_STS_MIN_ANGLE_LIMIT_L = 9; // 最小角度限位低字节(限制活动范围)
constexpr int SMS_STS_MIN_ANGLE_LIMIT_H = 10;
constexpr int SMS_STS_MAX_ANGLE_LIMIT_L = 11; // 最大角度限位低字节
constexpr int SMS_STS_MAX_ANGLE_LIMIT_H = 12;
constexpr int SMS_STS_CW_DEAD         = 26;  // 顺时针死区(消除齿轮间隙抖动)
constexpr int SMS_STS_CCW_DEAD        = 27;  // 逆时针死区
constexpr int SMS_STS_OFS_L           = 31;  // 零点偏移低字节(标定中位用)
constexpr int SMS_STS_OFS_H           = 32;
constexpr int SMS_STS_MODE            = 33;  // 运行模式: 0=角度模式 1=轮子模式(恒速)

// ---------- SRAM 读写(运行类) ----------
constexpr int SMS_STS_TORQUE_ENABLE      = 40; // 舵机使能(0=松/掉电, 1=锁住/出力)
constexpr int SMS_STS_ACC                = 41; // 加速度(8.7°/s² 每单位)
constexpr int SMS_STS_GOAL_POSITION_L    = 42; // 目标位置低字节
constexpr int SMS_STS_GOAL_POSITION_H    = 43;
constexpr int SMS_STS_GOAL_TIME_L        = 44; // 运动时间(0=按速度模式走)
constexpr int SMS_STS_GOAL_TIME_H        = 45;
constexpr int SMS_STS_GOAL_SPEED_L       = 46; // 目标速度低字节(0.732rpm 每单位)
constexpr int SMS_STS_GOAL_SPEED_H       = 47;
constexpr int SMS_STS_LOCK               = 55; // EPROM 锁(1=锁住配置防误改)

// ---------- SRAM 只读(状态回读) ----------
constexpr int SMS_STS_PRESENT_POSITION_L = 56; // 当前位置低字节(0~4095=一整圈)
constexpr int SMS_STS_PRESENT_POSITION_H = 57;
constexpr int SMS_STS_PRESENT_SPEED_L    = 58; // 当前速度低字节(bit15=方向)
constexpr int SMS_STS_PRESENT_SPEED_H    = 59;
constexpr int SMS_STS_PRESENT_LOAD_L     = 60; // 当前负载低字节
constexpr int SMS_STS_PRESENT_LOAD_H     = 61;
constexpr int SMS_STS_PRESENT_VOLTAGE    = 62; // 当前电压
constexpr int SMS_STS_PRESENT_TEMPERATURE= 63; // 当前温度
constexpr int SMS_STS_MOVING             = 66; // 是否在运动中(0=到位 1=运动)
constexpr int SMS_STS_PRESENT_CURRENT_L  = 69; // 当前电流低字节
constexpr int SMS_STS_PRESENT_CURRENT_H  = 70;

// ============================================================================
// sms_sts 类：继承协议处理器(获得读/写/ping 等一切能力)，再补上"STS 人话 API"
// ============================================================================
class sms_sts : public protocol_packet_handler {
public:
    // 构造函数：绑定串口；STS 系列字节序 = 0(小端)
    // 同时初始化一个"同步写"分组器：从 ACC(41) 起每舵机写 7 字节
    // (7 字节布局 = ACC + 位置低/高 + 时间低/高(写0) + 速度低/高)
    explicit sms_sts(PortHandler* portHandler);

    // 公共成员：同步写分组器(Python 里是 self.groupSyncWrite，示例里会直接用)
    GroupSyncWrite groupSyncWrite;

    // ---- 高层 API(返回值=通信结果，数据/错误用引用带出) ---------------------
    // 带加速度地走到目标位置(位置单位 0~4095，速度单位 0.732rpm，加速度单位 8.7°/s²)
    int WritePosEx(int scs_id, int position, int speed, int acc, int& error);
    // 读当前位置(返回带正负的位置，0~4095)
    int ReadPos(int scs_id, int& present_position, int& error);
    // 读当前速度(bit15 为符号位，已解码为带正负)
    int ReadSpeed(int scs_id, int& present_speed, int& error);
    // 一次性读"位置+速度"(位置在低 16 位、速度在高 16 位)
    int ReadPosSpeed(int scs_id, int& present_position, int& present_speed, int& error);
    // 读"是否还在运动"
    int ReadMoving(int scs_id, int& moving, int& error);

    // 把"某舵机目标位置"加入同步写批次(成功 true)；发批次用 groupSyncWrite.txPacket()
    bool SyncWritePosEx(int scs_id, int position, int speed, int acc);
    // 异步写(暂存，等 RegAction 一起动)
    int RegWritePosEx(int scs_id, int position, int speed, int acc, int& error);
    // 触发所有异步写(广播)
    int RegAction();
    // 切到"轮子模式"(恒速转)，配合 WriteSpec 用
    int WheelMode(int scs_id, int& error);
    // 切到"位置模式"(角度定位)，配合 WritePosEx 用
    int PositionMode(int scs_id, int& error);
    // 轮子模式下的速度控制(正=正转 负=反转，速度单位同前)
    int WriteSpec(int scs_id, int speed, int acc, int& error);
    // 锁/解锁 EPROM 配置(锁上后 ID/波特率/角度范围等改不了)
    int LockEprom(int scs_id, int& error);
    int unLockEprom(int scs_id, int& error);
};

}  // namespace scs

#endif  // SMS_STS_H
