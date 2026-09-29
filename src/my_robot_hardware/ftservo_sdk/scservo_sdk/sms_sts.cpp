// ============================================================================
// sms_sts.cpp —— SMS/STS 系列舵机高层 API(实现)
//   逐函数对照 Python sms_sts.py
//   核心套路(务必看懂一次，之后就全懂了)：
//     "让舵机动" = 把 {加速度, 目标位置, 0, 0, 目标速度} 这 7 字节
//                 从寄存器地址 ACC=41 开始连续写进去。
//     地址 41..47 的排布：
//        41=ACC | 42=PosL | 43=PosH | 44=TimeL(写0) | 45=TimeH(写0) | 46=SpdL | 47=SpdH
//     Time 写 0 表示"不按固定时间走"，改由速度参数控制快慢。
// ============================================================================

#include "scservo_def.h"
#include "sms_sts.h"

namespace scs {

// 构造函数：等价 Python sms_sts.__init__
//   Python: protocol_packet_handler.__init__(self, portHandler, 0)   # 0=小端
//           self.groupSyncWrite = GroupSyncWrite(self, SMS_STS_ACC, 7)
sms_sts::sms_sts(PortHandler* portHandler)
    : protocol_packet_handler(portHandler, 0),   // 基类：绑定串口 + 小端字节序
      groupSyncWrite(this, SMS_STS_ACC, 7)       // 同步写分组器：ACC 起、每舵机 7 字节
{
}

// ============================================================================
// WritePosEx：最常见的"让舵机带加速度、带速度限制走到某位置"
//   入参：
//     position : 目标位置(0~4095)。可带负号？不——这里是角度值，先 toscs 编码，
//                若为负会按位 15 编码(一般角度模式用不到负)
//     speed    : 限速(单位 0.732 rpm)
//     acc      : 加速度(单位 8.7 °/s²)
//   Python 返回 writeTxRx 的 (result, error)
// ============================================================================
int sms_sts::WritePosEx(int scs_id, int position, int speed, int acc, int& error)
{
    position = scs_toscs(position, 15);   // 编码成舵机原始 16 位(负值置 bit15)

    // 组装从 ACC=41 开始要连续写入的 7 个字节
    std::vector<uint8_t> txpacket = {
        static_cast<uint8_t>(acc),                        // 地址41 ACC 加速度
        static_cast<uint8_t>(scs_lobyte(position)),       // 地址42 目标位置低字节
        static_cast<uint8_t>(scs_hibyte(position)),       // 地址43 目标位置高字节
        0,                                                // 地址44 时间低(0=按速度走)
        0,                                                // 地址45 时间高
        static_cast<uint8_t>(scs_lobyte(speed)),          // 地址46 目标速度低字节
        static_cast<uint8_t>(scs_hibyte(speed))           // 地址47 目标速度高字节
    };

    // 从 ACC 地址起写这 7 字节，并等舵机回执(能拿到错误)
    return writeTxRx(scs_id, SMS_STS_ACC, static_cast<int>(txpacket.size()),
                     txpacket, error);
}

// 读当前位置：从 56(PRESENT_POSITION_L) 读 2 字节 -> 拼字 -> 按 bit15 解码带符号
int sms_sts::ReadPos(int scs_id, int& present_position, int& error)
{
    int raw = 0;
    int result = read2ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L, raw, error);
    present_position = scs_tohost(raw, 15);   // 解码带符号(角度模式通常为正)
    return result;
}

// 读当前速度(bit15 表示方向，故必须 tohost 解码出正负)
int sms_sts::ReadSpeed(int scs_id, int& present_speed, int& error)
{
    int raw = 0;
    int result = read2ByteTxRx(scs_id, SMS_STS_PRESENT_SPEED_L, raw, error);
    present_speed = scs_tohost(raw, 15);
    return result;
}

// 一次读 4 字节拿到"位置(低16位)+速度(高16位)"，减少总线往返(更快更省)
int sms_sts::ReadPosSpeed(int scs_id, int& present_position, int& present_speed,
                          int& error)
{
    int raw = 0;
    int result = read4ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L, raw, error);
    // 32 位里：低 16 位是位置，高 16 位是速度
    int pos = scs_loword(raw);
    int spd = scs_hiword(raw);
    present_position = scs_tohost(pos, 15);   // 各自按 bit15 解码
    present_speed = scs_tohost(spd, 15);
    return result;
}

// 读舵机是否还在运动(0=已到位，1=运动中)——常用于"等它到位再干下一件事"
int sms_sts::ReadMoving(int scs_id, int& moving, int& error)
{
    uint8_t m = 0;
    int result = read1ByteTxRx(scs_id, SMS_STS_MOVING, m, error);
    moving = m;
    return result;
}

// ============================================================================
// SyncWritePosEx：同步写版 WritePosEx。
//   注意：它【不直接发】，只是把数据塞进 groupSyncWrite 的批次里。
//   攒完所有舵机后，由调用方执行 groupSyncWrite.txPacket() 一次性广播出去。
//   返回 true/false = addParam 是否成功
// ============================================================================
bool sms_sts::SyncWritePosEx(int scs_id, int position, int speed, int acc)
{
    position = scs_toscs(position, 15);
    std::vector<uint8_t> txpacket = {
        static_cast<uint8_t>(acc),
        static_cast<uint8_t>(scs_lobyte(position)),
        static_cast<uint8_t>(scs_hibyte(position)),
        0, 0,
        static_cast<uint8_t>(scs_lobyte(speed)),
        static_cast<uint8_t>(scs_hibyte(speed))
    };
    return groupSyncWrite.addParam(scs_id, txpacket);
}

// 异步写版 WritePosEx：先 REG_WRITE 暂存，之后调 RegAction() 一起执行
int sms_sts::RegWritePosEx(int scs_id, int position, int speed, int acc, int& error)
{
    position = scs_toscs(position, 15);
    std::vector<uint8_t> txpacket = {
        static_cast<uint8_t>(acc),
        static_cast<uint8_t>(scs_lobyte(position)),
        static_cast<uint8_t>(scs_hibyte(position)),
        0, 0,
        static_cast<uint8_t>(scs_lobyte(speed)),
        static_cast<uint8_t>(scs_hibyte(speed))
    };
    return regWriteTxRx(scs_id, SMS_STS_ACC, static_cast<int>(txpacket.size()),
                        txpacket, error);
}

// 触发所有异步写：广播 ACTION(0x05)，让之前 RegWrite 暂存的目标一起动
int sms_sts::RegAction()
{
    return action(BROADCAST_ID);
}

// 切到轮子模式：把地址 33(MODE) 写 1 (0=角度/定位模式, 1=轮子/恒速模式)
int sms_sts::WheelMode(int scs_id, int& error)
{
    return write1ByteTxRx(scs_id, SMS_STS_MODE, 1, error);
}

// PositionMode：把运行模式寄存器写回 0，恢复角度定位模式。
// 位置模式下，舵机会按照 WritePosEx 写入的目标位置运动。
int sms_sts::PositionMode(int scs_id, int& error)
{
    return write1ByteTxRx(scs_id, SMS_STS_MODE, 0, error);
}

// ============================================================================
// WriteSpec：轮子模式下的速度控制。速度带符号：正=正转 负=反转，数值=速率
//   txpacket = {acc, 0,0,0,0, spdL, spdH} 位置区全 0(轮子模式不关心位置)
// ============================================================================
int sms_sts::WriteSpec(int scs_id, int speed, int acc, int& error)
{
    speed = scs_toscs(speed, 15);   // 负速度 -> bit15 置 1 编码
    std::vector<uint8_t> txpacket = {
        static_cast<uint8_t>(acc),
        0, 0, 0, 0,                                   // 位置/时间区(轮子模式不用)
        static_cast<uint8_t>(scs_lobyte(speed)),
        static_cast<uint8_t>(scs_hibyte(speed))
    };
    return writeTxRx(scs_id, SMS_STS_ACC, static_cast<int>(txpacket.size()),
                     txpacket, error);
}

// 锁 EPROM：往地址 55(LOCK) 写 1。锁上后 ID/波特率/角度限位等配置改不了(防误写)
int sms_sts::LockEprom(int scs_id, int& error)
{
    return write1ByteTxRx(scs_id, SMS_STS_LOCK, 1, error);
}

// 解锁 EPROM：往 LOCK 写 0
int sms_sts::unLockEprom(int scs_id, int& error)
{
    return write1ByteTxRx(scs_id, SMS_STS_LOCK, 0, error);
}

}  // namespace scs
