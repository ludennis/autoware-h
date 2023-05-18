#ifndef __VEHICLE_TRUCK_GATEWAY_CAN_FRAME_ID_H__
#define __VEHICLE_TRUCK_GATEWAY_CAN_FRAME_ID_H__

enum class CanFrameId
{
    DBW_CONTROL1 = 0x100,
    DBW_CONTROL2 = 0x200,
    DBW_CONTROL3 = 0x600,
    DBW_VCU_STATE1 = 0x300,
    DBW_VCU_STATE2 = 0x400,
    DBW_VCU_STATE3 = 0x500,
};

#endif // __VEHICLE_TRUCK_GATEWAY_CAN_FRAME_ID_H__
