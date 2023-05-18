#ifndef __VEHICLE_TRUCK_GATEWAY_VCU_MESSAGE_DATA_H__
#define __VEHICLE_TRUCK_GATEWAY_VCU_MESSAGE_DATA_H__

#include <stdint.h>

enum class CtrlCmdType : uint8_t
{
    OPEN_LOOP,
    CLOSE_LOOP,
};

#pragma pack(push, 1)

struct ControlCmdRaw
{
    uint8_t enable : 1;
    uint8_t type : 1;
    uint8_t rolling_counter : 6;
    uint32_t unused : 24;
    float value;

    ControlCmdRaw()
        : enable(1)
        , type(static_cast<uint8_t>(CtrlCmdType::CLOSE_LOOP))
        , rolling_counter(0)
        , unused(0)
        , value(0.0f)
    {}
};

struct ControlCmdRaw1
{
    uint16_t steerCmd : 15;
    uint8_t enableSteer : 1;
    uint16_t brakeDecelerationCmd : 14;
    uint8_t enableBrake : 1;
    uint8_t unused0 : 1;
    uint8_t engineTqCmd : 7;
    uint8_t enableEngine : 1;
    uint16_t accelerationCmd : 14;
    uint8_t unused1 : 1;
    uint8_t enableAMT : 1;
    uint8_t amtGearCmd : 3;
    uint8_t blinkerCmd : 2;
    uint8_t enableBlinker : 1;
    uint8_t aebRequestCmd : 1;
    uint8_t enableAeb : 1;

    ControlCmdRaw1()
        : steerCmd(0)
        , enableSteer(0)
        , brakeDecelerationCmd(0)
        , enableBrake(0)
        , unused0(0)
        , engineTqCmd(0)
        , enableEngine(0)
        , accelerationCmd(0)
        , unused1(0)
        , enableAMT(0)
        , amtGearCmd(0)
        , blinkerCmd(0)
        , enableBlinker(0)
        , aebRequestCmd(0)
        , enableAeb(0)
    {}
};

struct ControlCmdRaw2
{
    uint8_t rollingCounter;
    uint64_t unused : 56;

    ControlCmdRaw2()
        : rollingCounter(0)
        , unused(0)
    {}
};

struct ControlCmdRaw3
{
    uint16_t speedCmd;
    uint8_t  engineCtrlMode : 2;
    uint64_t unused : 46;

    ControlCmdRaw3()
        : speedCmd(0)
        , engineCtrlMode(0)
        , unused(0)
    {}
};

struct TruckDbwStateRaw1
{
    uint8_t gatewayKeepAlive;
    uint64_t unused : 56;
};

struct TruckDbwStateRaw2
{
    uint16_t vehicleSpeed;
    uint8_t  amtGearRange : 3;
    uint8_t unused : 1;
    uint8_t amtGearPos : 4;
    uint8_t blinker : 2;
    uint8_t aebSet : 2;
    uint8_t brakePedal : 2;
    uint8_t steerMcuError : 2;
    uint16_t steerWheelAngle;
    uint16_t engineSpeed;
};

struct TruckDbwStateRaw3
{
    uint8_t ctrlSwithchAeb : 2;
    uint8_t ctrlSwithchAmt : 2;
    uint8_t ctrlSwithchBlinker : 2;
    uint8_t ctrlSwithchBrake : 2;
    uint8_t ctrlSwithchEngine : 2;
    uint8_t ctrlSwithchSteer : 2;
    uint64_t unused : 52;
};


#pragma pack(pop)

static_assert(
    sizeof(ControlCmdRaw) == 8, "Packed structure size check failed");
static_assert(
    sizeof(ControlCmdRaw1) == 8, "Packed structure size check failed");
static_assert(
    sizeof(ControlCmdRaw2) == 8, "Packed structure size check failed");
static_assert(
    sizeof(ControlCmdRaw3) == 8, "Packed structure size check failed");
static_assert(
    sizeof(TruckDbwStateRaw1) == 8, "Packed structure size check failed");
static_assert(
    sizeof(TruckDbwStateRaw2) == 8, "Packed structure size check failed");
static_assert(
    sizeof(TruckDbwStateRaw3) == 8, "Packed structure size check failed");

#endif // __VEHICLE_GATEWAY_VCU_MESSAGE_DATA_H__
