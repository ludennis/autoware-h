#include <can_frame_id_truck.h>
#include <vehicle_gateway_node.h>

#define TRACE_TAG "VehicleGatewayNode"

static float SPEED_LOWER_BOUND = -5.0f;
static float SPEED_UPPER_BOUND = 60.0f;
static float STEER_LIMIT = 1050.0f;
static float LATERAL_ACCELERATION_LIMIT = 2.0f;
static float STEER_CONSTANT = 0.00033018867f;
// Gear ratio of truck is 23.92
static const float SPEED_CTRL_TORQUE_GAIN = 2.0f;
static const float SPEED_CTRL_ACCE_GAIN = 0.004f;
static const float SPEED_CTRL_DCCE_GAIN = 0.4f;
static const float LIMITED_STEER_ANGLE = 1.8f;
static const bool USE_ITRI_EPS_CONTROLLER = false;

template <typename T>
static inline T Clamp(const T value, T bottom, T top)
{
    return std::max(bottom, std::min(top, value));
}

VehicleGateway::VehicleGateway()
    : mNodeHandle()
    , mRate(100)
    , mBlinkerCmdSubscriber()
    , mSpeedCmdSubscriber()
    , mSteerCmdSubscriber()
    , mSentMessagesPublisher()
    , mReceivedMessagesSubscriber()
    , mVehicleDbwStatePublisher()
    , mSpeedCmd()
    , mSteeringCmd()
    , mTruckControlCmd()
    , mTruckGearRatio()
    , mTargetEngineSpd(500.0f)
    , mIpcKeepAlive()
    , mEngineControlCmd()
    , mVehicleDbwState()
    , mSteerLimitESC(STEER_LIMIT)
    , mSelfDrivingStatus(false)
    , mResetSelfDrivingStatus(false)
    , mGatewayCounter(0)
    , mEpsControllerRight(std::make_shared<EPSController>(PARAM_FILE, "parameters_right", 0.01))
    , mEpsControllerLeft(std::make_shared<EPSController>(PARAM_FILE, "parameters_left",  0.01))
    , mEPSDiagnosis(0.5f, 0.01f, 70000.0f)
{
    mBlinkerCmdSubscriber =
        mNodeHandle.subscribe("/blinker_cmd", 0,
            &VehicleGateway::BlinkerCmdCallback, this);
    mSpeedCmdSubscriber =
        mNodeHandle.subscribe("/speed_cmd", 0,
            &VehicleGateway::SpeedCmdCallback, this);
    mSteerCmdSubscriber =
        mNodeHandle.subscribe("/steer_cmd", 0,
            &VehicleGateway::SteerCmdCallback, this);
    mSentMessagesPublisher =
        mNodeHandle.advertise<can_msgs::Frame>("/sent_messages", 0);

    mReceivedMessagesSubscriber =
        mNodeHandle.subscribe("/received_messages", 0,
            &VehicleGateway::ReceivedMessagesCallback, this);
    mVehicleDbwStatePublisher =
        mNodeHandle.advertise<itri_msgs::VehicleState>(
            "/vehicle_state", 0);

    bool steeringCmdEnable;
    bool speedCmdEnable;
    ros::NodeHandle privateNode("~");
    privateNode.param<bool>(
        "enable_steering_control", steeringCmdEnable, true);
    privateNode.param<bool>(
        "enable_speed_control", speedCmdEnable, true);
    mSteeringCmd.enable = steeringCmdEnable;
    mSpeedCmd.enable = speedCmdEnable;
}

void VehicleGateway::RunOnce()
{
    mVehicleDbwStatePublisher.publish(mVehicleDbwState);
    ros::spinOnce();
    OriginalMessagesCallback();
    GetSelfDrivingStatus();
    GetTruckGearRatio();
    GetTargetEngineSpeed();
    GetTruckControlCmd();
    GetCruiseControlCmd();
    GetGearControlCmd();
    IpcRollingCounterCallback();
    TruckControlCmdCallback();
    EngineControlCmdCallback();
    mRate.sleep();
}

float VehicleGateway::GetLimitedSteerCmd(const float originalSteerCommand)
{
    return originalSteerCommand;
    const float angleDifference = Clamp(
        originalSteerCommand - mVehicleDbwState.steering_angle,
        -LIMITED_STEER_ANGLE, LIMITED_STEER_ANGLE);

    return mVehicleDbwState.steering_angle + angleDifference;
}

void VehicleGateway::BlinkerCmdCallback(const itri_msgs::blinker_cmd & msg)
{
    mTruckControlCmd.enableBlinker = itri_msgs::VehicleState::DBW_ON;
    mTruckControlCmd.blinkerCmd = msg.type;
}

void VehicleGateway::SpeedCmdCallback(
    const itri_msgs::speed_cmd & msg)
{
    mSpeedCmd.type = msg.type;
    switch (msg.type)
    {
        case itri_msgs::speed_cmd::OPEN_LOOP:
            mSpeedCmd.value = msg.acceleration;
            break;
        case itri_msgs::speed_cmd::CLOSE_LOOP:
            mSpeedCmd.value = Clamp(
                msg.kph, SPEED_LOWER_BOUND, SPEED_UPPER_BOUND);
            break;
    }
}

void VehicleGateway::SteerCmdCallback(
    const itri_msgs::steer_cmd & msg)
{
    mSteeringCmd.type = msg.type;
    switch (msg.type)
    {
        case itri_msgs::steer_cmd::OPEN_LOOP:
            mSteeringCmd.value = msg.torque;
            break;
        case itri_msgs::steer_cmd::CLOSE_LOOP:
            mSteeringCmd.value = Clamp(msg.angle, -STEER_LIMIT, STEER_LIMIT);
            break;
    }
    mSteeringCmd.rolling_counter ++;
}

void VehicleGateway::ReceivedMessagesCallback(const can_msgs::Frame & frame)
{
    switch (static_cast<CanFrameId>(frame.id))
    {
        case CanFrameId::DBW_VCU_STATE1:
            {
                const TruckDbwStateRaw1 * rawData =
                    reinterpret_cast<const TruckDbwStateRaw1* const>(
                        frame.data.data());

                mVehicleDbwState.gateway_keep_alive = rawData->gatewayKeepAlive;

            }
            break;
        case CanFrameId::DBW_VCU_STATE2:
            {
                const TruckDbwStateRaw2 * rawData =
                    reinterpret_cast<const TruckDbwStateRaw2* const>(
                        frame.data.data());

                mVehicleDbwState.speed = rawData->vehicleSpeed / 256.0;
                mVehicleDbwState.amt_gear_range = rawData->amtGearRange - 2;
                mVehicleDbwState.amt_gear_pos = rawData->amtGearPos;
                mVehicleDbwState.blinker = rawData->blinker;
                mVehicleDbwState.aeb_set = rawData->aebSet;
                mVehicleDbwState.brake_pedal = rawData->brakePedal;
                mVehicleDbwState.steer_mcu_error = rawData->steerMcuError;
                mVehicleDbwState.steering_angle = 3200.0 - 0.1 * (rawData->steerWheelAngle);
                mVehicleDbwState.engine_speed = rawData->engineSpeed * 0.125;
            }
            break;
        case CanFrameId::DBW_VCU_STATE3:
            {
                const TruckDbwStateRaw3 * rawData =
                    reinterpret_cast<const TruckDbwStateRaw3* const>(
                        frame.data.data());

                mVehicleDbwState.ctrl_switch_aeb = rawData->ctrlSwithchAeb;
                mVehicleDbwState.ctrl_switch_amt = rawData->ctrlSwithchAmt;
                mVehicleDbwState.ctrl_switch_blinker = rawData->ctrlSwithchBlinker;
                mVehicleDbwState.ctrl_switch_brake = rawData->ctrlSwithchBrake;
                mVehicleDbwState.ctrl_switch_engine = rawData->ctrlSwithchEngine;
                mVehicleDbwState.ctrl_switch_steer = rawData->ctrlSwithchSteer;
            }
            break;
    }
    mSteerLimitESC = LATERAL_ACCELERATION_LIMIT / STEER_CONSTANT /
        std::pow((mVehicleDbwState.speed + 1.0f) / 3.6f, 2.0f);
}

void VehicleGateway::OriginalMessagesCallback()
{
    mVehicleDbwState.steering_torque = 0;
    // mode
    if (mVehicleDbwState.ctrl_switch_amt > 0 &&
        mVehicleDbwState.ctrl_switch_brake > 0 &&
        mVehicleDbwState.ctrl_switch_engine > 0 &&
        mVehicleDbwState.ctrl_switch_steer > 0 &&
        mVehicleDbwState.ctrl_switch_blinker > 0 &&
        mSelfDrivingStatus == true)
    {
        mVehicleDbwState.mode = 4;
    }
    else if ((mVehicleDbwState.ctrl_switch_amt < 1 ||
        mVehicleDbwState.ctrl_switch_brake < 1 ||
        mVehicleDbwState.ctrl_switch_engine < 1) &&
        mVehicleDbwState.ctrl_switch_steer > 0 &&
        mVehicleDbwState.ctrl_switch_blinker > 0 &&
        mSelfDrivingStatus == true)
    {
        mVehicleDbwState.mode = 3;
    }
    else if (mVehicleDbwState.ctrl_switch_amt > 0 &&
        mVehicleDbwState.ctrl_switch_brake > 0 &&
        mVehicleDbwState.ctrl_switch_engine > 0 &&
        mVehicleDbwState.ctrl_switch_steer < 1 &&
        mVehicleDbwState.ctrl_switch_blinker > 0 &&
        mSelfDrivingStatus == true)
    {
        mVehicleDbwState.mode = 2;
    }
    else if (mVehicleDbwState.ctrl_switch_amt < 1 &&
        mVehicleDbwState.ctrl_switch_brake < 1 &&
        mVehicleDbwState.ctrl_switch_engine < 1 &&
        mVehicleDbwState.ctrl_switch_steer < 1 &&
        mVehicleDbwState.ctrl_switch_blinker > 0)
    {
        mVehicleDbwState.mode = 1;
    }
    else
    {
        mVehicleDbwState.mode = 0;
    }

    // gear_state
    if (mVehicleDbwState.amt_gear_range <= -1)
    {
        mVehicleDbwState.gear_state = 1;
    }
    else if (mVehicleDbwState.amt_gear_range == 0)
    {
        mVehicleDbwState.gear_state = 2;
    }
    else if (mVehicleDbwState.amt_gear_range >= 1)
    {
        mVehicleDbwState.gear_state = 3;
    }
    else
    {
        mVehicleDbwState.gear_state = 0;
    }

    // eps_state
    if (mVehicleDbwState.steer_mcu_error == 0 &&
        mVehicleDbwState.ctrl_switch_steer == 1 &&
        (mSteeringCmd.enable < 1 ||
        mVehicleDbwState.ctrl_switch_blinker < 1))
    {
        mVehicleDbwState.eps_state = 1;
    }
    else if (mVehicleDbwState.steer_mcu_error == 0 &&
        mVehicleDbwState.ctrl_switch_steer == 1 &&
        mSteeringCmd.enable > 0 &&
        mVehicleDbwState.ctrl_switch_blinker > 0)
    {
        mVehicleDbwState.eps_state = 2;
    }
    else
    {
        mVehicleDbwState.eps_state = 0;
    }

    // throttle_state
    if (mVehicleDbwState.ctrl_switch_amt > 0 &&
        mVehicleDbwState.ctrl_switch_brake > 0 &&
        mVehicleDbwState.ctrl_switch_engine > 0 &&
        mSpeedCmd.enable < 1)
    {
        mVehicleDbwState.throttle_state = 1;
    }
    else if (mVehicleDbwState.ctrl_switch_amt > 0 &&
        mVehicleDbwState.ctrl_switch_brake > 0 &&
        mVehicleDbwState.ctrl_switch_engine > 0 &&
        mSpeedCmd.enable > 0)
    {
        mVehicleDbwState.throttle_state = 2;
    }
    else
    {
        mVehicleDbwState.throttle_state = 0;
    }
}

void VehicleGateway::GetSelfDrivingStatus()
{
    if (mVehicleDbwState.ctrl_switch_blinker == 1 &&
        mVehicleDbwState.brake_pedal == 0 &&
        mSelfDrivingStatus != true &&
        mResetSelfDrivingStatus != true)
    {
        mSelfDrivingStatus = true;
    }
    if (mSelfDrivingStatus != false &&
        (mVehicleDbwState.ctrl_switch_blinker == 0 ||
        mVehicleDbwState.brake_pedal == 1 ))
    {
        mSelfDrivingStatus = false;
        mResetSelfDrivingStatus = true;
    }
    if (mVehicleDbwState.ctrl_switch_blinker == 0)
    {
        mResetSelfDrivingStatus = false;
    }
}

void VehicleGateway::GetGearControlCmd()
{
    if (mSelfDrivingStatus == true &&
        mVehicleDbwState.ctrl_switch_amt > 0 &&
        mSpeedCmd.value > 0.5f &&
        mVehicleDbwState.gear_state != 3)
    {
        if (mVehicleDbwState.gear_state == 1)
        {
            mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_ON;
            mTruckControlCmd.brakeDecelerationCmd =
                static_cast<uint16_t>(1024 * (10.0f - 1.0f));
            if (mVehicleDbwState.speed < 1.0f)
            {
                mTruckControlCmd.enableAMT = itri_msgs::VehicleState::DBW_ON;
                mTruckControlCmd.amtGearCmd = static_cast<uint8_t>(2);
            }
        }
        else if (mVehicleDbwState.gear_state == 2)
        {
            mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_ON;
            mTruckControlCmd.brakeDecelerationCmd =
                static_cast<uint16_t>(1024 * (10.0f - 1.0f));
            if (mVehicleDbwState.speed < 1.0f)
            {
                mTruckControlCmd.enableAMT = itri_msgs::VehicleState::DBW_ON;
                mTruckControlCmd.amtGearCmd = static_cast<uint8_t>(4);
            }
        }
    }
    else if (mSelfDrivingStatus == true &&
        mVehicleDbwState.ctrl_switch_amt > 0 &&
        mSpeedCmd.value < -0.5f &&
        mVehicleDbwState.gear_state != 1)
    {
        if (mVehicleDbwState.gear_state == 3)
        {
            mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_ON;
            mTruckControlCmd.brakeDecelerationCmd =
                static_cast<uint16_t>(1024 * (10.0f - 1.0f));
            if (mVehicleDbwState.speed < 1.0f)
            {
                mTruckControlCmd.enableAMT = itri_msgs::VehicleState::DBW_ON;
                mTruckControlCmd.amtGearCmd = static_cast<uint8_t>(2);
            }
        }
        else if (mVehicleDbwState.gear_state == 2)
        {
            mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_ON;
            mTruckControlCmd.brakeDecelerationCmd =
                static_cast<uint16_t>(1024 * (10.0f - 1.0f));
            if (mVehicleDbwState.speed < 1.0f)
            {
                mTruckControlCmd.enableAMT = itri_msgs::VehicleState::DBW_ON;
                mTruckControlCmd.amtGearCmd = static_cast<uint8_t>(0);
            }
        }
    }
    else
    {
        mTruckControlCmd.enableAMT = itri_msgs::VehicleState::DBW_OFF;
        mTruckControlCmd.amtGearCmd = static_cast<uint8_t>(2);
    }
}

void VehicleGateway::GetTruckGearRatio()
{
    const float gearRatio1 = 10.127f;
    const float gearRatio2 = 6.414f;
    const float gearRatio3 = 4.038f;
    const float gearRatio4 = 2.507f;
    const float gearRatio5 = 1.588f;
    const float gearRatio6 = 1.0f;
    const float gearRatioR = 9.902f;
    const float gearRatioLow = 1.0f;
    const float gearRatioHigh = 0.795f;
    const float gearRatioFinal = 3.727f;

    if (mVehicleDbwState.amt_gear_pos == 1)
    {
        mTruckGearRatio = gearRatio1 * gearRatioLow * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 2)
    {
        mTruckGearRatio = gearRatio1 * gearRatioHigh * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 3)
    {
        mTruckGearRatio = gearRatio2 * gearRatioLow * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 4)
    {
        mTruckGearRatio = gearRatio2 * gearRatioHigh * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 5)
    {
        mTruckGearRatio = gearRatio3 * gearRatioLow * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 6)
    {
        mTruckGearRatio = gearRatio3 * gearRatioHigh * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 7)
    {
        mTruckGearRatio = gearRatio4 * gearRatioLow * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 8)
    {
        mTruckGearRatio = gearRatio4 * gearRatioHigh * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 9)
    {
        mTruckGearRatio = gearRatio5 * gearRatioLow * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 10)
    {
        mTruckGearRatio = gearRatio5 * gearRatioHigh * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 11)
    {
        mTruckGearRatio = gearRatio6 * gearRatioLow * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_pos == 12)
    {
        mTruckGearRatio = gearRatio6 * gearRatioHigh * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_range == -1)
    {
        mTruckGearRatio = gearRatioR * gearRatioLow * gearRatioFinal;
    }
    else if (mVehicleDbwState.amt_gear_range == -2)
    {
        mTruckGearRatio = gearRatioR * gearRatioHigh * gearRatioFinal;
    }
    else
    {
        mTruckGearRatio = gearRatio1 * gearRatioHigh * gearRatioFinal;
    }

}

void VehicleGateway::GetTargetEngineSpeed()
{
    const float wheelRadius = 0.5f;
    const float unitConvert = 0.10472f * 3.6f;
    if (std::abs(mSpeedCmd.value) < 3.6f && std::abs(mSpeedCmd.value) > 0.2f)
        mTargetEngineSpd = 600.0f;
    else if (std::abs(mSpeedCmd.value) >= 3.6f)
    {
        if (mVehicleDbwState.speed < 3.0f)
            mTargetEngineSpd = 600.0f;
        else
            mTargetEngineSpd =
                mTruckGearRatio * std::abs(mSpeedCmd.value) / (wheelRadius * unitConvert);
    }
    else
        mTargetEngineSpd = 500.0f;

    mTargetEngineSpd = std::max(mTargetEngineSpd, 500.0f);
}

void VehicleGateway::GetCruiseControlCmd()
{
    const auto epsDiagnosisState = mEPSDiagnosis.Diagnosis(
        mVehicleDbwState.steering_angle);
    if(epsDiagnosisState == EPSDiagnosisState::FAIL)
    {
        mSpeedCmd.value = 0.9f * mVehicleDbwState.speed;
        mTruckControlCmd.enableSteer = 0;
        mVehicleDbwState.eps_state = 0;
    }

    const float engineSpeedError = mTargetEngineSpd - mVehicleDbwState.engine_speed;
    const float vehicleSpeedError = mSpeedCmd.value - mVehicleDbwState.speed;
    float accelCmd = 0.0f;
    float torqueCmd = 0.0f;
    float speedCmd = 0.0f;
    float decelCmd = 0.0f;
    mEngineControlCmd.engineCtrlMode = itri_msgs::VehicleState::ENG_CTRL_MODE_ACCEL;

    switch (mEngineControlCmd.engineCtrlMode)
    {
        case itri_msgs::VehicleState::ENG_CTRL_MODE_ACCEL:
            if (mSpeedCmd.value > 0.2f && engineSpeedError > 0.0f)
            {
                mTruckControlCmd.enableEngine = itri_msgs::VehicleState::DBW_ON;
                mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_OFF;
                accelCmd = std::min(engineSpeedError * SPEED_CTRL_ACCE_GAIN, 15.999f);
                decelCmd = 0.0f;
            }
            if (vehicleSpeedError <= 0.0f)
            {
                mTruckControlCmd.enableEngine = itri_msgs::VehicleState::DBW_ON;
                mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_ON;
                accelCmd = 0.0f;
                decelCmd = std::max(vehicleSpeedError * SPEED_CTRL_DCCE_GAIN, -9.999f);
                decelCmd = mVehicleDbwState.speed > 3.9f ? decelCmd : 0.0f;
            }
            if (mVehicleDbwState.speed < 5.0f && mSpeedCmd.value < 0.2f)
            {
                mTruckControlCmd.enableEngine = itri_msgs::VehicleState::DBW_OFF;
                mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_ON;
                accelCmd = 0.0f;
                decelCmd = -1.0f;
            }
            break;
        case itri_msgs::VehicleState::ENG_CTRL_MODE_TORQUE:
            if (mSpeedCmd.value > 0.2f && vehicleSpeedError > 0.0f)
            {
                mTruckControlCmd.enableEngine = itri_msgs::VehicleState::DBW_ON;
                mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_OFF;
                torqueCmd = std::min(vehicleSpeedError * SPEED_CTRL_TORQUE_GAIN, 100.0f);
                decelCmd = 0.0f;
            }
            if (vehicleSpeedError <= 0.0f)
            {
                mTruckControlCmd.enableEngine = itri_msgs::VehicleState::DBW_ON;
                mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_ON;
                torqueCmd = 0.0f;
                decelCmd = std::max(vehicleSpeedError * SPEED_CTRL_DCCE_GAIN, -9.999f);
                decelCmd = mVehicleDbwState.speed > 3.9f ? decelCmd : 0.0f;
            }
            if (mVehicleDbwState.speed < 5.0f && mSpeedCmd.value < 0.2f)
            {
                mTruckControlCmd.enableEngine = itri_msgs::VehicleState::DBW_OFF;
                mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_ON;
                torqueCmd = 0.0f;
                decelCmd = -1.0f;
            }
            break;
        case itri_msgs::VehicleState::ENG_CTRL_MODE_SPEED:
            mTruckControlCmd.enableEngine = itri_msgs::VehicleState::DBW_ON;
            mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_ON;
            speedCmd = mSpeedCmd.value;
            break;
    }

    if (mSelfDrivingStatus != true)
    {
        mTruckControlCmd.enableEngine = itri_msgs::VehicleState::DBW_OFF;
        mTruckControlCmd.enableBrake = itri_msgs::VehicleState::DBW_OFF;
        decelCmd = 0.0f;
        accelCmd = 0.0f;
        torqueCmd = 0.0f;
    }

    mVehicleDbwState.enable_engine_ctr = mTruckControlCmd.enableEngine;
    mVehicleDbwState.enable_brake_ctr = mTruckControlCmd.enableBrake;
    mVehicleDbwState.engine_ctrl_mode_cmd = mEngineControlCmd.engineCtrlMode;
    mVehicleDbwState.engine_target_speed = mTargetEngineSpd;
    mVehicleDbwState.engine_accel_cmd = accelCmd;
    mVehicleDbwState.engine_torque_cmd = torqueCmd;
    mVehicleDbwState.engine_speed_cmd = speedCmd;
    mVehicleDbwState.deceleration_cmd = decelCmd;
    mVehicleDbwState.speedCmd = mSpeedCmd.value;
    mTruckControlCmd.engineTqCmd =
        static_cast<uint8_t>(mVehicleDbwState.engine_torque_cmd);
    mTruckControlCmd.accelerationCmd =
        static_cast<uint16_t>(mVehicleDbwState.engine_accel_cmd * 1024.0f);
    mTruckControlCmd.brakeDecelerationCmd =
        static_cast<uint16_t>(1024.0f * (10.0f + mVehicleDbwState.deceleration_cmd));
    mEngineControlCmd.speedCmd =
        static_cast<uint16_t>(mVehicleDbwState.engine_speed_cmd * 256.0f);
}

void VehicleGateway::GetTruckControlCmd()
{
    if (mSelfDrivingStatus == true)
    {
        mTruckControlCmd.enableSteer = static_cast<uint8_t>(mSteeringCmd.enable);
        mTruckControlCmd.enableBlinker = static_cast<uint8_t>(mSteeringCmd.enable);
        mTruckControlCmd.enableAeb = static_cast<uint8_t>(0);
        mTruckControlCmd.aebRequestCmd = static_cast<uint8_t>(0);
        float steerCmdRight = mEpsControllerRight->OneStep(1.92f * mSteeringCmd.value - mVehicleDbwState.steering_angle);
        float steerCmdLeft = mEpsControllerLeft->OneStep(1.93f * mSteeringCmd.value - mVehicleDbwState.steering_angle);
        float steerCmd = !std::signbit(1.9f * mSteeringCmd.value - mVehicleDbwState.steering_angle) ? steerCmdRight : steerCmdLeft;
        if (USE_ITRI_EPS_CONTROLLER)
            steerCmd = Clamp(steerCmd, -1050.0f, 1050.0f);
        else
            steerCmd = Clamp(mSteeringCmd.value, -1050.0f, 1050.0f);
        mVehicleDbwState.steering_cmd = GetLimitedSteerCmd(steerCmd);
        mTruckControlCmd.steerCmd =
            static_cast<uint16_t>((GetLimitedSteerCmd(steerCmd) + 1050.0f) * 10.0f);
    }
    else
    {
        mTruckControlCmd = {};
        mEngineControlCmd = {};
    }
}

void VehicleGateway::TruckControlCmdCallback()
{
    can_msgs::Frame canFrame;
    canFrame.is_extended = false;
    canFrame.is_rtr = false;
    canFrame.is_error = false;
    canFrame.dlc = 8;

    const uint8_t (*truckControlData)[8] =
        reinterpret_cast<const uint8_t (*const)[8]>(&mTruckControlCmd);
    canFrame.header.stamp = ros::Time::now();
    canFrame.id = static_cast<int>(CanFrameId::DBW_CONTROL1);
    std::copy(std::begin(*truckControlData), std::end(*truckControlData),
        canFrame.data.begin());
    mSentMessagesPublisher.publish(canFrame);
    mTruckControlCmd.enableBlinker = false;
}

void VehicleGateway::IpcRollingCounterCallback()
{
    can_msgs::Frame canFrame;
    canFrame.is_extended = false;
    canFrame.is_rtr = false;
    canFrame.is_error = false;
    canFrame.dlc = 8;
    mGatewayCounter ++;
    if(mGatewayCounter >= 100)
    {
        mGatewayCounter = 0;
    }
    mIpcKeepAlive.rollingCounter = mGatewayCounter / 10;

    const uint8_t (*counterData)[8] =
        reinterpret_cast<const uint8_t (*const)[8]>(&mIpcKeepAlive);
    canFrame.header.stamp = ros::Time::now();
    canFrame.id = static_cast<int>(CanFrameId::DBW_CONTROL2);
    std::copy(std::begin(*counterData), std::end(*counterData),
        canFrame.data.begin());
    mSentMessagesPublisher.publish(canFrame);
}

void VehicleGateway::EngineControlCmdCallback()
{
    can_msgs::Frame canFrame;
    canFrame.is_extended = true;
    canFrame.is_rtr = false;
    canFrame.is_error = false;
    canFrame.dlc = 8;

    const uint8_t (*engineControlData)[8] =
        reinterpret_cast<const uint8_t (*const)[8]>(&mEngineControlCmd);
    canFrame.header.stamp = ros::Time::now();
    canFrame.id = static_cast<int>(CanFrameId::DBW_CONTROL3);
    std::copy(std::begin(*engineControlData), std::end(*engineControlData),
        canFrame.data.begin());
    mSentMessagesPublisher.publish(canFrame);
}
