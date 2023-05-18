#include <motion_planner/custom_path_state.h>
#include <motion_planner/idle_state.h>
#include <motion_planner/parking_path_generator.h>
#include <trace/utils.h>

std::shared_ptr<State> CustomPathState::Generate(
    const int indexMatch, MotionPlannerContext & context)
{
    context.mBehavior.hint = false;
    ParkingPlannerUtils::ParkingPath parkingPath;

    ParkingPlannerUtils::ParkingSpaceInfo parkingSpaceInfo;
    ParkingPlannerUtils::ParkingSpaceInfoUpdate(
        context.mParkingStatus.parking_space.array,
        context.mGlobalPath->mPath, parkingSpaceInfo);

    const auto parkingSpaceType =
        static_cast<ParkingPlannerUtils::ParkingSpaceType>(
            context.mParkingStatus.parking_space.type);
    switch (parkingSpaceType)
    {
        case ParkingPlannerUtils::ParkingSpaceType::VERTICAL:
        {
            VerticalParking verticalParking;

            verticalParking.ParameterUpdate(
                parkingSpaceInfo.mParkingLocation,
                parkingSpaceInfo.mSearchPathHead,
                context.mParkingStatus.parking_space.array);

            verticalParking.PathGenerator(
                context.mParkingStatus.phase,
                parkingSpaceInfo.mParkingLocation,
                *context.mGlobalPath, parkingSpaceInfo.mSearchPathHead,
                context.mBehavior, context.mCarState, parkingPath);
            break;
        }

        case ParkingPlannerUtils::ParkingSpaceType::PARALLEL:
        {
            ParallelParking parallelParking;

            parallelParking.ParameterUpdate(
                parkingSpaceInfo.mParkingLocation,
                parkingSpaceInfo.mSearchPathHead,
                context.mParkingStatus.parking_space.array);

            parallelParking.PathGenerator(
                context.mParkingStatus.phase,
                parkingSpaceInfo.mParkingLocation,
                parkingSpaceInfo.mSearchPathHead,
                context.mCarState,
                context.mParkingStatus.parking_space.array,
                parkingPath);
            break;
        }

        case ParkingPlannerUtils::ParkingSpaceType::OBLIQUE_BACK_IN:
        {
            ObliqueParking obliqueParking;

            obliqueParking.ParameterUpdate(
                parkingSpaceInfo.mParkingLocation,
                parkingSpaceInfo.mSearchPathHead,
                context.mParkingStatus.parking_space.array);

            obliqueParking.PathGenerator(
                context.mParkingStatus.phase,
                parkingSpaceInfo.mParkingLocation,
                *context.mGlobalPath, parkingSpaceInfo.mSearchPathHead,
                context.mBehavior, context.mCarState, parkingPath);
            break;
        }
    }

    context.mLocalPaths.waypointArrays.clear();
    context.mLocalPaths.waypointArrays.push_back(parkingPath.path);
    context.mFeedbackParkingStatus.phase =
        static_cast<int>(parkingPath.phase);

    return NextState(context);
}

std::shared_ptr<State> CustomPathState::NextState(
    MotionPlannerContext & context)
{
    if (context.mLocalPaths.waypointArrays.size() > 0)
    {
        context.mPubParkingPathStatus.publish(context.mFeedbackParkingStatus);
        context.mPubLocalPaths.publish(context.mLocalPaths);
        context.mPubWaypoints.publish(context.mLocalPaths.waypointArrays[0]);
        return std::make_shared<IdleState>();
    }
    return shared_from_this();
}
