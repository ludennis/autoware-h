#include <motion_planner/cost_function.h>
#include <file_loader.h>
#include <gtest/gtest.h>

static const float SMALL_VALUE = 1.0e-03f;

TEST(CostFunction, Convolution)
{
    // numbers form:
    // HU, Xuemin, et al. Dynamic path planning for autonomous driving
    // on various roads with avoidance of static and moving obstacles.
    // Mechanical Systems and Signal Processing, 2018, 100: 482-500.
    const std::vector<float> kernal = {9.13472e-12f, 6.07588e-09f,
        1.48672e-06f, 0.00013383f, 0.00443185f, 0.053991f, 0.241971f,
        0.398942f, 0.241971f, 0.053991f, 0.00443185f, 0.00013383f,
        1.48672e-06f, 6.07588e-09f, 9.13472e-12f};
    const std::vector<float> array = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f};

    const std::vector<float> result =
        CostFunction::Convolution(kernal, array);

    const std::vector<float> answer = {0.999999f, 0.999865f, 0.995433f,
        0.941444f, 0.699606f, 0.305096f, 0.117116f, 0.305095f, 0.699473f,
        0.937012f, 0.941442f, 0.757894f ,0.601057f, 0.758029f, 0.946009f};
    for (size_t i = 0; i < result.size(); ++ i)
    {
        ASSERT_NEAR(answer[i], result[i], SMALL_VALUE);
    }
}

TEST(CostFunction, FrenetTransform)
{
    itri_msgs::WaypointArray path;
    LoadPathData("cost_function/path.json", path);

    itri_msgs::DetectedObjectArray objectList;
    itri_msgs::DetectedObjectArray objectListSD;
    LoadObjectData("cost_function/object_list.json", objectList);
    LoadObjectData("cost_function/object_list.json", objectListSD);

    const float pathS = CostFunction::FrenetTransform(path, objectList);
    ASSERT_NEAR(28.4002f, pathS, SMALL_VALUE);

    for (size_t i = 0; i < objectList.objects.size(); ++ i)
    {
        for (size_t j = 0;
            j < objectList.objects[i].convex_hull.polygon.points.size(); ++ j)
        {
            ASSERT_NEAR(
                objectListSD.objects[i].convex_hull.polygon.points[j].s,
                objectList.objects[i].convex_hull.polygon.points[j].s,
                SMALL_VALUE);
            ASSERT_NEAR(
                objectListSD.objects[i].convex_hull.polygon.points[j].d,
                objectList.objects[i].convex_hull.polygon.points[j].d,
                SMALL_VALUE);
        }
    }
}

TEST(CostFunction, FindDangerObjects)
{
    itri_msgs::WaypointArray path;
    LoadPathData("cost_function/path.json", path);

    itri_msgs::DetectedObjectArray objectList;
    LoadObjectData("cost_function/object_list.json", objectList);

    itri_msgs::DetectedObjectArray dangerObjects;
    CostFunction::FindDangerObjects(
        path, 0.1f, 10.0f ,10.0f, objectList, dangerObjects);
    ASSERT_EQ(0, dangerObjects.objects.size());

    CostFunction::FindDangerObjects(
        path, 0.5f, 10.0f, 10.0f, objectList, dangerObjects);
    ASSERT_EQ(1, dangerObjects.objects.size());
    ASSERT_EQ(1, dangerObjects.objects[0].id);

    CostFunction::FindDangerObjects(
        path, 0.8f, 10.0f, 10.0f, objectList, dangerObjects);
    ASSERT_EQ(1, dangerObjects.objects.size());
    ASSERT_EQ(0, dangerObjects.objects[0].id);

    CostFunction::FindDangerObjects(
        path, 0.8f, 15.0f, 15.0f, objectList, dangerObjects);
    ASSERT_EQ(2, dangerObjects.objects.size());
    ASSERT_EQ(0, dangerObjects.objects[0].id);
    ASSERT_EQ(1, dangerObjects.objects[1].id);

    CostFunction::FindDangerObjects(
        path, 4.0f, 28.0f, 20.0f, objectList, dangerObjects);
    ASSERT_EQ(2, dangerObjects.objects.size());
    ASSERT_EQ(0, dangerObjects.objects[0].id);
    ASSERT_EQ(1, dangerObjects.objects[1].id);
}

TEST(CostFunction, HaveObjectOnPath)
{
    itri_msgs::WaypointArray path;
    LoadPathData("cost_function/path.json", path);

    itri_msgs::DetectedObjectArray objectList;
    LoadObjectData("cost_function/object_list.json", objectList);

    bool objectOnPath = CostFunction::HaveObjectOnPath(path, objectList);
    ASSERT_EQ(true, objectOnPath);

    objectList.objects.erase(
        objectList.objects.begin(), objectList.objects.begin() + 2);
    objectOnPath = CostFunction::HaveObjectOnPath(path, objectList);
    ASSERT_EQ(false, objectOnPath);
}

TEST(CostFunction, CollisionCheck)
{
    itri_msgs::WaypointArray path;
    itri_msgs::WaypointArrays localPaths;
    LoadPathData("cost_function/local_path1.json", path);
    localPaths.waypointArrays.push_back(path);
    LoadPathData("cost_function/local_path2.json", path);
    localPaths.waypointArrays.push_back(path);
    LoadPathData("cost_function/local_path3.json", path);
    localPaths.waypointArrays.push_back(path);

    itri_msgs::DetectedObjectArray objectList;
    LoadObjectData("cost_function/object_list2.json", objectList);

    const std::vector<float> collisionCheck = CostFunction::CollisionCheck(
        localPaths, 2.0f, 10.0f, 10.0f, objectList);

    ASSERT_NEAR(1.0f, collisionCheck[0], SMALL_VALUE);
    ASSERT_NEAR(0.0f, collisionCheck[1], SMALL_VALUE);
    ASSERT_NEAR(0.0f, collisionCheck[2], SMALL_VALUE);
}

TEST(CostFunction, CollisionCost)
{
    itri_msgs::WaypointArray path;
    itri_msgs::WaypointArrays localPaths;
    LoadPathData("cost_function/local_path1.json", path);
    localPaths.waypointArrays.push_back(path);
    LoadPathData("cost_function/local_path2.json", path);
    localPaths.waypointArrays.push_back(path);
    LoadPathData("cost_function/local_path3.json", path);
    localPaths.waypointArrays.push_back(path);

    itri_msgs::DetectedObjectArray objectList;
    LoadObjectData("cost_function/object_list2.json", objectList);

    const std::vector<float> collisionCost = CostFunction::CollisionCost(
        localPaths, 0.4f, 5.0f, objectList);

    ASSERT_NEAR(0.426975f, collisionCost[0], SMALL_VALUE);
    ASSERT_NEAR(0.286513f, collisionCost[1], SMALL_VALUE);
    ASSERT_NEAR(0.286513f, collisionCost[2], SMALL_VALUE);
}

TEST(CostFunction, TransitionCost)
{
    itri_msgs::WaypointArray path;
    itri_msgs::WaypointArrays localPaths;
    LoadPathData("cost_function/local_path1.json", path);
    localPaths.waypointArrays.push_back(path);
    LoadPathData("cost_function/local_path2.json", path);
    localPaths.waypointArrays.push_back(path);
    LoadPathData("cost_function/local_path3.json", path);
    localPaths.waypointArrays.push_back(path);

    LoadPathData("cost_function/global_path.json", path);

    const std::vector<float> transitionCost = CostFunction::TransitionCost(
        localPaths, path);

    ASSERT_NEAR(0.46080f, transitionCost[0], SMALL_VALUE);
    ASSERT_NEAR(0.32225f, transitionCost[1], SMALL_VALUE);
    ASSERT_NEAR(0.21695f, transitionCost[2], SMALL_VALUE);
}

TEST(CostFunction, GetPathCost)
{
    itri_msgs::WaypointArray path;
    itri_msgs::WaypointArrays localPaths;
    LoadPathData("cost_function/local_path1.json", path);
    localPaths.waypointArrays.push_back(path);
    LoadPathData("cost_function/local_path2.json", path);
    localPaths.waypointArrays.push_back(path);
    LoadPathData("cost_function/local_path3.json", path);
    localPaths.waypointArrays.push_back(path);

    LoadPathData("cost_function/global_path.json", path);

    itri_msgs::DetectedObjectArray objectList;
    LoadObjectData("cost_function/object_list2.json", objectList);

    MotionPlannerUtils::PlannerParam param;
    param.weightCollision = 0.7;
    param.weightTransition = 0.3;

    CostFunction::GetPathCost(path, 0.4f, 5.0f, param, objectList, localPaths);

    ASSERT_NEAR(0.43712f, localPaths.waypointArrays[0].cost, SMALL_VALUE);
    ASSERT_NEAR(0.29723f, localPaths.waypointArrays[1].cost, SMALL_VALUE);
    ASSERT_NEAR(0.26564f, localPaths.waypointArrays[2].cost, SMALL_VALUE);
}
