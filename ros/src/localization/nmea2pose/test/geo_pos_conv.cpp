#include <gtest/gtest.h>
#include <geo_pos_conv.h>

TEST(GeoPoseConvert, GetXYZ)
{
    double lat = 24.775084704;
    double lon = 121.045888961;
    double h = 146.593;

    ITRI::GeoPosConv geo;
    geo.SetGPSReference(0);
    geo.ConvLLH2XYZ(lat, lon, h);

    ASSERT_NEAR(geo.x(), 0, 0.001);
    ASSERT_NEAR(geo.y(), 0, 0.001);
    ASSERT_NEAR(geo.z(), 0, 0.001);
}

TEST(GeoPoseConvert, GetLLH)
{
    double x = 0;
    double y = 0;
    double z = 0;

    ITRI::GeoPosConv geo;
    geo.SetGPSReference(0);
    geo.ConvXYZ2LLH(x, y, z);

    ASSERT_NEAR(geo.latitude(), 24.775, 0.001);
    ASSERT_NEAR(geo.longitude(), 121.045, 0.001);
    ASSERT_NEAR(geo.altitude(), 146.593, 0.001);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}