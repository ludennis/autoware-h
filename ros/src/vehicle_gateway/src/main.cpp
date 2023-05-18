#include <ros/init.h>
#include <vehicle_gateway_node.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "vehicle_gateway_node");
    VehicleGateway vehicleTruckGateway;

    while (ros::ok())
        vehicleTruckGateway.RunOnce();

    return 0;
}
