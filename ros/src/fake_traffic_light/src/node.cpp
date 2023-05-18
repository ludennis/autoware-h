#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include <itri_msgs/TrafficLightObjects.h>
#include <termios.h>
#include <thread>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41

static itri_msgs::TrafficLightObject gTrafficLightObject;
static bool gTmpForRouteSelect = false;

static void ResetLights()
{
    gTrafficLightObject.dontCare = false;
    gTrafficLightObject.green = false;
    gTrafficLightObject.left = false;
    gTrafficLightObject.red = false;
    gTrafficLightObject.right = false;
    gTrafficLightObject.straight = false;
    gTrafficLightObject.yellow = false;
}

class FakeTrafficLight
{
public:
    FakeTrafficLight();
    void KeyLoop();

private:
    ros::NodeHandle mNodeHangle;
    ros::Publisher mPubSignal;
};

FakeTrafficLight::FakeTrafficLight()
{
    mPubSignal = mNodeHangle.advertise<itri_msgs::TrafficLightObjects>(
        "traffic_light_status_fake", 0);
    ResetLights();
}

static int kfd = 0;
static struct termios cooked, raw;
void quit(int sig)
{
    (void) sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

void Key()
{
    char c;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        switch(c)
        {
            case 'r':
            case 'R':
                ROS_INFO("red light");
                gTrafficLightObject.red = true;
                gTrafficLightObject.green = false;
                gTrafficLightObject.yellow = false;
                break;
            case 'g':
            case 'G':
                ROS_INFO("green light");
                ResetLights();
                gTrafficLightObject.green = true;
                break;
            case 'y':
            case 'Y':
                ROS_INFO("yellow light");
                gTrafficLightObject.red = false;
                gTrafficLightObject.green = false;
                gTrafficLightObject.yellow = true;
                break;
            case KEYCODE_R:
                ROS_INFO("right light");
                gTrafficLightObject.right = !gTrafficLightObject.right;
                break;
            case KEYCODE_L:
                ROS_INFO("left light");
                gTrafficLightObject.left = !gTrafficLightObject.left;
                break;
            case KEYCODE_U:
                ROS_INFO("straight light");
                gTrafficLightObject.straight = !gTrafficLightObject.straight;
                break;
            case 'n':
            case 'N':
                ROS_INFO("no traffic light detected");
                ResetLights();
                break;
            case 'q':
            case 'Q':
                ROS_INFO("select other route");
                gTmpForRouteSelect = !gTmpForRouteSelect;
                gTrafficLightObject.score = gTmpForRouteSelect ? 1.0f : 0.0f;
                break;
            default:
                break;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_traffic_light");
    FakeTrafficLight fake_traffic_light;

    signal(SIGINT, quit);

    fake_traffic_light.KeyLoop();

    return(0);
}


void FakeTrafficLight::KeyLoop()
{
    std::thread ctrThread(Key);

    ros::Rate rate(10);
    while(ros::ok())
    {
        itri_msgs::TrafficLightObjects obj;
        obj.lights.push_back(gTrafficLightObject);
        mPubSignal.publish(obj);
        rate.sleep();
    }

    ctrThread.join();
    return;
}
