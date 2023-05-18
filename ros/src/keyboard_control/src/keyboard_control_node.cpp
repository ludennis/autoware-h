#include <ros/ros.h>
#include <signal.h>
#include <stdint.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <itri_msgs/blinker_cmd.h>
#include <itri_msgs/speed_cmd.h>
#include <itri_msgs/steer_cmd.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

itri_msgs::blinker_cmd gBlinkerCmd;
itri_msgs::speed_cmd gSpeedCmd;
itri_msgs::steer_cmd gSteerCmd;
bool reset = false;

template <typename T>
static inline T Clamp(const T value, T bottom, T top)
{
    return std::max(bottom, std::min(top, value));
}

class TeleopVehicle
{
public:
    TeleopVehicle();
    void keyLoop();

private:
    ros::NodeHandle mNodeHandler;
    ros::Publisher mPubBlinker;
    ros::Publisher mPubSpeed;
    ros::Publisher mPubSteer;
};

TeleopVehicle::TeleopVehicle()
{
    gBlinkerCmd.type = itri_msgs::blinker_cmd::NONE;
    gSpeedCmd.type = itri_msgs::speed_cmd::CLOSE_LOOP;
    gSpeedCmd.kph = 0.0f;
    gSteerCmd.type = itri_msgs::steer_cmd::CLOSE_LOOP;
    gSteerCmd.angle = 0.0f;
    mPubBlinker = mNodeHandler.advertise<itri_msgs::blinker_cmd>(
        "/blinker_cmd", 0);
    mPubSpeed = mNodeHandler.advertise<itri_msgs::speed_cmd>("/speed_cmd", 0);
    mPubSteer = mNodeHandler.advertise<itri_msgs::steer_cmd>("/steer_cmd", 0);
}

int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

void key_func()
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
    puts("---------------------------");
    puts("Use arrow keys to move the Vehicle.");

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        switch(c)
        {
            case KEYCODE_L:
                gSteerCmd.angle -= 5.0;
                reset = false;
                break;
            case KEYCODE_R:
                gSteerCmd.angle += 5.0;
                reset = false;
                break;
            case KEYCODE_U:
                gSpeedCmd.kph += 1.0;
                reset = false;
                break;
            case KEYCODE_D:
                gSpeedCmd.kph -= 1.0;
                reset = false;
                break;
            case 'R':
            case 'r':
                reset = true;
                gBlinkerCmd.type = itri_msgs::blinker_cmd::NONE;
                break;
            case '4':
                gBlinkerCmd.type = itri_msgs::blinker_cmd::LEFT;
                break;
            case '5':
                gBlinkerCmd.type = itri_msgs::blinker_cmd::HAZARD;
                break;
            case '6':
                gBlinkerCmd.type = itri_msgs::blinker_cmd::RIGHT;
                break;
        }
        gSpeedCmd.kph = Clamp(gSpeedCmd.kph, 0.0f, 40.0f);
        gSteerCmd.angle = Clamp(gSteerCmd.angle, -1050.0f, 1050.0f);
    }
}

void TeleopVehicle::keyLoop()
{
    std::thread ctrThread(key_func);
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        if (reset)
        {
            gSpeedCmd.kph *= 0.995f;
            gSteerCmd.angle *= 0.995f;
            if (std::abs(gSpeedCmd.kph) < 0.1f &&
                std::abs(gSteerCmd.angle) < 0.1f)
                reset = false;
        }
        mPubBlinker.publish(gBlinkerCmd);
        mPubSpeed.publish(gSpeedCmd);
        mPubSteer.publish(gSteerCmd);
        loop_rate.sleep();
    }
    ctrThread.join();
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_Vehicle");
    TeleopVehicle teleop_Vehicle;
    signal(SIGINT,quit);
    teleop_Vehicle.keyLoop();
    return(0);
}
