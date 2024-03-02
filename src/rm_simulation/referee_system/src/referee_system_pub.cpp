#include "referee_system/referee_system.h"
#include <ros/ros.h>
#include <thread>
#include <future>
#include <iostream>
#include <sstream>

referee_system::referee_system msg;
ros::Publisher pub;

void init_msg()
{
    msg.color = 0;

    msg.b1_blood = 150;
    msg.b1_bullet = 20;
    msg.b3_blood = 150;
    msg.b3_bullet = 200;
    msg.b4_blood = 150;
    msg.b4_bullet = 200;
    msg.b7_blood = 600;
    msg.b7_bullet = 750;
    msg.blue_base_blood = 1500;
    msg.blue_base_protected = true;
    msg.blue_base_shield = 1500;
    msg.blue_energy = 0;

    msg.benefit_area_open = false;

    msg.r1_blood = 150;
    msg.r1_bullet = 20;
    msg.r3_blood = 150;
    msg.r3_bullet = 200;
    msg.r4_blood = 150;
    msg.r4_bullet = 200;
    msg.r7_blood = 600;
    msg.r7_bullet = 750;
    msg.red_base_blood = 1500;
    msg.red_base_proteted = true;
    msg.red_base_shield = 1500;
    msg.red_energy = 0;
}

void pub_msg()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        pub.publish(msg);
        rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "referee_pub");

    ros::NodeHandle nh;
    pub = nh.advertise<referee_system::referee_system>("referee_system", 10);

    init_msg();

    auto tree_future = std::async(std::launch::async, pub_msg);

    std::string input;
    while (true)
    {
        std::getline(std::cin, input);
        std::istringstream s(input);
        std::string name;
        int num;
        ROS_INFO("b");
        if (s >> name >> num){
            ROS_INFO("a");
            if (name == "r1"){
                msg.r1_blood = num;
            }
            else if (name == "r3"){
                msg.r3_blood = num;
            }
            else if (name == "r4"){
                msg.r4_blood = num;
            }
            else if (name == "r7"){
                msg.r7_blood = num;
            }
            else if (name == "b1"){
                msg.b1_blood = num;
            }
            else if (name == "b3"){
                msg.b3_blood = num;
            }
            else if (name == "b4"){
                msg.b4_blood = num;
            }
            else if (name == "b7"){
                msg.b7_blood = num;
            }
            else if (name == "red_base"){
                msg.red_base_blood = num;
            }
            else if (name == "blue_base"){
                msg.blue_base_blood = num;
            }
            else{
                break;
            }
        }
    }

    return 0;
}
