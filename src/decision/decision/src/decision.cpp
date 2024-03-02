#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_msgs/TFMessage.h>
#include <thread>
#include <future>
#include <nav_msgs/OccupancyGrid.h>

#include "decision_nodes/decision_nodes.hpp"
#include "decision_nodes/decision_base.h"

using namespace BT;

//坐标偏移量
double Point::offset_x = 0;
double Point::offset_y = 0;

std::list<RefereeMsg> decision_base::recent_referee[2];     //接收到的裁判系统信息
std::list<VisualMsg> decision_base::enemy_guard;            //敌方哨兵视觉信息
std::list<VisualMsg> decision_base::enemy_r1;               //敌方英雄视觉信息
std::list<VisualMsg> decision_base::enemy_r3;               //敌方3号步兵视觉信息
std::list<VisualMsg> decision_base::enemy_r4;               //敌方4号步兵视觉信息
std::list<VisualMsg> decision_base::my_r1;                  //我方英雄视觉信息
std::list<VisualMsg> decision_base::my_r3;                  //我方3号步兵视觉信息
std::list<VisualMsg> decision_base::my_r4;                  //我方4号步兵视觉信息

Point decision_base::pos;                       //哨兵坐标
double decision_base::time;                     //比赛时间
int decision_base::elive;                       //敌方存活数量
int decision_base::mlive;                       //我方存活数量
int decision_base::ally_blood;                  //友方血量
int decision_base::ally_blood_decrease;         //(3s) 友方血量减少
int decision_base::ally_shut;                   //(3s) 友方子弹发射数量
int decision_base::ally_live;                   //友方存活数
int decision_base::my_shut;                     //(3s) 我方子弹发射数量
int decision_base::my_blood;                    //我方血量总和
int decision_base::my_blood_decrease;           //(3s) 我方血量减少
int decision_base::enemy_blood;                 //敌方血量总和
int decision_base::enemy_blood_decrease;        //(3s) 敌方血量减少 
int decision_base::enemy_manual_live;           //敌方非哨兵单位存活数
int decision_base::enemy_manual_blood;          //敌方非哨兵单位血量
int decision_base::enemy_manual_blood_decrease; //(3s) 敌方非哨兵单位血量减少
double decision_base::min_dist;                 //最小到达距离
double decision_base::max_radius;               //最大浮动半径
bool decision_base::beginning_state;            //刚开局阶段
int decision_base::recover_blood;               //恢复血量
nav_msgs::OccupancyGrid tmp;
nav_msgs::OccupancyGrid &decision_base::grid_map = tmp;     //栅格地图
std::string decision_base::tmp_goal;            //临时目标
std::string decision_base::current_pos;         //当前位置描述
bool decision_base::beginning_state_locked;     //开局状态锁
bool decision_base::occupy_benefit;             //占领增益区

int decision_base::type;
geometry_msgs::PoseStamped *decision_base::GOAL;
geometry_msgs::PoseStamped *decision_base::TMP_GOAL;
geometry_msgs::PoseStamped decision_base::BENEFIT_LEFT;             //增益区左边
geometry_msgs::PoseStamped decision_base::MIDDLE;                   //中心点
geometry_msgs::PoseStamped decision_base::MIDDLE_LEFT;              //中间靠左
geometry_msgs::PoseStamped decision_base::MIDDLE_RIGHT;             //中间最有
geometry_msgs::PoseStamped decision_base::MY_RECHARGE;              //我方补给区
geometry_msgs::PoseStamped decision_base::MY_BASE_ENTRY;            //我方基地入口
geometry_msgs::PoseStamped decision_base::MY_BASE_UP_LEFT;          //我方基地左前
geometry_msgs::PoseStamped decision_base::MY_BASE_UP_RIGHT;         //我方基地右前
geometry_msgs::PoseStamped decision_base::MY_BASE_DOWN_LEFT;        //我方基地左后
geometry_msgs::PoseStamped decision_base::MY_BASE_DOWN_RIGHT;       //我方基地右后
geometry_msgs::PoseStamped decision_base::MY_WALL_LEFT;             //我方墙体左边
geometry_msgs::PoseStamped decision_base::MY_WALL_RIGHT;            //我方墙体右边
geometry_msgs::PoseStamped decision_base::MY_WALL_BACK_LEFT;        //我方墙体靠后最左
geometry_msgs::PoseStamped decision_base::MY_WALL_BACK_MID;         //我方墙体靠后中间
geometry_msgs::PoseStamped decision_base::MY_WALL_BACK_RIGHT;       //我方墙体靠后最右
geometry_msgs::PoseStamped decision_base::MY_DOMAIN_LEFT;           //我方区域最左
geometry_msgs::PoseStamped decision_base::MY_DOMAIN_MID;            //我方区域中间
geometry_msgs::PoseStamped decision_base::MY_DOMAIN_RIGHT;          //我方区域右边
geometry_msgs::PoseStamped decision_base::ENEMY_WALL_RIGHT;         //敌方墙体右边
geometry_msgs::PoseStamped decision_base::ENEMY_WALL_LEFT;          //敌方墙体左边
geometry_msgs::PoseStamped decision_base::ENEMY_WALL_FRONT_LEFT;    //敌方墙体靠前左边
geometry_msgs::PoseStamped decision_base::ENEMY_WALL_FRONT_MIDDLE;  //敌方墙体靠前中间
geometry_msgs::PoseStamped decision_base::ENEMY_WALL_FRONT_RIGHT;   //敌方墙体靠前右边
geometry_msgs::PoseStamped decision_base::ENEMY_BASE_ENTRY;         //敌方基地入口
geometry_msgs::PoseStamped decision_base::STORED_POSITION;          //当前位置

//裁判系统消息回调函数
void referee_System_Callback(const referee_system::referee_system &msg)
{
    decision_base::Update_Referee_Msg(msg);
}

//坐标平移量
double tf_x_odom = 0, tf_y_odom = 0;
double tf_x_map = 0, tf_y_map = 0;
//tf消息回调函数
void tf_Callback(const tf2_msgs::TFMessage &tf_msg)
{
    for (auto &msg : tf_msg.transforms)
    {
        if (msg.header.frame_id == "odom" && msg.child_frame_id == "base_link")
        {
            tf_x_odom = msg.transform.translation.x;
            tf_y_odom = msg.transform.translation.y;
        }
        else if (msg.header.frame_id == "map" && msg.child_frame_id == "odom")
        {
            tf_x_map = msg.transform.translation.x;
            tf_y_map = msg.transform.translation.y;
        }
    }
    decision_base::Update_Current_Pos(tf_x_map+tf_x_odom, tf_y_map+tf_y_odom);
}

//栅格地图回调函数
void grid_Map_Callback(const nav_msgs::OccupancyGrid& map)
{
    decision_base::Set_Grid_Map(map);
}

//启动行为树
void start_Behavior_Tree()
{
    ROS_INFO("start to execute behavior tree");

    BehaviorTreeFactory factory;
    //注册行为树节点
    RegisterNodes(factory);
    //读取xml文件
    std::string path = ros::package::getPath("decision");
    factory.registerBehaviorTreeFromFile(path + "/tree/main_tree.xml");
    //factory.createTreeFromFile(path + "/tree/main_tree.xml");
    auto tree = factory.createTree("MainTree");
    //执行行为树
    tree.tickWhileRunning();

    ROS_INFO("behavior tree is over");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "decision_node");
    ros::NodeHandle nh;

    //初始化静态成员变量
    decision_base::Init_Static_Variable();
    //订阅节点
    ros::Subscriber sub1 = nh.subscribe("referee_system", 10, referee_System_Callback);
    ros::Subscriber sub2 = nh.subscribe("/tf", 10, tf_Callback);
    ros::Subscriber sub3 = nh.subscribe("/move_base/global_costmap/costmap", 1, grid_Map_Callback);

    //发布节点
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    //异步执行行为树
    auto tree_future = std::async(std::launch::async, start_Behavior_Tree);

    //设置消息发送频率为 10hz
    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        if (decision_base::GOAL == NULL){
            ROS_INFO("hh");
        }
        pub.publish(*decision_base::GOAL);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}