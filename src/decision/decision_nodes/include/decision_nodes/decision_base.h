#ifndef DECISION_BASE
#define DECISION_BASE

#include <referee_system/referee_system.h>
#include <geometry_msgs/PoseStamped.h>
#include <list>
#include <ros/ros.h>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>

struct RefereeMsg
//接收裁判系统的消息
{
    bool color;                 //敌方颜色 0红色 1蓝色
    int remaining_time;         //剩余时间
    int my_base_blood;          //我方基地血量
    bool my_base_protected;     //我方基地无敌
    int my_base_shield;         //我方基地护盾量
    int enemy_base_blood;       //敌方基地血量
    int my_guard_blood;         //我方哨兵血量
    int my_guard_bullet;        //我方哨兵子弹
    int enemy_guard_blood;      //敌方哨兵血量
    int my_robot1;              //我方英雄血量
    int my_robot1_bullet;       //我方英雄子弹
    int enemy_robot1;           //敌方英雄血量
    int my_robot3;              //我方3号步兵血量
    int my_robot3_bullet;       //我方3号步兵子弹
    int enemy_robot3;           //敌方3号步兵血量
    int my_robot4;              //我方4号步兵血量
    int my_robot4_bullet;       //我方4号步兵子弹
    int enemy_robot4;           //敌方4号步兵血量
    int my_energy;              //我方能量
    int enemy_energy;           //敌方能量
    bool benefit_open;          //增益区是否打开

    RefereeMsg(const referee_system::referee_system& msg);
};

struct Point
//坐标信息
{
    //初始点相对于中心点的偏移量
    static double offset_x;
    static double offset_y;

    double x;
    double y;

    Point(){}
    Point(double x, double y);
};

struct VisualMsg
//视觉信息
{
    double time;            //信息接受时间
    Point robot_pos;        //座标点

    VisualMsg(){}
    VisualMsg(double time, double x, double y);
};

class decision_base
//所有执行节点的基类 使用静态成员便于统一消息
{
private:
    static std::list<RefereeMsg> recent_referee[2];     //接收到的裁判系统信息
    static std::list<VisualMsg> enemy_guard;            //敌方哨兵视觉信息
    static std::list<VisualMsg> enemy_r1;               //敌方英雄视觉信息
    static std::list<VisualMsg> enemy_r3;               //敌方3号步兵视觉信息
    static std::list<VisualMsg> enemy_r4;               //敌方4号步兵视觉信息
    static std::list<VisualMsg> my_r1;                  //我方英雄视觉信息
    static std::list<VisualMsg> my_r3;                  //我方3号步兵视觉信息
    static std::list<VisualMsg> my_r4;                  //我方4号步兵视觉信息

    static Point pos;                       //哨兵坐标
    static double time;                     //比赛时间
    static int elive;                       //敌方存活数量
    static int mlive;                       //我方存活数量

    static int ally_blood;                  //友方血量
    static int ally_blood_decrease;         //(3s) 友方血量减少
    static int ally_shut;                   //(3s) 友方子弹发射数量
    static int ally_live;                   //友方存活数
    static int my_shut;                     //(3s) 我方子弹发射数量
    static int my_blood;                //我方血量总和
    static int my_blood_decrease;           //(3s) 我方血量减少

    static int enemy_blood;             //敌方血量总和
    static int enemy_blood_decrease;        //(3s) 敌方血量减少 
    static int enemy_manual_live;           //敌方非哨兵单位存活数
    static int enemy_manual_blood;          //敌方非哨兵单位血量
    static int enemy_manual_blood_decrease; //(3s) 敌方非哨兵单位血量减少
    
    static double min_dist;                 //最小到达距离
    static double max_radius;               //最大浮动半径
    static bool beginning_state;            //刚开局阶段
    static int recover_blood;               //恢复血量

    static nav_msgs::OccupancyGrid &grid_map;       //栅格地图
    static std::string tmp_goal;            //临时目标
    static std::string current_pos;         //当前位置描述
    static bool beginning_state_locked;     //开局状态封锁
    bool static occupy_benefit;             //占领增益区

    int static Ally_Amount_In_Specific_Pos(int area);   //某个区域的队友人数
    int static Enemy_Amount_In_Specific_Pos(int area);  //某个区域敌方人数
    geometry_msgs::PoseStamped static Standarize_Pose(double x, double y);       //将坐标点转换为消息格式
    geometry_msgs::PoseStamped static Standarize_Pose(Point p);                  //将坐标点转换为消息格式
    bool static Is_Position_Valid();                    //判断当前点是否合法
    bool static Seek_Valid_Position();                  //寻找合法点

public:
    static int type;
    static geometry_msgs::PoseStamped *GOAL;
    static geometry_msgs::PoseStamped *TMP_GOAL;

    static geometry_msgs::PoseStamped BENEFIT_LEFT;             //增益区左边
    static geometry_msgs::PoseStamped MIDDLE;                   //中心点
    static geometry_msgs::PoseStamped MIDDLE_LEFT;              //中间靠左
    static geometry_msgs::PoseStamped MIDDLE_RIGHT;             //中间右边

    static geometry_msgs::PoseStamped MY_RECHARGE;              //我方补给区
    static geometry_msgs::PoseStamped MY_BASE_ENTRY;            //我方基地入口
    static geometry_msgs::PoseStamped MY_BASE_UP_LEFT;          //我方基地左前
    static geometry_msgs::PoseStamped MY_BASE_UP_RIGHT;         //我方基地右前
    static geometry_msgs::PoseStamped MY_BASE_DOWN_LEFT;        //我方基地左后
    static geometry_msgs::PoseStamped MY_BASE_DOWN_RIGHT;       //我方基地右后
    static geometry_msgs::PoseStamped MY_WALL_LEFT;             //我方墙体左边
    static geometry_msgs::PoseStamped MY_WALL_RIGHT;            //我方墙体右边
    static geometry_msgs::PoseStamped MY_WALL_BACK_LEFT;        //我方墙体左后
    static geometry_msgs::PoseStamped MY_WALL_BACK_MID;         //我方墙体靠后中间
    static geometry_msgs::PoseStamped MY_WALL_BACK_RIGHT;       //我方墙体靠后最右
    static geometry_msgs::PoseStamped MY_DOMAIN_LEFT;           //我方区域左边
    static geometry_msgs::PoseStamped MY_DOMAIN_MID;            //我方区域中间
    static geometry_msgs::PoseStamped MY_DOMAIN_RIGHT;          //我方区域右边

    static geometry_msgs::PoseStamped ENEMY_WALL_RIGHT;         //敌方墙体右边
    static geometry_msgs::PoseStamped ENEMY_WALL_LEFT;          //敌方墙体左边
    static geometry_msgs::PoseStamped ENEMY_WALL_FRONT_LEFT;    //敌方墙体靠前左边
    static geometry_msgs::PoseStamped ENEMY_WALL_FRONT_MIDDLE;  //敌方墙体靠前中间
    static geometry_msgs::PoseStamped ENEMY_WALL_FRONT_RIGHT;   //敌方墙体靠前右边
    static geometry_msgs::PoseStamped ENEMY_BASE_ENTRY;         //敌方基地入口

    static geometry_msgs::PoseStamped STORED_POSITION;          //中间储存位置

    //更新哨兵坐标
    void static Update_Current_Pos(double x, double);
    //更新裁判系统消息
    void static Update_Referee_Msg(const referee_system::referee_system &msg);
    //更新视觉信息
    void static Update_Visual_Msg();
    //到达目标点
    bool static Close_Enough();
    //初始化静态成员变量
    void static Init_Static_Variable();
    //设置目标
    void static Set_Goal(std::string msg);
    //设置坐标偏移量
    void static Set_Offset(double x, double y);
    //设置目标点为当前位置
    void static Stay_Put();
    //修改目标点至合法点
    void static Modify_Goal();
    //增大搜索半径
    void static Add_Max_Radius();
    //初始化最大半径
    void static Init_Max_Radius();
    //更新当前位置信息
    void static Set_Current_Position(std::string posi);
    //获取当前位置信息
    std::string static Get_Current_Position();
    //设置栅格地图
    void static Set_Grid_Map(const nav_msgs::OccupancyGrid& map);
    //追踪机器人设置临时点位
    void static Set_Temp_Goal(double x, double y, bool del = true);
    //获取追踪机器人的位置
    bool static Get_Trace_Robot_Pos(double &x, double &y, std::string name);
    

    /*统一判断条件*/
    bool static Is_Game_Over();                         //判断是否结束
    bool static Is_Beginning_State();                   //判断是否为刚开局阶段
    void static End_Beginning_State();                  //结束开局状态
    bool static Is_Enemy_Guard_Killed();                //敌方哨兵阵亡 
    bool static My_Guard_Hurt();                        //我方哨兵残血
    bool static Inferior_Situation();                   //劣势
    bool static Favorable_Situation();                  //优势
    bool static Enemy_Guaed_Dissappear();               //敌方哨兵消失
    bool static Ally_Stay_by();                         //队友在一起作战
    bool static Low_Attack_Efficience();                //攻击效率低
    bool static Sub_Health();                           //亚健康
    bool static Recover_Limitted();                     //血量恢复到达上限
    bool static Enemy_Manual_Killed();                  //敌方手动机器人阵亡
    bool static Attacking_Ragingly();                   //哨兵攻击较强
    bool static Base_Been_Attacked();                   //基地被攻击
    bool static Enemy_Attack_Slowly();                  //敌方攻击很慢
    bool static Low_Blood();                            //血量低于150
    bool static Ally_Poor_State();                      //友方状态差
    bool static Attacking();                            //攻击中
    bool static Find_Enemy(std::string name);           //发现敌方机器人
    bool static Whther_Occupy_Benefit();                //是否占领增益区
    bool static Blood_Less_200();                       //血量少于250
    bool static Recover_Finished();                     //血量恢复完成



    /*开局前往敌方基地  发现敌方哨兵来我方基地*/
    bool static state_locked();                         //状态锁
    void static lock_state();                           //锁住状态
    bool static Enemy_Guard_At_Left();                  //敌方哨兵出现在左方
    bool static Enemy_Guard_RetratL();                  //敌方哨兵撤退
    bool static Enemy_Guard_In_My_Domain();             //敌方哨兵在我方区域
    bool static Enemy_Guard_In_Enemy_Doamin();          //敌方哨兵在敌方区域
    bool static Enemy_Guard_In_Unknow_Domain();         //敌方哨兵位置未知

    /*开局前往敌方基地  发现敌方哨兵在正前方*/
    bool static Enemy_Guard_In_Front();                 //敌方哨兵在正前方
    bool static Enemy_Guard_Advanced();                 //敌方哨兵前进
    bool static Enemy_Guard_Retrait_R();                //敌方哨兵撤退
    bool static Posi_Over_Half();                       //位置过半
    bool static Enemy_Guard_RetratR();                  //敌方哨兵撤退

    /*开局前往敌方基地  敌方哨兵不动*/
    bool static Blood_Over_200();                       //血量高于200

    /*侦察一圈*/
    bool static Enemy_Guard_In_My_Domain_Right();       //在我方区域右方发现敌方
    bool static Enemy_Guard_In_Middle_Right();          //在中间靠右发现敌方哨兵

    /*基地被攻击*/
    bool static Bese_Been_Attacked();                   //基地被攻击

    /*寻找徘徊点*/
    void static Set_Wander_Point();
    void static Is_Wander_Point_Valid(double x, double y);
    void static Seek_Wander_Point(double x, double y, bool flag[]);

};

#endif