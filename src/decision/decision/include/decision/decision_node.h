#include <tf/tf.h>
#include <ros/ros.h>
#include <referee_system/referee_system.h>
#include <geometry_msgs/PoseStamped.h>
#include <list>

int type = 0;   //0四台步兵   1我方双步兵 敌方英雄   2我方英雄 敌方双步兵   3双英雄

//接收到的有用信息
struct RefereeMsg
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
    RefereeMsg(const referee_system::referee_system& msg)
    {
        color = msg.color;
        remaining_time = msg.remaining_time;
        my_base_blood = color ? msg.red_base_blood : msg.blue_base_blood;
        my_base_protected = color ? msg.red_base_proteted : msg.blue_base_protected;
        my_base_shield = color ? msg.red_base_shield : msg.blue_base_shield;
        enemy_base_blood = color ? msg.blue_base_blood : msg.red_base_blood;
        my_guard_blood = color ? msg.r7_blood : msg.b7_blood;
        my_guard_bullet = color ? msg.r7_bullet : msg.b7_bullet;
        enemy_guard_blood = color ? msg.b7_blood : msg.r7_blood;
        my_robot3 = color ? msg.r3_blood : msg.b3_blood;
        my_robot3_bullet = color ? msg.r3_bullet : msg.b3_bullet;
        enemy_robot3 = color ? msg.b3_blood : msg.r3_blood;
        if (type == 0 || type == 1)
        {
            my_robot4 = color ? msg.r4_blood : msg.b4_blood;
            my_robot4_bullet = color ? msg.r4_bullet : msg.b4_bullet;
        }
        else
        {
            my_robot1 = color ? msg.r1_blood : msg.b1_blood;
            my_robot1_bullet = color ? msg.r1_bullet : msg.b1_bullet;
        }
        if (type == 0 || type == 3)
        {
            enemy_robot4 = color ? msg.b4_blood : msg.r4_blood;
        }
        else
        {
            enemy_robot1 = color ? msg.b1_blood : msg.r1_blood;
        }
        my_robot4 = color ? msg.r4_blood : msg.b4_blood;
        enemy_robot4 = color ? msg.b4_blood : msg.r4_blood;
        my_energy = color ? msg.red_energy : msg.blue_energy;
        enemy_energy = color ? msg.blue_energy : msg.blue_energy;
        benefit_open = msg.benefit_area_open;
    }
};

struct VisualMsg
{
    double time;
    double robot_posi[2];
};

class DecisionMaker
{
private:
    double x, y;
    double time = -1;
    int elive;
    int mlive;

    int ally_blood;
    int ally_blood_decrease;
    int ally_shut;
    int ally_live;
    int my_live;
    int my_shut;
    int my_blood_sum;
    int my_blood_decrease;

    int enemy_live;
    int enemy_blood_sum;
    int enemy_blood_decrease;
    int enemy_ally_live;
    int enemy_ally_blood;
    int enemy_ally_blood_decrease;

    std::list<RefereeMsg> recent_referee[2];            //用来记录3秒内接收到的裁判系统消息
    std::list<VisualMsg> enemy_guard;
    std::list<VisualMsg> enemy_r1;
    std::list<VisualMsg> enemy_r3;
    std::list<VisualMsg> enemy_r4;
    std::list<VisualMsg> my_r1;
    std::list<VisualMsg> my_r3;
    std::list<VisualMsg> my_r4;
    int instruction = 0;    //当前工作
    //0:开局 1:开局前往对方补给区 2:向增左 攻击敌方哨兵 3:在增左 攻击敌方哨兵5s 4:停留在增左 等待下一指令  5:我方哨兵在增左残血
    //6:我方增左劣势  7:在增左 敌方哨兵阵亡  8:在增左 敌方哨兵消失  9:在增左 敌方哨兵撤退  10:在增左 攻击效率低
    //11:在增左打不过 返回我方掩体后方  12:敌方哨兵阵亡 前往敌方补给区前  13:敌方哨兵阵亡 回补给区  14:在我方补给区找到敌方哨兵  15:在敌方补给区找到敌方哨兵
    //16:未找到敌方哨兵 17:敌方哨兵撤退 跟踪敌方  18:敌方哨兵撤退 回补给区  19:敌方哨兵撤退 去干扰敌方步兵  20:敌方哨兵出现在敌方补给区前 前往中间最右边
    //21:在中右 攻击敌方哨兵5s  22:停留中右 等待下一指令  23：在中右劣势  24:在中右 敌方哨兵阵亡  25:在中右  敌方哨兵消失
    //26:在中右 敌方哨兵前进  27:在中右 敌方哨兵撤退  28:在中右 攻击效率低  29:在中右打不过 退回到我方掩体后方  30:前往敌方掩体右方
    //31:开局前往敌方补给区前  32:在敌方补给区前攻击  33:敌方补给区前劣势  34:敌方补给区前 敌方哨兵阵亡  35:敌方补给区前 敌方哨兵消失
    //36:敌方补给区前 敌方步兵阵亡  37:敌方补给区前攻击效率低  38:劣势 返回我方掩体正后方  39:从敌方补给区返回中间右方  40:前往中间最左方
    //41:返回我方领域左方  42:在中间右方发现敌人  43:敌方哨兵在我方领域靠右  44:在中间左方攻击敌方5s  45:在我方区域左方攻击
    //46:在敌方领域侦察一圈  47:在中间靠右攻击敌方  48:在中间左方攻击敌人  49:前往我方掩体右前方  50:在基地入口攻击
    //51:在敌方掩体左前方攻击  52:在敌方掩体左前方劣势  53:敌方哨兵阵亡 在敌方掩体左前方  54:前往我方区域右出口

    //1001:离开增左并恢复完成(劣势) 在补给区 1002:退回到我方掩体左后方 1003:前进到敌方补给区前 1004:离开增左并恢复完成(优势) 在补给区 1005:在我方领域发现敌方哨兵 我方掩体后方靠左领域
    //1006:敌方哨兵在未知位置 前往敌方掩体右方  1007:退回到我方掩体右后方  1008:前往到敌方掩体右方  1009:退回到我方掩体正后方  1010:在我方领域左方位置保护
    //1011:前往我方掩体右前方放哨  1012:在我方基地入口保护  1013:在我方区域右出口

    //1101:前往基地左前方       1102:离开基地左前方         1103:前往基地右前方         1104:离开基地右前方         1105:前往基地右后方
    //1106:离开基地右后方       1107:前往基地左后方         1108:离开基地左后方         1109:返回补给区恢复         1110:离开补给区
    //1111:前往掩体右后         1112:离开掩体右后          1113:前往掩体左后           1114:离开掩体左后           1115

    //1201:前往掩体右          1202:离开掩体右              1203:前往掩体左:            1204:离开掩体左             1205:前往中右
    //1206:离开中右            1207:前往中左               1208:离开中左               1209:前往中心               1210:离开中心
    
    //1301:前往敌方掩体右       1302:离开敌方掩体右          1303:前往敌方掩体左          1304:离开敌方掩体左        1305:前往敌方区域右
    //1306:离开敌方区域右       1307:前往敌方掩体前          1308:离开敌方掩体前          1309:前往敌方基地          1310:离开敌方基地

    //1401:巡视

    //移动地点
    double Point[40][3] = {
        //此坐标基于蓝方视角
        {3, -3, 0},         //0  敌方补给区前
        {0, 1, 0},          //1  增益区左边缘
        {-5.4, 3.4, 0},     //2  我方补给区
        {-2, 1, 0},         //3  我方掩体左后方
        {1.5, -3, 0},       //4  敌方掩体右方
        {0, 3, 0},          //5  中间位置最左方
        {-2, 3, 0},         //6  我方掩体后方靠左领域
        {0, -3, 0},         //7  中间位置最右方
        {1, -3, 0},         //8  敌方掩体右方靠后
        {-2, -1, 0},        //9  我方掩体右后方
        {1.5, 2, 0},        //10 敌方掩体左方
        {-2, 0, 0},         //11 我方掩体正后方
        {0, -2, 0},         //12 中间区域靠右
        {-3, 2, 0},         //13 我方领域靠左
        {-1, -2, 0},        //14 我方掩体右前方
        {-3, -1, 0},        //15 基地路口
        {2.5, 1, 0},        //16 敌方掩体左前方
        {-2, -2, 0},        //17 我方区域右出口

        {-3, -2, 0},        //18 我方基地左前
        {-3.5, -3.5, 0},    //19 我方基地右前
        {-5.5, -3.5, 0},    //20 我方基地右后
        {-5.5, -1.5, 0},    //21 我方基地左后
        {-5.5, 3.5, 0},     //22 我方补给区
        {-3, -0.5, 0},      //23 我方掩体右后
        {-3, -1.5, 0},      //24 我方掩体左后
        {-1, -2, 0},        //25 我方掩体右
        {-1, 2.5, 0},       //26 我方掩体左
        {0, -3, 0},         //27 中间右方
        {0.5, 3, 0},        //28 中间左方
        {1.5, -3, 0},       //29 敌方掩体右
        {1.5, 2, 0},        //30 敌方掩体左
        {3, -3, 0},         //31 敌方区域右
        {2.5, -0.5, 0},     //32 敌方掩体前
        {3.5, 2, 0},        //33 敌方基地前
        {0, 0, 0}           //34 中心
    };
    geometry_msgs::PoseStamped Area_Enemy_Support() { return PointMsg(Point[0]); }   //敌方补给区前
    geometry_msgs::PoseStamped Area_BiL() { return PointMsg(Point[1]); }   //增益区左边缘
    geometry_msgs::PoseStamped Area_My_Support() { return PointMsg(Point[2]); }   //我方补给区
    geometry_msgs::PoseStamped Area_My_Barrier_BL() { return PointMsg(Point[3]); }   //我方掩体左后方
    geometry_msgs::PoseStamped Area_Enemy_Barrier_R() { return PointMsg(Point[4]); }   //敌方掩体右方
    geometry_msgs::PoseStamped Area_Middle_Leftest() { return PointMsg(Point[5]); }   //中间位置最左方
    geometry_msgs::PoseStamped Area_My_Domain() { return PointMsg(Point[6]); }   //我方掩体后方靠左领域
    geometry_msgs::PoseStamped Area_Middle_Rightest() { return PointMsg(Point[7]); }   //中间位置最右方
    geometry_msgs::PoseStamped Area_Enemy_Barrier_BR() { return PointMsg(Point[8]); }   //敌方掩体右后方
    geometry_msgs::PoseStamped Area_My_Barrier_BR(){return PointMsg(Point[9]);}     //我方掩体右后方
    geometry_msgs::PoseStamped Area_Enemy_Barrier_L(){return PointMsg(Point[10]);}     //敌方掩体左方
    geometry_msgs::PoseStamped Area_My_Barrier_B(){return PointMsg(Point[11]);}     //我方掩体正后方
    geometry_msgs::PoseStamped Area_Middle_Right(){return PointMsg(Point[12]);}     //中间区域靠右
    geometry_msgs::PoseStamped Area_My_Domain_Left(){return PointMsg(Point[13]);}     //我方领域靠左
    geometry_msgs::PoseStamped Area_My_Barrier_FR(){return PointMsg(Point[14]);}     //我方掩体右前方
    geometry_msgs::PoseStamped Area_Base_Entry(){return PointMsg(Point[15]);}     //基地入口
    geometry_msgs::PoseStamped Area_Enemy_Barrier_FL(){return PointMsg(Point[16]);}     //敌方掩体左前方
    geometry_msgs::PoseStamped Area_My_Domain_Entry_R(){return PointMsg(Point[17]);}     //我方区域右出口

    geometry_msgs::PoseStamped Area_Current_Position(){double pos[3] = {x, y, 0}; return PointMsg(pos);}     //当前位置
    geometry_msgs::PoseStamped Posi_My_Base_LF(){return PointMsg(Point[18]);}     //我方基地左前
    geometry_msgs::PoseStamped Posi_My_Base_RF(){return PointMsg(Point[19]);}     //我方基地右前
    geometry_msgs::PoseStamped Posi_My_Base_RB(){return PointMsg(Point[20]);}     //我方基地右后
    geometry_msgs::PoseStamped Posi_My_Base_LB(){return PointMsg(Point[21]);}     //我方基地左后
    geometry_msgs::PoseStamped Posi_My_Support(){return PointMsg(Point[22]);}     //我方补给区
    geometry_msgs::PoseStamped Posi_My_Barrier_RB(){return PointMsg(Point[23]);}     //我方掩体右后
    geometry_msgs::PoseStamped Posi_My_Barrier_LB(){return PointMsg(Point[24]);}     //我方掩体左后
    geometry_msgs::PoseStamped Posi_My_Barrier_R(){return PointMsg(Point[25]);}     //我方掩体后
    geometry_msgs::PoseStamped Posi_My_Barrier_L(){return PointMsg(Point[26]);}     //我方掩体左
    geometry_msgs::PoseStamped Posi_Middle_R(){return PointMsg(Point[27]);}     //中间右方
    geometry_msgs::PoseStamped Posi_Middle_L(){return PointMsg(Point[28]);}     //中间左方
    geometry_msgs::PoseStamped Posi_Enemy_Barrier_R(){return PointMsg(Point[29]);}     //敌方掩体右
    geometry_msgs::PoseStamped Posi_Enemy_Barrier_L(){return PointMsg(Point[30]);}     //敌方掩体左
    geometry_msgs::PoseStamped Posi_Enemy_Domain_R(){return PointMsg(Point[31]);}     //敌方区域右
    geometry_msgs::PoseStamped Posi_Enemy_Barrier_F(){return PointMsg(Point[32]);}     //敌方掩体前
    geometry_msgs::PoseStamped Posi_Enemy_Base(){return PointMsg(Point[33]);}     //敌方基地前
    geometry_msgs::PoseStamped Posi_Middle_M(){return PointMsg(Point[34]);}     //中心


    void Update();
    geometry_msgs::PoseStamped PointMsg(double p[3]);                                    //将坐标转换为指定消息类型 方向始终向x轴正方向
    bool NearPoint(int n, double t = 0.2) {return (Point[n][0]-x)*(Point[n][0]-x) + (Point[n][1]-y)*(Point[n][1]-y) <= t * t;}
    bool InMyArea(double x, double y) {return x <= -1.5;}
    bool InMidArea(double x, double y) {return x > -1.5 && x < 1.5;}
    bool InEnemyArea(double x, double y) {return x >= 1.5;}
    bool InLeftArea(double x, double y) {return y >= 0;}
    bool InRightArea(double x, double y) {return y <= 0;}
    geometry_msgs::PoseStamped TrackNenmy(int n) {return Area_BiL();}
    int StateScore();

    bool Enemy_Guard_At_Left();                         //敌方哨兵出现在增左
    bool Attack_Guard_At_BiL();                         //在增左攻击敌方哨兵5s
    bool Is_Enemy_Guard_Retrat_From_BiL();              //判断敌方哨兵是否从增左撤退
    bool My_Guard_Hert();                               //我方哨兵在残血
    bool Inferior_At_Begining();                        //我方在开局劣势
    bool Leave_BiL();                                   //判断是否需要离开增左
    bool Low_Efficiency(int t = 20);                    //在增左攻击效率低
    bool Back_And_Recover_Adversely();                  //从增益区左方返回补给区 并等待恢复(处于劣势状态)
    bool Inferior_Measure_At_BiL();                     //在增左劣势的解决措施
    bool Back_From_BiL_To_My_Barrier();                 //从增左退回到我方掩体后方
    bool Enemy_Guard_Died_At_BiL();                     //敌方哨兵在增左阵亡
    bool Advance_To_Enemy_Support();                    //前进到敌方补给区前
    bool Back_And_Recover_Favourably();                 //从增益区回到补给区 并等待恢复(处于优势地位)
    bool Enemy_Guard_Disappear_At_BL();                 //敌方哨兵消失
    bool Enemy_Guard_In_My_Domain();                    //敌方哨兵在我方区域
    bool Enemy_Guard_In_Enemy_Domain();                 //敌方哨兵在敌方区域
    bool Enemy_Guard_In_Unknow_Domain();                //敌方哨兵位置未知
    bool Enemy_Guard_Retrat_From_BiL();                 //敌方哨兵从增左撤退
    bool Track_Enemy_Guard_At_BL();                     //跟踪敌方哨兵
    bool Advance_To_Enemy_Barrier_BL();                 //其方敌方掩体后方干扰

    bool Enemy_Guard_In_Front();                        //敌方哨兵出现在前方
    bool Attack_Guard_At_MidR();                        //在中右攻击敌方哨兵5s
    bool Leave_MidR();                                  //离开中间最右方
    bool Inferior_Measure_At_MidR();                    //在中右劣势的措施
    bool Back_From_MidR_To_My_Barrier();                //从中右退回到我方掩体后方
    bool Enemy_Guard_Died_At_MidR();                    //敌方哨兵在中右阵亡
    bool Enemy_Guard_Disappear_At_MidR();               //敌方哨兵消失
    bool Advance_To_Enemy_Barrier_R();                  //前进到敌方掩体右方
    bool Enemy_Guard_Advance();                         //敌方哨兵向前推进
    bool Track_Enemy_Guard_At_MidR();                   //跟踪敌方哨兵
    bool Is_Enemy_Guard_Retrat_From_MidR();             //判断敌方哨兵从中右撤退
    bool Enemy_Guard_Retrat_From_MidR();                //敌方哨兵从中右撤退

    bool Start_And_Advance_To_Enemy_Support();          //开局前往敌方补给区前
    bool Attack_Guard_At_EneSup();                      //在敌方补给区前攻击敌方5s
    bool Leave_EneSup();                                //离开敌方补给区
    bool Inferior_Measure_At_EneSup();                  //在敌方补给区前劣势的措施
    bool Back_From_EneSup_To_My_BarrierB();             //从敌方补给区前返回到我方掩体正后方
    bool Enemy_Guard_Died_At_EneSup();                  //敌方哨兵阵亡 在敌方补给区前
    bool Enemy_Guard_Disappear_At_EneSup();             //敌方哨兵在敌方补给区前消失
    bool Back_To_My_DomainL_By_Right();                 //从右边返回我方区域左方
    bool Find_Enemy_At_Middle_Right();                  //在中间右方发现敌方
    bool Leave_Middle_Right();                          //离开中间靠右区域
    bool Back_To_Base_Enter();                          //返回基地入口
    bool Leave_Base_Entry();                            //离开基地入口
    bool Find_Enemy_At_Middle_Left();                   //在中间左边发现敌方
    bool Leave_Middle_Leftest();                        //离开中间左方
    bool Find_Enemy_At_My_Domain_Left();                //在我方区域左方发现敌方
    bool Advance_To_My_Barrier_FR();                    //前往我方掩体右前方
    bool Advance_To_Enemy_Barrier_FL();                 //前进到敌方掩体左前方
    bool Leave_Enemy_Barrier_FL();                      //离开敌方掩体左前方
    bool Inferior_Measure_At_Enemy_Barrier_FL();        //在敌方掩体左前方劣势的措施
    bool Enemy_Guard_Died_At_Enemy_Barrier_FL();        //敌方哨兵阵亡 在敌方掩体左前方
    bool Back_To_My_Domain_Entry_R();                   //返回我方区域右出口
    bool Enemy_Ally_Died_And_Stay();                    //敌方步兵均阵亡

    bool Inferior_Situation();                          //劣势
    bool Favorable_Situation();                         //优势
    int Ally_Posi(int x);                               //友军位置
    int Enemy_Posi(int x);                              //敌方位置

    bool Base_Been_Attacker();                          //基地被攻击
    bool Return_To_Base_LF();                           //前往基地左前方
    bool Leave_Base_LF();                               //离开基地左前方
    bool Return_To_Base_RF();                           //前往基地右前方
    bool Leave_Base_RF();                               //离开基地右前方
    bool Return_To_Base_RB();                           //前往基地右后方
    bool Leave_Base_RB();                               //离开基地右后方
    bool Return_To_Base_LB();                           //前往基地左后方
    bool Leave_Base_LB();                               //离开基地左后方

    bool Return_To_Recover();                           //回补给区恢复

    bool Move_To_Barrier_RB();                          //前往掩体右后
    bool Leave_Barrier_RB();                            //离开掩体右后

    bool Move_To_Barrier_LB();                          //前往掩体左后
    bool Leave_Barrier_LB();                            //离开掩体左后

    bool Move_To_Barrier_R();                           //前往掩体右
    bool Leave_Barrier_R();                             //离开掩体右

    bool Move_To_Barrier_L();                           //前往掩体左
    bool Leave_Barrier_L();                             //离开掩体左

    bool Move_To_Middle_R();                            //前往中间右
    bool Leave_Middle_R();                              //离开中间右

    bool Move_To_Middle_L();                            //前往中间左
    bool Leave_Middle_L();                              //离开中间左

    bool Move_To_Middle_M();                            //前往中间中
    bool Leave_Middle_M();                              //前往中间中

    bool Move_To_Enemy_Barrier_R();                     //前往敌方掩体右
    bool Leave_Enemy_Barrier_R();                       //离开敌方掩体右

    bool Move_To_Enemy_Barrier_L();                     //前往敌方掩体左
    bool Leave_Enemy_Barrier_L();                       //离开敌方掩体左

    bool Inspect_Enemy_R_F();                           //敌方区域右方巡视 向前
    bool Inspect_Enemy_R_B();                           //敌方区域右方巡视 向后

    bool Move_TO_Enemy_Barrier_F();                     //前往敌方掩体前
    bool Leave_Enemy_Barrier_F();                       //离开敌方掩体前

    bool Move_To_Enemy_Domain_R();                      //前往敌方区域右
    bool Leave_Enemy_Domain_R();                        //离开敌方区域右

    bool Move_To_Enemy_Base();                          //前往敌方基地
    bool Leave_Enemy_Base();                            //离开敌方基地

public:
    geometry_msgs::PoseStamped DecisionAction(const referee_system::referee_system& msg, double x, double y);

};

void DecisionMaker::Update()
{
    enemy_live = 0;
    my_live = 0;
    if (type == 0 || type == 3)
    {
        enemy_blood_decrease = (recent_referee[0].front().enemy_robot3 + recent_referee[0].front().enemy_robot4 + recent_referee[0].front().enemy_guard_blood)
                                -(recent_referee[0].back().enemy_robot3 + recent_referee[0].back().enemy_robot4 + recent_referee[0].back().enemy_guard_blood);
        enemy_blood_sum = recent_referee[0].back().enemy_robot3 + recent_referee[0].back().enemy_robot4 + recent_referee[0].back().enemy_guard_blood;
        if (recent_referee[0].back().enemy_robot3 > 0)
            enemy_live++;
        if (recent_referee[0].back().enemy_robot4 > 0)
            enemy_live++;
        if (recent_referee[0].back().enemy_guard_blood > 0)
            enemy_live++;
    }
    else
    {
        enemy_blood_decrease = (recent_referee[0].front().enemy_robot3 + recent_referee[0].front().enemy_robot1 + recent_referee[0].front().enemy_guard_blood)
                                -(recent_referee[0].back().enemy_robot3 + recent_referee[0].back().enemy_robot1 + recent_referee[0].back().enemy_guard_blood);
        enemy_blood_sum = recent_referee[0].back().enemy_robot3 + recent_referee[0].back().enemy_robot1 + recent_referee[0].back().enemy_guard_blood;
        if (recent_referee[0].back().enemy_robot3 > 0)
            enemy_live++;
        if (recent_referee[0].back().enemy_robot1 > 0)
            enemy_live++;
        if (recent_referee[0].back().enemy_guard_blood > 0)
            enemy_live++;
    }
    if (type == 0 || type == 1)
    {
        my_blood_decrease = (recent_referee[0].front().my_robot3 + recent_referee[0].front().my_robot4 + recent_referee[0].front().my_guard_blood)
                                -(recent_referee[0].back().my_robot3 + recent_referee[0].back().my_robot4 + recent_referee[0].back().my_guard_blood);
        my_shut = (recent_referee[0].front().my_robot3_bullet + recent_referee[0].front().my_robot4_bullet + recent_referee[0].front().my_guard_bullet)
                    - (recent_referee[0].back().my_robot3_bullet + recent_referee[0].back().my_robot4_bullet + recent_referee[0].back().my_guard_bullet);
        my_blood_sum = recent_referee[0].back().my_robot3 + recent_referee[0].back().my_robot4 + recent_referee[0].back().my_guard_blood;
        if (recent_referee[0].back().my_robot3 > 0)
            my_live++;
        if (recent_referee[0].back().my_robot4 > 0)
            my_live++;
        if (recent_referee[0].back().my_guard_blood > 0)
            my_live++;

        ally_blood_decrease = (recent_referee[0].front().my_robot3 + recent_referee[0].front().my_robot4)
                                -(recent_referee[0].back().my_robot3 + recent_referee[0].back().my_robot4);
        ally_shut = (recent_referee[0].front().my_robot3_bullet + recent_referee[0].front().my_robot4_bullet)
                    - (recent_referee[0].back().my_robot3_bullet + recent_referee[0].back().my_robot4_bullet);
        ally_blood = recent_referee[0].back().my_robot3 + recent_referee[0].back().my_robot4;
    }
    else
    {
        my_blood_decrease = (recent_referee[0].front().my_robot3 + recent_referee[0].front().my_robot1 + recent_referee[0].front().my_guard_blood)
                                -(recent_referee[0].back().my_robot3 + recent_referee[0].back().my_robot1 + recent_referee[0].back().my_guard_blood);
        my_shut = (recent_referee[0].front().my_robot3_bullet + recent_referee[0].front().my_robot1_bullet + recent_referee[0].front().my_guard_bullet)
                    - (recent_referee[0].back().my_robot3_bullet + recent_referee[0].back().my_robot1_bullet + recent_referee[0].back().my_guard_bullet);
        my_blood_sum = recent_referee[0].back().my_robot3 + recent_referee[0].back().my_robot1 + recent_referee[0].back().my_guard_blood;
        if (recent_referee[0].back().my_robot3 > 0)
            my_live++;
        if (recent_referee[0].back().my_robot1 > 0)
            my_live++;
        if (recent_referee[0].back().my_guard_blood > 0)
            my_live++;

        ally_blood_decrease = (recent_referee[0].front().my_robot3 + recent_referee[0].front().my_robot1)
                                -(recent_referee[0].back().my_robot3 + recent_referee[0].back().my_robot1);
        ally_shut = (recent_referee[0].front().my_robot3_bullet + recent_referee[0].front().my_robot1_bullet)
                    - (recent_referee[0].back().my_robot3_bullet + recent_referee[0].back().my_robot1_bullet);
        ally_blood = recent_referee[0].back().my_robot3 + recent_referee[0].back().my_robot1;
    }
    ally_live = my_live - 1;
    enemy_ally_live = recent_referee[0].back().enemy_guard_blood == 0 ? enemy_live : enemy_live - 1;
    enemy_ally_blood = enemy_blood_sum - recent_referee[0].back().enemy_guard_blood;
    enemy_ally_blood_decrease = enemy_blood_decrease - (recent_referee[0].front().enemy_guard_blood - recent_referee[0].back().enemy_guard_blood);
}

//将坐标转换为指定消息类型 方向始终向x轴正方向
geometry_msgs::PoseStamped DecisionMaker::PointMsg(double p[3])
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.pose.position.x = p[0];
    msg.pose.position.y = p[1];
    msg.pose.position.z = p[2];

    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    msg.pose.orientation.x = q.getX();
    msg.pose.orientation.y = q.getY();
    msg.pose.orientation.z = q.getZ();
    msg.pose.orientation.w = q.getW();
    return msg;
}

int DecisionMaker::StateScore()
{

}

/*------------------------------------------开局敌方哨兵出现在左方----------------------------------------*/
/*------------------------------------------开局敌方哨兵出现在左方----------------------------------------*/

//敌方哨兵从增益区左方撤退
bool DecisionMaker::Is_Enemy_Guard_Retrat_From_BiL()
{
    if (enemy_guard.back().robot_posi[0] - enemy_guard.front().robot_posi[0] > 1.5)
        return true;
    if (enemy_guard.back().robot_posi[0] >= 1.5)
        return true;
    return false;
}

//增益区左方我方哨兵残血
bool DecisionMaker::My_Guard_Hert()
{
    if (recent_referee[0].back().my_guard_blood <= 100)
        return true;
    return false;
}

//我方呈劣势
bool DecisionMaker::Inferior_At_Begining()
{
    //我方盟友血量和低于100 敌方盟友血量和大于250
    if (ally_blood < 100 && enemy_ally_blood > 250)
        return true;
    //我方盟友存活数少于敌方 我方盟友血量减少比敌方减少大于30
    if (ally_live < enemy_ally_live && ally_blood_decrease - enemy_ally_blood_decrease > 30)
        return true;
    //我方盟友存活数少于敌方 敌方血量为我方两倍
    if (ally_live < enemy_ally_live && enemy_ally_blood / double(ally_blood+1) > 2)
        return true;
    //敌方哨兵血量比我方多300 我方哨兵血量低于200 我方盟友血量低于敌方
    if (recent_referee[0].back().enemy_guard_blood - recent_referee[0].back().my_guard_blood > 300 && recent_referee[0].back().my_guard_blood < 200 && ally_blood < enemy_ally_blood)
        return true;
    return false;
}

//攻击效率低
bool DecisionMaker::Low_Efficiency(int t)
{
    return recent_referee[0].front().my_guard_bullet - recent_referee[0].back().my_guard_bullet > t;
}

//如果敌方哨兵在增益区左方干扰我方  前往增益区左方
bool DecisionMaker::Enemy_Guard_At_Left()
{
    if (instruction != 1 && instruction != 2)
        return false;
    //敌方在增左干扰我方
    if (!enemy_guard.empty() && enemy_guard.back().robot_posi[1] >= 0 && enemy_guard.back().robot_posi[0] < 1.5)
        instruction = 2;
    if (instruction != 2)
        return false;
    //没有到达增左
    if (instruction == 2 && !NearPoint(1))
    {
        return true;
    }
    //到达
    else
    {
        instruction = 3;
        return false;
    }
}

//增益区左方停留5秒
bool DecisionMaker::Attack_Guard_At_BiL()
{
    if (instruction != 3)
        return false;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    else
    {
        time = -1;
        instruction = 4;
        return false;
    }
}

//发生情况 离开增益区左方
bool DecisionMaker::Leave_BiL()
{
    if (instruction != 4)
        return false;
    //我方哨兵残血
    if (My_Guard_Hert())
    {
        instruction = 5;
        return true;
    }
    //我方劣势
    else if (Inferior_At_Begining())
    {
        instruction = 6;
        return true;
    }
    //敌方哨兵阵亡
    else if (recent_referee[0].back().enemy_guard_blood == 0)
    {
        instruction = 7;
        return true;
    }
    //敌方哨兵消失
    else if (enemy_guard.empty())
    {
        instruction = 8;
        return true;
    }
    //敌方哨兵撤退
    else if (Is_Enemy_Guard_Retrat_From_BiL())
    {
        instruction = 9;
        return true;
    }
    // //攻击效率低
    // else if (Low_Efficiency())
    // {
    //     instruction = 10;
    // }
    return false;
}

//从增益区左边撤退并等待恢复
bool DecisionMaker::Back_And_Recover_Adversely()
{
    if (instruction != 5)
        return false;
    if (type == 0 || type == 1)
    {
        if (((recent_referee[0].back().my_robot3 > 0 && recent_referee[0].back().my_robot3 <= 50)
            || (recent_referee[0].back().my_robot4 > 0 && recent_referee[0].back().my_robot4 <= 50))
            && recent_referee[0].back().my_guard_blood <= 350)
            return true;
        else if (recent_referee[0].back().my_guard_blood <= 550)
            return true;
    }
    else
    {
        if (((recent_referee[0].back().my_robot3 > 0 && recent_referee[0].back().my_robot3 <= 50)
            || (recent_referee[0].back().my_robot1 > 0 && recent_referee[0].back().my_robot1 <= 50))
            && recent_referee[0].back().my_guard_blood <= 350)
            return true;
        else if (recent_referee[0].back().my_guard_blood <= 550)
            return true;
    }
    instruction = 1001;
    return false;
}

//在增左劣势的解决措施
bool DecisionMaker::Inferior_Measure_At_BiL()
{
    if (instruction != 6)
        return false;
    //队友阵亡 撤退
    if (my_live - 1 == 0)
    {
        instruction = 11;
        return false;
    }
    //队友不在攻击 撤退
    if (type == 0 || type == 1)
    {
        //友军攻击频率低
        if (ally_shut / (my_live - 1) < 10)
        {
            instruction = 11;
            return false;
        }
    }
    else
    {
        //我方3号步兵阵亡 或友方攻击频率低
        if (recent_referee[0].back().my_robot3 == 0 || ally_shut < 10)
        {
            instruction = 11;
            return false;
        }
    }
    //队友仍在战斗 留在原地协助
    instruction = 4;
    return true;
}

//从增左退回到我方掩体左后方
bool DecisionMaker::Back_From_BiL_To_My_Barrier()
{
    if (instruction != 11)
        return false;
    if (instruction == 11 && NearPoint(3))
    {
        instruction = 1002;
        return false;
    }
    return true;
}

//敌方哨兵在增益区左方阵亡
bool DecisionMaker::Enemy_Guard_Died_At_BiL()
{
    if (instruction != 7)
        return false;
    int ave  = enemy_live == 0 ? 0 : my_blood_decrease / enemy_live;
    //敌方攻击弱 哨兵状态好 前往敌方掩体右方
    if (ave < 30 && recent_referee[0].back().my_guard_blood >= 150)
    {
        instruction = 12;
        return true;
    }
    //敌方攻击弱 哨兵状态不好 回补给区
    else if (ave < 30)
    {
        instruction = 13;
        return true;
    }
    //敌方还在攻击 原地迎战
    else
    {
        instruction = 4;
        return true;
    }
}

//敌方哨兵阵亡 前进到敌方掩体右方
bool DecisionMaker::Advance_To_Enemy_Support()
{
    if (instruction != 12)
        return false;
    if (!NearPoint(0))
        return true;
    else
    {
        instruction = 1003;
        return false;
    }
}

//敌方哨兵阵亡 敌方攻击弱 哨兵状态不好 回补给区 等待恢复完成
bool DecisionMaker::Back_And_Recover_Favourably()
{
    if (instruction != 13 && instruction != 18)
        return false;
    if (type == 0 || type == 1)
    {
        if (((recent_referee[0].back().my_robot3 > 0 && recent_referee[0].back().my_robot3 <= 50)
            || (recent_referee[0].back().my_robot4 > 0 && recent_referee[0].back().my_robot4 <= 50))
            && recent_referee[0].back().my_guard_blood <= 350)
            return true;
        else if (recent_referee[0].back().my_guard_blood <= 550)
            return true;
    }
    else
    {
        if (((recent_referee[0].back().my_robot3 > 0 && recent_referee[0].back().my_robot3 <= 50)
            || (recent_referee[0].back().my_robot1 > 0 && recent_referee[0].back().my_robot1 <= 50))
            && recent_referee[0].back().my_guard_blood <= 350)
            return true;
        else if (recent_referee[0].back().my_guard_blood <= 550)
            return true;
    }
    instruction = 1004;
    return false;
}

//敌方哨兵消失  向左前进寻找
bool DecisionMaker::Enemy_Guard_Disappear_At_BL()
{
    if (instruction != 8)
        return false;
    if (!NearPoint(5))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    //到达最左方 停留3s观察
    if (time - recent_referee[0].back().remaining_time <= 3)
        return true;
    else
    {
        time = -1;
        //敌方哨兵出现在我方区域
        if (!enemy_guard.empty() && enemy_guard.back().robot_posi[0] < -1.5)
        {
            instruction = 14;
            return false;
        }
        //敌方哨兵出现在敌方区域
        if (!enemy_guard.empty() && enemy_guard.back().robot_posi[0] > 1.5)
        {
            instruction = 15;
            return false;
        }
        //敌方哨兵在未知区域
        else
        {
            instruction = 16;
            return false;
        }
    }
}

//敌方哨兵在我方区域
bool DecisionMaker::Enemy_Guard_In_My_Domain()
{
    if (instruction != 14)
        return false;
    if (!NearPoint(6))
        return true;
    instruction = 1005;
    return false;
}

//敌方哨兵在敌方区域
bool DecisionMaker::Enemy_Guard_In_Enemy_Domain()
{
    if (instruction != 15)
        return false;
    return true;
}

//敌方哨兵位置未知
bool DecisionMaker::Enemy_Guard_In_Unknow_Domain()
{
    if (instruction != 16)
        return false;
    //返回补给区
    if (recent_referee[0].back().my_guard_blood < 200)
    {
        instruction = 13;
        return false;
    }
    //去敌方掩体右方
    if (!NearPoint(4))
        return true;
    instruction = 1006;
    return false;
}

//敌方哨兵从增左撤退
bool DecisionMaker::Enemy_Guard_Retrat_From_BiL()
{
    if (instruction != 9)
        return false;
    //跟踪敌方哨兵
    if (recent_referee[0].back().enemy_guard_blood - recent_referee[0].back().my_guard_blood < 150
        && recent_referee[0].back().enemy_guard_blood / double(recent_referee[0].back().my_guard_blood) < 2
        && (my_blood_sum - recent_referee[0].back().my_guard_blood) - (enemy_blood_sum - recent_referee[0].back().enemy_guard_blood) > -100)
    {
        instruction = 17;
        return true;
    }
    //回补给区
    if (recent_referee[0].back().my_guard_blood < 200)
    {
        instruction = 18;
        return true;
    }
    //去干扰敌方步兵
    else
    {
        instruction = 19;
        return true;
    }
}

//跟踪敌方哨兵
bool DecisionMaker::Track_Enemy_Guard_At_BL()
{
    if (instruction != 17)
        return false;
}

//去敌方掩体右方干扰
bool DecisionMaker::Advance_To_Enemy_Barrier_BL()
{
    if (instruction != 19)
        return false;
    if (!NearPoint(4))
    {
        return true;
    }
    instruction = 1006;
    return false;
}


/*------------------------------------------开局敌方哨兵出现在前方----------------------------------------*/
/*------------------------------------------开局敌方哨兵出现在前方----------------------------------------*/


//敌方哨兵出现在敌方补给区 前往中心最左方或原地不动
bool DecisionMaker::Enemy_Guard_In_Front()
{
    if (instruction != 1 && instruction != 20)
        return false;
    //敌方哨兵出现在敌方补给区前方
    if (instruction == 1 && !enemy_guard.empty() && enemy_guard.back().robot_posi[1] <= -2 && enemy_guard.back().robot_posi[0] > 0)
    {
        instruction = 20;
    }
    if (instruction != 20)
        return false;
    //我方哨兵位置过半
    if (x > 0)
    {
        instruction = 21;
        return false;
    }
    //未过半
    if (!NearPoint(7))
    {
        return true;
    }
    else
    {
        instruction = 21;
        return false;
    }
}

//在中右攻击敌方哨兵5s
bool DecisionMaker::Attack_Guard_At_MidR()
{
    if (instruction != 21)
        return false;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    else
    {
        time = -1;
        instruction = 22;
        return false;
    }
}

//敌方哨兵向前推进
bool DecisionMaker::Enemy_Guard_Advance()
{
    if (enemy_guard.back().robot_posi[0] - enemy_guard.front().robot_posi[0] < -1.5)
        return true;
    if (enemy_guard.back().robot_posi[0] < 0)
        return true;
    if (enemy_guard.back().robot_posi[1] > -1)
        return true;
    return false;
}                   

//敌方哨兵从中右撤退
bool DecisionMaker::Is_Enemy_Guard_Retrat_From_MidR()
{
    if (enemy_guard.back().robot_posi[0] > 3)
        return true;
    if (enemy_guard.back().robot_posi[1] > -2)
        return true;
    if (enemy_guard.back().robot_posi[0] - enemy_guard.front().robot_posi[0] > 1.5)
        return true;
    if (enemy_guard.back().robot_posi[1] - enemy_guard.front().robot_posi[1] > 1 && enemy_guard.back().robot_posi[0] > 1.5)
        return true;
    return false;
}            

//离开中间最右方
bool DecisionMaker::Leave_MidR()
{
    if (instruction != 22)
        return false;
    //我方哨兵残血
    if (My_Guard_Hert())
    {
        instruction = 5;
        return true;
    }
    //我方劣势
    else if (Inferior_At_Begining())
    {
        instruction = 23;
        return true;
    }
    //敌方哨兵阵亡
    else if (recent_referee[0].back().enemy_guard_blood == 0)
    {
        instruction = 24;
        return true;
    }
    //敌方哨兵消失
    else if (enemy_guard.empty())
    {
        instruction = 25;
        return true;
    }
    //敌方哨兵前进
    else if (Enemy_Guard_Advance())
    {
        instruction = 26;
        return true;
    }
    //敌方哨兵撤退
    else if (Is_Enemy_Guard_Retrat_From_MidR())
    {
        instruction = 27;
        return true;
    }
    // //攻击效率低
    // else if (Low_Efficiency())
    // {
    //     instruction = 28;
    // }
    return false;
}

//在中右劣势的解决措施
bool DecisionMaker::Inferior_Measure_At_MidR()
{
    if (instruction != 23)
        return false;
    //队友阵亡 撤退
    if (my_live - 1 == 0)
    {
        instruction = 29;
        return false;
    }
    //队友不在攻击 撤退
    if (type == 0 || type == 1)
    {
        //友军攻击频率低
        if (ally_shut / (my_live - 1) < 10)
        {
            instruction = 29;
            return false;
        }
    }
    else
    {
        //我方3号步兵阵亡 或友方攻击频率低
        if (recent_referee[0].back().my_robot3 == 0 || ally_shut < 10)
        {
            instruction = 29;
            return false;
        }
    }
    //队友仍在战斗 留在原地协助
    instruction = 22;
    return true;
}

//从中右退回到我方掩体后方
bool DecisionMaker::Back_From_MidR_To_My_Barrier()               
{
    if (instruction != 29)
        return false;
    if (!NearPoint(9))
        return true;
    instruction = 1007;
    return false;
}

//敌方哨兵在中右阵亡
bool DecisionMaker::Enemy_Guard_Died_At_MidR()
{
    if (instruction != 24)
        return false;
    int ave  = enemy_live == 0 ? 0 : my_blood_decrease / enemy_live;
    //敌方攻击弱 哨兵状态好 前往敌方补给区前
    if (ave < 30 && recent_referee[0].back().my_guard_blood >= 150)
    {
        instruction = 12;
        return true;
    }
    //敌方攻击弱 哨兵状态不好 回补给区
    else if (ave < 30)
    {
        instruction = 13;
        return true;
    }
    //敌方还在攻击 原地迎战
    else
    {
        instruction = 22;
        return true;
    }
}

//敌方哨兵在中右消失
bool DecisionMaker::Enemy_Guard_Disappear_At_MidR()
{
    if (instruction != 25)
        return false;
    //哨兵血量不足
    if (recent_referee[0].back().my_guard_blood < 150)
    {
        instruction = 13;
        return false;
    }
    //前往敌方掩体右方
    instruction = 30;
    return false;
}              

//前进到敌方掩体右方
bool DecisionMaker::Advance_To_Enemy_Barrier_R()  
{
    if (instruction != 30)
        return false;
    if (!NearPoint(4))
        return true;
    instruction = 1008;
    return false;
}                

//跟踪敌方哨兵
bool DecisionMaker::Track_Enemy_Guard_At_MidR()
{
    if (instruction != 26)
        return false;
    return false;
}                   

//敌方哨兵从中右撤退
bool DecisionMaker::Enemy_Guard_Retrat_From_MidR()
{
    if (instruction != 27)
        return false;
    //回补给区
    if (recent_referee[0].back().my_guard_blood < 150)
    {
        if ((my_blood_sum - recent_referee[0].back().my_guard_blood) - (enemy_blood_sum - recent_referee[0].back().enemy_guard_blood) > 100)
        {
            instruction = 13;
            return false;
        }
        else
        {
            instruction = 5;
            return false;
        }
    }
    //前往敌方补给区前
    if (recent_referee[0].back().enemy_guard_blood - recent_referee[0].back().my_guard_blood < 150
        && recent_referee[0].back().enemy_guard_blood / double(recent_referee[0].back().my_guard_blood) < 2
        && (my_blood_sum - recent_referee[0].back().my_guard_blood) - (enemy_blood_sum - recent_referee[0].back().enemy_guard_blood) > -100)
    {
        instruction = 12;
        return false;
    }
    return false;
}               


/*------------------------------------------开局前往敌方补给区----------------------------------------*/
/*------------------------------------------开局前往敌方补给区----------------------------------------*/

//开局前往敌方补给区前
bool DecisionMaker::Start_And_Advance_To_Enemy_Support()
{
    if (instruction != 1)
        return false;
    if (!NearPoint(0))
        return true;
    instruction = 31;
    return false;
}          

//在敌方补给区前攻击敌方5s
bool DecisionMaker::Attack_Guard_At_EneSup()
{
    if (instruction != 31)
        return false;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    time = -1;
    instruction = 32;
    return false;
}                       

//离开敌方补给区
bool DecisionMaker::Leave_EneSup()
{
    if (instruction != 32)
        return false;
    //哨兵残血
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 13;
        return true;
    }
    //我方劣势
    if (Inferior_At_Begining())
    {
        instruction = 33;
        return true;
    }
    //敌方哨兵阵亡
    if (recent_referee[0].back().enemy_guard_blood == 0)
    {
        instruction = 34;
        return true;
    }
    //未发现敌方哨兵
    if (enemy_guard.empty())
    {
        instruction = 35;
        return true;
    }
    //敌方步兵阵亡
    if (enemy_ally_live == 0)
    {
        instruction = 36;
        return true;
    }
    // //攻击效率低
    // if (Low_Efficiency())
    // {
    //     instruction = 37;
    //     return true;
    // }
    return false;
}

//在敌方补给区前劣势的措施
bool DecisionMaker::Inferior_Measure_At_EneSup()
{
    if (instruction != 33)
        return false;
    //队友阵亡 撤退
    if (my_live - 1 == 0)
    {
        instruction = 38;
        return false;
    }
    //队友不在攻击 撤退
    if (type == 0 || type == 1)
    {
        //友军攻击频率低
        if (ally_shut / (my_live - 1) < 10)
        {
            instruction = 38;
            return false;
        }
    }
    else
    {
        //我方3号步兵阵亡 或友方攻击频率低
        if (recent_referee[0].back().my_robot3 == 0 || ally_shut < 10)
        {
            instruction = 38;
            return false;
        }
    }
    //队友仍在战斗 留在原地协助
    instruction = 32;
    return true;
}                

//从敌方补给区前返回到我方掩体正后方
bool DecisionMaker::Back_From_EneSup_To_My_BarrierB()
{
    if (instruction != 38)
        return false;
    if (!NearPoint(11))
        return true;
    instruction = 1009;
    return false;
}             

//敌方哨兵阵亡 在敌方补给区前
bool DecisionMaker::Enemy_Guard_Died_At_EneSup()
{
    if (instruction != 34)
        return false;
    //哨兵状态良好 进入全局模式
    if (recent_referee[0].back().my_guard_blood >= 150)
    {
        instruction = 1003;
        return true;
    }
    //哨兵状态不好 返回补给区
    else
    {
        instruction = 13;
        return true;
    }
}                  

//敌方哨兵在敌方补给区前消失
bool DecisionMaker::Enemy_Guard_Disappear_At_EneSup()
{
    if (instruction != 35)
        return false;
    //我方哨兵攻击慢 盟友状态不好 返回我方区域
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet <= 20 && ally_blood < 100)
    {
        instruction = 39;
        return true;
    }
    //我方哨攻击过慢 侦察一圈
    else if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet <= 5)
    {
        instruction = 46;
        return true;
    }
    //原地攻击
    instruction = 32;
    return true;
}             

//从右边返回我方区域左方
bool DecisionMaker::Back_To_My_DomainL_By_Right()
{
    if (instruction != 39 && instruction != 40 && instruction != 41)
        return false;
    //前往中间靠右
    if (instruction == 39 && !NearPoint(12))
        return true;
    else if (instruction == 39)
    {
        //发现敌人 哨兵攻击强
        if (recent_referee[0].front().my_guard_bullet - recent_referee[0].back().my_guard_bullet > 6)
        {
            instruction = 42;
            return false;
        }
        //敌方哨兵在中间靠右
        if (!enemy_guard.empty() && enemy_guard.back().robot_posi[0] < 1.5 && enemy_guard.back().robot_posi[1] < 1 && enemy_guard.back().robot_posi[0] > -1.5)
        {
            instruction = 42;
            return false;
        }
        //敌方哨兵在我方靠右
        if (!enemy_guard.empty() && enemy_guard.back().robot_posi[0] < 1.5 && enemy_guard.back().robot_posi[1] < 1 && enemy_guard.back().robot_posi[0] <= -1.5)
        {
            instruction = 43;
            return false;
        }
        instruction = 40;
    }
    //前往中间最左方
    if (instruction == 40 && !NearPoint(5))
        return true;
    else if (instruction == 40)
    {
        //发现敌方哨兵
        if (!enemy_guard.empty())
        {
            instruction = 44;
            return false;
        }
        instruction = 41;
    }
    //前往我方区域左方
    if (instruction == 41 && !NearPoint(13))
        return true;
    instruction = 45;
    return false;
}                 

//在中间右方发现敌方
bool DecisionMaker::Find_Enemy_At_Middle_Right()
{
    if (instruction != 42)
        return false;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    time = -1;
    instruction = 47;
    return false;
}   

//离开中间靠右区域
bool DecisionMaker::Leave_Middle_Right()
{
    if (instruction != 47)
        return false;
    //哨兵残血
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 5;
        return true;
    }
    //哨兵攻击较慢
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet < 15)
    {
        //劣势
        if (Inferior_At_Begining())
        {
            instruction = 38;
            return true;
        }
        //敌方哨兵阵亡
        if (recent_referee[0].back().enemy_guard_blood == 0)
        {
            //返回我方补给区
            if (recent_referee[0].back().my_guard_blood < 200)
            {
                instruction = 13;
                return true;
            }
            //前进到敌方补给区前
            else
            {
                instruction = 12;
                return true;
            }
        }
        //敌方哨兵未阵亡 前往我方区域左方
        else
        {
            instruction = 41;
            return true;
        }
    }
    return false;
}                          

//返回基地入口
bool DecisionMaker::Back_To_Base_Enter()
{
    if (instruction != 43)
        return false;
    if (!NearPoint(15))
        return true;
    instruction = 50;
    return false;
}                          

//离开基地入口
bool DecisionMaker::Leave_Base_Entry()
{
    if (instruction != 50)
        return false;
    //发现敌方 或 哨兵在攻击
    if (!enemy_guard.empty() || !enemy_r3.empty() || !enemy_r4.empty() || !enemy_r1.empty() || (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet > 10))
        return false;
    //返回补给区
    if (recent_referee[0].back().my_guard_blood < 200)
    {
        instruction = 5;
        return true;
    }
    //劣势 守在原地
    if (Inferior_At_Begining())
    {
        instruction = 1012;
        return true;
    }
    //前往我方掩体右前方
    else
    {
        instruction = 49;
        return true;
    }
}                            

//在中间左边发现敌方
bool DecisionMaker::Find_Enemy_At_Middle_Left()
{
    if (instruction != 44)
        return false;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    instruction = 48;
    time = -1;
    return false;
}                   

//离开中间左方
bool DecisionMaker::Leave_Middle_Leftest()
{
    if (instruction != 48)
        return false;
    //哨兵残血
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 5;
        return true;
    }
    //哨兵攻击较慢
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet < 15)
    {
        //劣势
        if (Inferior_At_Begining())
        {
            instruction = 38;
            return true;
        }
        //敌方哨兵阵亡
        if (recent_referee[0].back().enemy_guard_blood == 0)
        {
            //返回我方补给区
            if (recent_referee[0].back().my_guard_blood < 200)
            {
                instruction = 13;
                return true;
            }
            //前进到敌方补给区前
            else
            {
                instruction = 12;
                return true;
            }
        }
        //敌方哨兵未阵亡 前往我方区域左方
        else
        {
            instruction = 41;
            return true;
        }
    }
    return false;
}                        

//回到我方区域左方              
bool DecisionMaker::Find_Enemy_At_My_Domain_Left()
{
    if (instruction != 45)
        return false;
    //发现敌方 或 哨兵在攻击
    if (!enemy_guard.empty() || !enemy_r3.empty() || !enemy_r4.empty() || !enemy_r1.empty() || (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet > 10))
        return true;
    //返回补给区
    if (recent_referee[0].back().my_guard_blood < 200)
    {
        instruction = 5;
        return false;
    }
    //劣势 守在原地
    if (Inferior_At_Begining())
    {
        instruction = 1010;
        return false;
    }
    //前往我方掩体右前方
    else
    {
        instruction = 49;
        return false;
    }
}              

//前往我方掩体右前方
bool DecisionMaker::Advance_To_My_Barrier_FR()
{
    if (instruction != 49)
        return false;
    if (!NearPoint(14))
        return true;
    instruction = 1011;
    return false;
}                    

//前进到敌方掩体左前方
bool DecisionMaker::Advance_To_Enemy_Barrier_FL()
{
    if (instruction != 46)
        return false;
    if (!NearPoint(16))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    time = -1;
    instruction = 51;
    return false;
}                 

//离开敌方掩体左前方
bool DecisionMaker::Leave_Enemy_Barrier_FL()
{
    if (instruction != 51)
        return false;
    //哨兵血量不足
    if (recent_referee[0].back().my_guard_blood <= 100)
    {
        instruction = 5;
        return true;
    }
    //哨兵攻击强
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 20)
        return false;
    //劣势
    if (Inferior_At_Begining())
    {
        instruction = 52;
        return true;
    }
    //敌方哨兵阵亡
    if (recent_referee[0].back().enemy_guard_blood == 0)
    {
        instruction = 53;
        return true;
    }
    //血量充足 攻击效率低
    instruction = 54;
    return true;
}

//在敌方掩体左前方劣势的措施
bool DecisionMaker::Inferior_Measure_At_Enemy_Barrier_FL()
{
    if (instruction != 52)
        return false;
    if (!NearPoint(3))
        return true;
    instruction = 1002;
    return false;
}        

//敌方哨兵阵亡 在敌方掩体左前方
bool DecisionMaker::Enemy_Guard_Died_At_Enemy_Barrier_FL()
{
    if (instruction != 53)
        return false;
    if (!NearPoint(0))
        return true;
    instruction = 1003;
    return false;
}        

//返回我方区域右出口
bool DecisionMaker::Back_To_My_Domain_Entry_R()
{
    if (instruction != 54)
        return false;
    if (!NearPoint(17))
        return true;
    instruction = 1013;
    return false;
}                   

//敌方步兵均阵亡
bool DecisionMaker::Enemy_Ally_Died_And_Stay()
{
    if (instruction != 36)
        return false;
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 20)
        return false;
    if (recent_referee[0].back().my_guard_blood <= 150)
    {
        instruction = 13;
        return true;
    }
    //更新到全局任务
    instruction = 1003;
    return true;
}                             


/*------------------------------------------全局任务----------------------------------------*/
/*------------------------------------------全局任务----------------------------------------*/

//友军位置
int DecisionMaker::Ally_Posi(int x)
{
    int a, b;
    if (x == 1)
        a = -6, b = -1.5;
    else if (x == 2)
        a = -1.5, b = 1.5;
    else if (x == 3)
        a = 1.5, b = 6;
    int n = 0;
    if (!my_r3.empty() && my_r3.back().robot_posi[0] >= a && my_r3.back().robot_posi[0] <= b)
        n++;
    if (type == 0 || type == 1)
    {
        if (!my_r4.empty() && my_r4.back().robot_posi[0] >= a && my_r4.back().robot_posi[0] <= b)
            n++;
    }
    else if (type == 2 || type == 3)
    {
        if (!my_r1.empty() && my_r1.back().robot_posi[0] >= a && my_r1.back().robot_posi[0] <= b)
            n++;
    }
    return n;
} 

//敌方位置                              
int DecisionMaker::Enemy_Posi(int x)
{
    int a, b;
    if (x == 1)
        a = -6, b = -1.5;
    else if (x == 2)
        a = -1.5, b = 1.5;
    else if (x == 3)
        a = 1.5, b = 6;
    int n = 0;
    if (!enemy_guard.empty() && enemy_guard.back().robot_posi[0] >= a && enemy_guard.back().robot_posi[0] <= b)
        n++;
    if (!enemy_r3.empty() && enemy_r3.back().robot_posi[0] >= a && enemy_r3.back().robot_posi[0] <= b)
        n++;
    if (type == 0 || type == 2)
    {
        if (!enemy_r4.empty() && enemy_r4.back().robot_posi[0] >= a && enemy_r4.back().robot_posi[0] <= b)
        n++;
    }
    if (type == 1 || type == 3)
    {
        if (!enemy_r1.empty() && enemy_r1.back().robot_posi[0] >= a && enemy_r1.back().robot_posi[0] <= b)
            n++;
    }
    return n;
}                              

//劣势
bool DecisionMaker::Inferior_Situation()
{
    if (ally_live < enemy_ally_live && enemy_ally_blood >= ally_blood*2)
    {
        ROS_INFO("%d %d", instruction, 11111);
        return true;
    }
    ROS_INFO("%d %d %d", enemy_ally_blood, ally_blood, 33333);
    if (enemy_ally_blood - ally_blood > 200 || enemy_ally_blood >= ally_blood *3)
    {
        ROS_INFO("%d %d", instruction, 22222);
        return true;
    }
    return false;
}   

//优势          
bool DecisionMaker::Favorable_Situation()
{
    if (enemy_ally_live < ally_live && ally_blood >= 2*enemy_ally_blood)
        return true;
    if (ally_blood - enemy_ally_blood > 200 || ally_blood >= enemy_ally_blood*3)
        return true;
    return false;
}                        

//基地被攻击
bool DecisionMaker::Base_Been_Attacker()
{
    //判断我方基地是否被攻击
    bool attacked = false;
    if (recent_referee[0].front().my_base_blood - recent_referee[0].back().my_base_blood > 0
        || recent_referee[0].front().my_base_shield - recent_referee[0].back().my_base_shield > 0)
        attacked = true;
    
    //我方基地没有被攻击
    if  (!attacked)
        return false;
    //我方哨兵正在攻击敌方
    if (recent_referee[0].front().my_guard_bullet - recent_referee[0].back().my_guard_bullet >= 10)
        return false;
    //敌方为双步兵阵容 我方基地护盾还在 友军血量大于50
    if ((type == 0 || type == 2) && recent_referee[0].back().my_base_shield > 0 && ally_blood >= 50)
        return false;
    instruction = 1101;
    return true;
}   

//前往基地左前方
bool DecisionMaker::Return_To_Base_LF()
{
    if (instruction != 1101)
        return false;
    if (!NearPoint(18))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 3)
        return true;
    time = -1;
    instruction = 1102;
    return false;
}   

//离开基地左前方
bool DecisionMaker::Leave_Base_LF()
{
    if (instruction != 1102)
        return false;
    //哨兵状态不好
    if (recent_referee[0].back().my_guard_blood <= 50)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[0].front().my_guard_bullet - recent_referee[0].back().my_guard_bullet >= 4)
        return false;
    //敌方可能在背面攻击
    if (recent_referee[0].front().my_base_blood - recent_referee[0].back().my_base_blood > 0
        || recent_referee[0].front().my_base_shield - recent_referee[0].back().my_base_shield > 0)
    {
        instruction = 1103;
        return true;
    }
    //哨兵状态不好
    if (recent_referee[0].back().my_guard_blood <= 200)
    {
        instruction = 1109;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        instruction = 1111;
        return true;
    }
    //优势
    if (Favorable_Situation())
    {
        instruction = 1205;
        return true;
    }
    //正常
    instruction = 1201;
    return true;

}

//前往基地右前方
bool DecisionMaker::Return_To_Base_RF()
{
    if (instruction != 1103)
        return false;
    if (!NearPoint(19))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 3)
        return true;
    time = -1;
    instruction = 1104;
    return false;
}   

//离开基地右前方
bool DecisionMaker::Leave_Base_RF()
{
    if (instruction != 1104)
        return false;
    //哨兵状态不好
    if (recent_referee[0].back().my_guard_blood <= 50)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[0].front().my_guard_bullet - recent_referee[0].back().my_guard_bullet >= 4)
        return false;
    //敌方可能在背面攻击
    if (recent_referee[0].front().my_base_blood - recent_referee[0].back().my_base_blood > 0
        || recent_referee[0].front().my_base_shield - recent_referee[0].back().my_base_shield > 0)
    {
        instruction = 1105;
        return true;
    }
    //哨兵状态不好
    if (recent_referee[0].back().my_guard_blood <= 200)
    {
        instruction = 1109;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        instruction = 1111;
        return true;
    }
    //优势
    if (Favorable_Situation())
    {
        instruction = 1205;
        return true;
    }
    //正常
    instruction = 1201;
    return true;

}

//前往基地右后方
bool DecisionMaker::Return_To_Base_RB()
{
    if (instruction != 1105)
        return false;
    if (!NearPoint(20))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 3)
        return true;
    time = -1;
    instruction = 1106;
    return false;
}   

//离开基地右后方
bool DecisionMaker::Leave_Base_RB()
{
    if (instruction != 1106)
        return false;
    //哨兵状态不好
    if (recent_referee[0].back().my_guard_blood <= 50)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[0].front().my_guard_bullet - recent_referee[0].back().my_guard_bullet >= 4)
        return false;
    //敌方可能在背面攻击
    if (recent_referee[0].front().my_base_blood - recent_referee[0].back().my_base_blood > 0
        || recent_referee[0].front().my_base_shield - recent_referee[0].back().my_base_shield > 0)
    {
        instruction = 1107;
        return true;
    }
    //哨兵状态不好
    if (recent_referee[0].back().my_guard_blood <= 200)
    {
        instruction = 1109;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        instruction = 1111;
        return true;
    }
    //优势
    if (Favorable_Situation())
    {
        instruction = 1205;
        return true;
    }
    //正常
    instruction = 1201;
    return true;

}

//前往基地左后方
bool DecisionMaker::Return_To_Base_LB()
{
    if (instruction != 1107)
        return false;
    if (!NearPoint(21))
        return false;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 3)
        return true;
    time = -1;
    instruction = 1108;
    return false;
}                                            

//离开基地左后方
bool DecisionMaker::Leave_Base_LB()
{
    if (instruction != 1108)
        return false;
    //哨兵状态不好
    if (recent_referee[0].back().my_guard_blood <= 50)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[0].front().my_guard_bullet - recent_referee[0].back().my_guard_bullet >= 4)
        return false;
    //哨兵状态不好
    if (recent_referee[0].back().my_guard_blood <= 200)
    {
        instruction = 1109;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        instruction = 1111;
        return true;
    }
    //优势
    if (Favorable_Situation())
    {
        instruction = 1205;
        return true;
    }
    //正常
    instruction = 1201;
    return true;

}

//回补给区恢复
bool DecisionMaker::Return_To_Recover()
{
    if (instruction != 1109)
        return false;
    if (!NearPoint(22))
        return true;
    if (ally_blood <= 100 && recent_referee[0].back().my_guard_blood < 400)
        return true;
    if (recent_referee[0].back().my_guard_blood < 550)
        return true;
    if (Inferior_Situation())
    {
        instruction = 1112;
        return false;
    }
    if (Favorable_Situation())
    {
        instruction = 1301;
        return false;
    }
    instruction = 1201;
    return false;
}                             

//前往掩体右后
bool DecisionMaker::Move_To_Barrier_RB()
{
    if (instruction != 1111)
        return false;
    if (!NearPoint(23))
        return true;
    if (time ==-1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 3)
        return true;
    time = -1;
    instruction = 1112;
    return false;
} 

//离开掩体右后    
bool DecisionMaker::Leave_Barrier_RB()
{
    if (instruction != 1112)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
        return false;
    //基地被攻击
    if (Base_Been_Attacker())
    {
        instruction = 1101;
        return true;
    }
    //优势
    if (Favorable_Situation())
    {
        instruction = 1205;
        return true;
    }
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 200)
    {
        instruction = 1109;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        return false;
    }
    //队友都在我方区域
    if (Ally_Posi(1) == ally_live)
    {
        return false;
    }
    //前往掩体右方
    else
    {
        instruction = 1201;
        return true;
    }
}                            

//前往掩体左后
bool DecisionMaker::Move_To_Barrier_LB()
{
    if (instruction != 1113)
        return false;
    if (!NearPoint(24))
        return true;
    if (time ==-1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 3)
        return true;
    time = -1;
    instruction = 1114;
    return false;
}   

//离开掩体左后                     
bool DecisionMaker::Leave_Barrier_LB()
{
    if (instruction != 1114)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
        return false;
    //基地被攻击
    if (Base_Been_Attacker())
    {
        instruction = 1101;
        return true;
    }
    //优势
    if (Favorable_Situation())
    {
        instruction = 1207;
        return true;
    }
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 200)
    {
        instruction = 1109;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        return false;
    }
    //队友都在我方区域
    if (Ally_Posi(1) == ally_live)
    {
        return false;
    }
    //前往掩体右方
    else
    {
        instruction = 1203;
        return true;
    }
}                            

//前往掩体右
bool DecisionMaker::Move_To_Barrier_R()
{
    if (instruction != 1201)
        return false;
    if (!NearPoint(25))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 3)
        return true;
    time = -1;
    instruction = 1202;
    return false;
}   

//离开掩体右                       
bool DecisionMaker::Leave_Barrier_R()
{
    if (instruction != 1202)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
        return false;
    //基地被攻击
    if (Base_Been_Attacker())
    {
        instruction = 1101;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        //队友在在身边
        if (Ally_Posi(2) >= 1 || Ally_Posi(3) >= 1)
            return false;
        else
        {
            instruction = 1111;
            return true;
        }
    }
    //优势
    if (Favorable_Situation())
    {
        //增益区打开
        if (recent_referee[0].back().benefit_open)
            instruction = 1209;
        else
            instruction = 1301;
        return true;
    }
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 150)
    {
        instruction = 1109;
        return true;
    }
    //队友都在中间
    if (Ally_Posi(2) == ally_live)
    {
        return false;
    }
    //增益区打开
    if (recent_referee[0].back().benefit_open && Enemy_Posi(2) < 3)
    {
        instruction = 1209;
        return true;
    }
    //长时间攻击慢
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet < 5)
    {
        instruction = 1205;
        return true;
    }
    return false;
}   

//前往掩体左
bool DecisionMaker::Move_To_Barrier_L()
{
    if (instruction != 1203)
        return false;
    if (!NearPoint(26))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 3)
        return true;
    time = -1;
    instruction = 1204;
    return false;
}   

//离开掩体左                        
bool DecisionMaker::Leave_Barrier_L()
{
    if (instruction != 1204)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
        return false;
    //基地被攻击
    if (Base_Been_Attacker())
    {
        instruction = 1101;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        //队友在在身边
        if (Ally_Posi(2) >= 1 || Ally_Posi(3) >= 1)
            return false;
        else
        {
            instruction = 1113;
            return true;
        }
    }
    //优势
    if (Favorable_Situation())
    {
        //增益区打开
        if (recent_referee[0].back().benefit_open)
            instruction = 1209;
        else
            instruction = 1303;
        return true;
    }
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 150)
    {
        instruction = 1109;
        return true;
    }
    //队友都在中间
    if (Ally_Posi(2) == ally_live)
    {
        return false;
    }
    //增益区打开
    if (recent_referee[0].back().benefit_open && Enemy_Posi(2) < 3)
    {
        instruction = 1209;
        return true;
    }
    //长时间攻击慢
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet < 5)
    {
        instruction = 1207;
        return true;
    }
    return false;
}                             

//前往中间右
bool DecisionMaker::Move_To_Middle_R()
{
    if (instruction != 1205)
        return false;
    if (!NearPoint(27))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    time = -1;
    instruction = 1206;
    return false;
}   

//离开中间右
bool DecisionMaker::Leave_Middle_R()
{
    if (instruction != 1206)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
        return false;
    //基地被攻击
    if (Base_Been_Attacker())
    {
        instruction = 1101;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        //队友在在身边
        if (Ally_Posi(2) >= 1 || Ally_Posi(3) >= 1)
            return false;
        else
        {
            ROS_INFO("%d %d", instruction, 55555);
            instruction = 1111;
            return true;
        }
    }
    //优势
    if (Favorable_Situation())
    {
        //增益区打开
        if (recent_referee[0].back().benefit_open)
            instruction = 1209;
        else
            instruction = 1303;
        return true;
    }
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 150)
    {
        instruction = 1109;
        return true;
    }
    //队友都在中间
    if (Ally_Posi(2) == ally_live)
    {
        return false;
    }
    //增益区打开
    if (recent_referee[0].back().benefit_open && Enemy_Posi(2) < 3)
    {
        instruction = 1209;
        return true;
    }
    //敌方跑到我方区域
    if (Enemy_Posi(1) >= 1)
    {
        instruction = 1201;
        return true;
    }
    //长时间攻击慢
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet < 5)
    {
        //队友在身边
        if (Ally_Posi(1) >= 1 || Ally_Posi(2) >= 1)
        {
            instruction = 1301;
            return true;
        }
        //不在
        else
        { 
            instruction = 1207;
            return true;
        }
    }
    return false;
}   

//前往中间左
bool DecisionMaker::Move_To_Middle_L()
{
    if (instruction != 1207)
        return false;
    if (!NearPoint(28))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    time = -1;
    instruction = 1208;
    return false;
}   

//离开中间左
bool DecisionMaker::Leave_Middle_L()
{
    if (instruction != 1208)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
        return false;
    //基地被攻击
    if (Base_Been_Attacker())
    {
        instruction = 1101;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        //队友在在身边
        if (Ally_Posi(2) >= 1 || Ally_Posi(3) >= 1)
            return false;
        else
        {
            instruction = 1113;
            return true;
        }
    }
    //优势
    if (Favorable_Situation())
    {
        //增益区打开
        if (recent_referee[0].back().benefit_open)
            instruction = 1209;
        else
            instruction = 1305;
        return true;
    }
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 150)
    {
        instruction = 1109;
        return true;
    }
    //队友都在中间
    if (Ally_Posi(2) == ally_live)
    {
        return false;
    }
    //增益区打开
    if (recent_referee[0].back().benefit_open && Enemy_Posi(2) < 3)
    {
        instruction = 1209;
        return true;
    }
    //敌方跑到我方区域
    if (Enemy_Posi(1) >= 1)
    {
        instruction = 1203;
        return true;
    }
    //长时间攻击慢
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet < 5)
    {
        //队友在身边
        if (Ally_Posi(1) >= 1 || Ally_Posi(2) >= 1)
        {
            instruction = 1303;
            return true;
        }
        //不在
        else
        { 
            instruction = 1205;
            return true;
        }
    }
    return false;
}                              

//前往中间中
bool DecisionMaker::Move_To_Middle_M()
{
    if (instruction != 1209)
        return false;
    if (!NearPoint(34))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 3)
        return true;
    time = -1;
    instruction = 1210;
    return false;
}   

//前往中间中
bool DecisionMaker::Leave_Middle_M()
{
    if (instruction != 1210)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //增益区未关闭
    if (recent_referee[0].back().benefit_open)
    {
        if (Enemy_Posi(1) <= 1)
            return false;
        if (Enemy_Posi(1) <= Ally_Posi(1) + 1)
            return false;
        if (recent_referee[0].front().my_guard_blood - recent_referee[0].back().my_guard_blood < 100)
            return false;
        instruction = 1113;
        return true;
    }
    //增益区关闭
    else
    {
        //哨兵血量不足
        if (recent_referee[0].back().my_guard_blood <= 200)
        {
            instruction = 1109;
            return true;
        }
        //哨兵在攻击
        if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
            return false;
        //基地被攻击
        if (Base_Been_Attacker())
        {
            instruction = 1101;
            return true;
        }
        //劣势
        if (Inferior_Situation())
        {
            //队友在在身边
            if (Ally_Posi(2) >= 1 || Ally_Posi(3) >= 1)
                return false;
            else
            {
                instruction = 1111;
                return true;
            }
        }
        //敌方跑到我方区域
        if (Enemy_Posi(1) >= 1)
        {
            instruction = 1201;
            return true;
        }
        //优势
        if (Favorable_Situation())
        {
            instruction = 1301;
            return true;
        }
        //正常
        instruction = 1201;
        return true;
    }
}                              

//敌方区域右方巡视 向前
bool DecisionMaker::Inspect_Enemy_R_F()
{
    if (instruction != 1401 && instruction != 1402 && instruction != 1403)
        return false;
    if (instruction == 1401 && !NearPoint(32))
        return true;
    if (instruction == 1401)
    {
        if (Enemy_Posi(3) != 0)
        {
            instruction = 1307;
            return false;
        }
        if (recent_referee[0].back().my_guard_blood <= 100)
        {
            instruction = 1209;
            return false;
        }
        instruction = 1402;
    }
    if (instruction == 1402 && !NearPoint(30))
        return true;
    if (instruction == 1402)
    {
        if (Enemy_Posi(3) > 0)
        {
            instruction = 1303;
            return false;
        }
        if (Enemy_Posi(2) > 0)
        {
            instruction = 1207;
            return false;
        }
        if (Enemy_Posi(1) > 0)
        {
            instruction = 1203;
            return false;
        }
        if (recent_referee[0].back().my_guard_blood <= 100)
        {
            instruction = 1209;
            return false;
        }
        instruction = 1403;
    }
    if (instruction == 1403 && !NearPoint(26))
        return true;
    if (instruction == 1403)
    {
        if (Enemy_Posi(2) > 0)
        {
            instruction = 1203;
            return false;
        }
        if (Enemy_Posi(1) > 0)
        {
            instruction = 1113;
            return false;
        }
        if (recent_referee[0].back().my_guard_blood <= 150)
        {
            instruction = 1209;
            return false;
        }
        instruction = 1301;
        return false;
    }
}   

//敌方区域右方巡视 向后
bool DecisionMaker::Inspect_Enemy_R_B()
{
    if (instruction != 1501 && instruction != 1502)
        return false;
    if (instruction == 1501 && !NearPoint(27))
        return true;
    if (instruction == 1501)
    {
        if (Enemy_Posi(2) != 0)
        {
            instruction = 1205;
            return false;
        }
        if (Enemy_Posi(1) != 0)
        {
            instruction = 1111;
            return false;
        }
        if (recent_referee[0].back().my_guard_blood <= 100)
        {
            instruction = 1209;
            return false;
        }
        instruction = 1502;
    }
    if (instruction == 1502 && !NearPoint(28))
        return true;
    if (instruction == 1502)
    {
        if (Enemy_Posi(3) > 0)
        {
            instruction = 1303;
            return false;
        }
        if (Enemy_Posi(2) > 0)
        {
            instruction = 1207;
            return false;
        }
        if (Enemy_Posi(1) > 0)
        {
            instruction = 1203;
            return false;
        }
        if (recent_referee[0].back().my_guard_blood <= 100)
        {
            instruction = 1209;
            return false;
        }
        instruction = 1301;
        return false;
    }
}                           

//前往敌方掩体右
bool DecisionMaker::Move_To_Enemy_Barrier_R()
{
    if (instruction != 1301)
        return false;
    if (!NearPoint(29))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    time = -1;
    instruction = 1302;
    return false;
}   

//离开敌方掩体右
bool DecisionMaker::Leave_Enemy_Barrier_R()
{
    if (instruction != 1302)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
        return false;
    //基地被攻击
    if (Base_Been_Attacker())
    {
        instruction = 1101;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        //队友在在身边
        if (Ally_Posi(3) >= 1)
            return false;
        else
        {
            instruction = 1111;
            return true;
        }
    }
    //增益区打开
    if (recent_referee[0].back().benefit_open)
    {
        //哨兵血量不足
        if (recent_referee[0].back().my_guard_blood <= 200)
        {
            instruction = 1209;
            return true;
        }
    }
    //敌方只剩哨兵
    if (Enemy_Posi(3) == 1 && recent_referee[0].back().enemy_guard_blood > 0 && !enemy_guard.empty() && enemy_guard.back().robot_posi[0] >= 1.5)
    {
        instruction = 1307;
        return true;
    }
    //敌方消失
    if (Enemy_Posi(3) == 0 && enemy_live >= 2)
    {
        instruction = 1401;
        return true;
    }
    //敌方步兵残血
    if (type == 0 || type == 1)
    {
        if (recent_referee[0].back().enemy_robot3 + recent_referee[0].back().enemy_robot4 <= 80)
        {
            instruction = 1305;
            return true;
        }
    }
    else if (recent_referee[0].back().enemy_robot3 + recent_referee[0].back().enemy_robot1 <= 80)
    {
        instruction = 1305;
        return true;
    }
    //攻击效率低
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet <= 5)
    {
        instruction = 1401;
        return true;
    }
    //优势
    if (Favorable_Situation())
    {
        instruction = 1305;
        return true;
    }
    //攻击敌方基地
    if (enemy_live <= 1 && recent_referee[0].back().remaining_time <= 60 && recent_referee[0].back().my_guard_bullet >= 80 && recent_referee[0].back().enemy_base_blood <= 500)
    {
        instruction = 1309;
        return true;
    }
    return false;
}   

//前往敌方掩体左
bool DecisionMaker::Move_To_Enemy_Barrier_L()
{
    if (instruction != 1303)
        return false;
    if (!NearPoint(30))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    time = -1;
    instruction = 1304;
    return false;
}   

//离开敌方掩体左
bool DecisionMaker::Leave_Enemy_Barrier_L()
{
    if (instruction != 1304)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
        return false;
    //基地被攻击
    if (Base_Been_Attacker())
    {
        instruction = 1101;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        //队友在在身边
        if (Ally_Posi(3) >= 1)
            return false;
        else
        {
            instruction = 1113;
            return true;
        }
    }
    //增益区打开
    if (recent_referee[0].back().benefit_open)
    {
        //哨兵血量不足
        if (recent_referee[0].back().my_guard_blood <= 200)
        {
            instruction = 1209;
            return true;
        }
    }
    //敌方只剩哨兵
    if (Enemy_Posi(3) == 1 && recent_referee[0].back().enemy_guard_blood > 0 && !enemy_guard.empty() && enemy_guard.back().robot_posi[0] >= 1.5)
    {
        instruction = 1307;
        return true;
    }
    //敌方消失
    if (Enemy_Posi(3) == 0 && enemy_live >= 2)
    {
        instruction = 1307;
        return true;
    }
    //敌方步兵残血
    if (type == 0 || type == 1)
    {
        if (recent_referee[0].back().enemy_robot3 + recent_referee[0].back().enemy_robot4 <= 80)
        {
            instruction = 1305;
            return true;
        }
    }
    else if (recent_referee[0].back().enemy_robot3 + recent_referee[0].back().enemy_robot1 <= 80)
    {
        instruction = 1305;
        return true;
    }
    //攻击效率低
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet <= 5)
    {
        instruction = 1307;
        return true;
    }
    //优势
    if (Favorable_Situation())
    {
        instruction = 1305;
        return true;
    }
    //攻击敌方基地
    if (enemy_live <= 1 && recent_referee[0].back().remaining_time <= 60 && recent_referee[0].back().my_guard_bullet >= 80 && recent_referee[0].back().enemy_base_blood <= 500)
    {
        instruction = 1309;
        return true;
    }
    return false;
}                       

//前往敌方掩体前
bool DecisionMaker::Move_TO_Enemy_Barrier_F()
{
    if (instruction != 1307)
        return false;
    if (!NearPoint(32))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    time = -1;
    instruction = 1308;
    return false;
}                     

//离开敌方掩体前
bool DecisionMaker::Leave_Enemy_Barrier_F()
{
    if (instruction != 1308)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
        return false;
    //基地被攻击
    if (Base_Been_Attacker())
    {
        instruction = 1101;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        //队友在在身边
        if (Ally_Posi(3) >= 1)
            return false;
        else
        {
            instruction = 1111;
            return true;
        }
    }
    //增益区打开
    if (recent_referee[0].back().benefit_open)
    {
        //哨兵血量不足
        if (recent_referee[0].back().my_guard_blood <= 200)
        {
            instruction = 1209;
            return true;
        }
    }
    //敌方消失
    if (Enemy_Posi(3) == 0 && enemy_live >= 2)
    {
        instruction = 1402;
        return true;
    }
    //敌方步兵残血
    if (type == 0 || type == 1)
    {
        if (recent_referee[0].back().enemy_robot3 + recent_referee[0].back().enemy_robot4 <= 80)
        {
            instruction = 1305;
            return true;
        }
    }
    else if (recent_referee[0].back().enemy_robot3 + recent_referee[0].back().enemy_robot1 <= 80)
    {
        instruction = 1305;
        return true;
    }
    //攻击效率低
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet <= 5)
    {
        instruction = 1402;
        return true;
    }
    //优势
    if (Favorable_Situation())
    {
        instruction = 1305;
        return true;
    }
    //攻击敌方基地
    if (enemy_live <= 1 && recent_referee[0].back().remaining_time <= 60 && recent_referee[0].back().my_guard_bullet >= 80 && recent_referee[0].back().enemy_base_blood <= 500)
    {
        instruction = 1309;
        return true;
    }
    return false;
}                       

//前往敌方区域右
bool DecisionMaker::Move_To_Enemy_Domain_R()
{
    if (instruction != 1305)
        return false;
    if (!NearPoint(31))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    time = -1;
    instruction = 1306;
    return false;
}   

//离开敌方区域右
bool DecisionMaker::Leave_Enemy_Domain_R()
{
    if (instruction != 1306)
        return false;
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 100)
    {
        instruction = 1109;
        return true;
    }
    //哨兵在攻击
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
        return false;
    //基地被攻击
    if (Base_Been_Attacker())
    {
        instruction = 1101;
        return true;
    }
    //劣势
    if (Inferior_Situation())
    {
        //队友在在身边
        if (Ally_Posi(3) >= 1)
            return false;
        else
        {
            instruction = 1201;
            return true;
        }
    }
    //增益区打开
    if (recent_referee[0].back().benefit_open)
    {
        //哨兵血量不足
        if (recent_referee[0].back().my_guard_blood <= 200)
        {
            instruction = 1209;
            return true;
        }
    }
    //敌方消失
    if (Enemy_Posi(3) == 0 && enemy_live >= 2)
    {
        instruction = 1501;
        return true;
    }
    //突然被攻击
    if (recent_referee[0].front().my_guard_blood - recent_referee[0].back().my_guard_blood > 210)
    {
        instruction = 1301;
        return true;
    }
    //攻击效率低
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet <= 5)
    {
        instruction = 1501;
        return true;
    }
    //攻击敌方基地
    if (enemy_live <= 1 && recent_referee[0].back().remaining_time <= 60 && recent_referee[0].back().my_guard_bullet >= 80 && recent_referee[0].back().enemy_base_blood <= 500)
    {
        instruction = 1309;
        return true;
    }
    return false;
}   

//前往敌方基地
bool DecisionMaker::Move_To_Enemy_Base()
{
    if (instruction != 1309)
        return false;
    if (!NearPoint(33))
        return true;
    if (time == -1)
        time = recent_referee[0].back().remaining_time;
    if (time - recent_referee[0].back().remaining_time <= 5)
        return true;
    time = -1;
    instruction = 1310;
    return false;
}   

//离开敌方基地
bool DecisionMaker::Leave_Enemy_Base()
{
    if (instruction != 1310)
        return false;
    if (recent_referee[0].back().my_guard_bullet * 5 - recent_referee[0].back().enemy_base_blood >= 50)
        return false;
    if (recent_referee[0].back().my_guard_bullet <= 50)
    {
        instruction = 1303;
        return true;
    }
    //哨兵血量低
    if (recent_referee[0].back().my_guard_blood < 50)
    {
        instruction = 1109;
        return true;
    }
    return false;
}                            

geometry_msgs::PoseStamped DecisionMaker::DecisionAction(const referee_system::referee_system& msg, double xx, double yy)
{
    VisualMsg tmp;
    tmp.time=msg.remaining_time;
    tmp.robot_posi[0]=-3;
    tmp.robot_posi[1]=-1;
    //enemy_guard.push_back(tmp);
    
    ROS_INFO("%d %d", msg.remaining_time, instruction);
    ROS_INFO("%lf %lf", xx, yy);
    
    RefereeMsg referee(msg);
    recent_referee[0].push_back(referee);
    recent_referee[1].push_back(referee);
    while (recent_referee[0].front().remaining_time - referee.remaining_time > 2)
        recent_referee[0].pop_front();
    while (recent_referee[1].front().remaining_time - referee.remaining_time > 5)
        recent_referee[1].pop_front();
    
    this->x = xx;
    this->y = yy;
    //更新信息
    Update();



    //开局阶段
    if (instruction <= 1000)
    {
        //开局更新任务 前往敌方补给区前
        if (recent_referee[0].back().remaining_time >= 295 && instruction == 0)
            instruction = 1;
        //未发现敌方哨兵 前往敌方补给区
        if (instruction == 1 && !Enemy_Guard_At_Left() && !Enemy_Guard_In_Front() && !NearPoint(0))
        {
            return Area_Enemy_Support();
        }
        /********************以下均为在前往敌方补给区的过程中 发现敌方哨兵也来干扰我方补给区前所作出的反应********************/
        //在左边发现敌方哨兵
        if (Enemy_Guard_At_Left())
        {
            return Area_BiL();
        }
        //在增左攻击敌方哨兵5s
        if (Attack_Guard_At_BiL())
        {
            return Area_BiL();
        }
        //若不需要离开增左 原地继续攻击
        if (instruction == 4 && !Leave_BiL())
        {
            return Area_BiL();
        }
        //哨兵在增左残血 返回补给区并等待恢复完成
        if (Back_And_Recover_Adversely())
        {
            return Area_My_Support();
        }
        //我方劣势 与队友继续作战
        if (Inferior_Measure_At_BiL() && instruction == 4)
        {
            return Area_BiL();
        }
        //我方劣势 退回我方掩体之后
        if (Back_From_BiL_To_My_Barrier())
        {
            return Area_My_Barrier_BL();
        }
        //敌方哨兵阵亡 敌方仍在攻击 原地迎战
        if (Enemy_Guard_Died_At_BiL() && instruction == 4)
        {
            return Area_BiL();
        }
        //敌方哨兵阵亡 前往敌方补给区前
        if (Advance_To_Enemy_Support())
        {
            return Area_Enemy_Support();
        }
        //敌方哨兵阵亡 回补给区并等待恢复完成
        if (Back_And_Recover_Favourably())
        {
            return Area_My_Support();
        }
        //敌方哨兵消失
        if (Enemy_Guard_Disappear_At_BL())
        {
            return Area_Middle_Leftest();
        }
        //敌方哨兵消失后 在我方领域发现敌方哨兵
        if (Enemy_Guard_In_My_Domain())
        {
            return Area_My_Domain();
        }
        //敌方哨兵消失后 未发现敌方哨兵
        if (Enemy_Guard_In_Unknow_Domain())
        {
            return Area_Enemy_Barrier_R();
        }
        //敌方哨兵消失后 在敌方领域发现敌方哨兵
        if (Enemy_Guard_In_Enemy_Domain())
        {
            instruction = 9;
        }
        //判断敌方哨兵撤退
        if (Enemy_Guard_Retrat_From_BiL());
        //敌方哨兵撤退 跟踪敌方
        if (Track_Enemy_Guard_At_BL())
        {

        }
        //敌方哨兵撤退 回补给区
        if (Back_And_Recover_Favourably())
        {
            return Area_My_Support();
        }
        //敌方哨兵撤退 去敌方障碍物右方干扰
        if (Advance_To_Enemy_Barrier_BL())
        {
            return Area_Enemy_Barrier_R();
        }

        /********************以下均为在前往敌方补给区的过程中 发现敌方哨兵在敌方补给区前所作出的反应********************/
        //在前方发现敌方哨兵  前往中右
        if (Enemy_Guard_In_Front())
        {
            return Area_Middle_Rightest();
        }
        //在中右攻击5s
        if (Attack_Guard_At_MidR())
        {
            double posi[3] = {x, y, 0};
            return PointMsg(posi);
        }
        //若不需要离开中右  继续原地攻击
        if (instruction == 22 && !Leave_MidR())
        {
            return Area_Middle_Rightest();
        }
        //哨兵残血 回补给区
        if (Back_And_Recover_Adversely())
        {
            return Area_My_Support();
        }
        //我方劣势 与队友继续作战
        if (Inferior_Measure_At_MidR())
        {
            return Area_Middle_Rightest();
        }
        //我方劣势 退回到我方掩体之后
        if (Back_From_MidR_To_My_Barrier())
        {
            return Area_My_Barrier_BR();
        }
        //敌方哨兵阵亡 敌方仍在攻击 原地迎战
        if (Enemy_Guard_Died_At_MidR() && instruction == 22)
        {
            return Area_Middle_Rightest();
        }
        //敌方哨兵阵亡 前往敌方敌方补给区前
        if (Advance_To_Enemy_Support())
        {
            return Area_Enemy_Support();
        }
        //敌方哨兵阵亡 回补给区并等待恢复完成
        if (Back_And_Recover_Favourably())
        {
            return Area_My_Support();
        }
        //敌方在中右消失
        if (Enemy_Guard_Disappear_At_MidR());
        //敌方在中右消失 回补给区
        if (Back_And_Recover_Favourably())
        {
            return Area_My_Support();
        }
        //敌方在中右消失 前往敌方掩体右方
        if (Advance_To_Enemy_Barrier_R())
        {
            return Area_Enemy_Barrier_R();
        }
        //敌方哨兵前进 跟踪敌方哨兵
        if (Track_Enemy_Guard_At_MidR())
        {

        }
        //敌方哨兵撤退
        if (Enemy_Guard_Retrat_From_MidR());
        //敌方哨兵撤退  回补给区
        if (Back_And_Recover_Adversely())
        {
            return Area_My_Support();
        }
        //敌方哨兵撤退  回补给区
        if (Back_And_Recover_Favourably())
        {
            return Area_My_Support();
        }
        //敌方哨兵撤退  前进到敌方补给区前
        if (Advance_To_Enemy_Support())
        {
            return Area_Enemy_Support();
        }

        /********************以下均为哨兵前往敌方补给区的过程中 未发现敌方哨兵所作出的反应********************/
        //未发现敌方哨兵  前往敌方补给区前干扰
        if (Start_And_Advance_To_Enemy_Support())
        {
            return Area_Enemy_Support();
        }
        //在敌方补给区前攻击敌方5s
        if (Attack_Guard_At_EneSup())
        {
            return Area_Enemy_Support();
        }
        //判断是否需要离开敌方补给区前
        if (instruction == 32 && !Leave_EneSup())
        {
            return Area_Enemy_Support();
        }
        //哨兵残血 返回补给区
        if (Back_And_Recover_Favourably())
        {
            return Area_My_Support();
        }
        //劣势 与队友继续作战
        if (Inferior_Measure_At_EneSup())
        {
            return Area_Enemy_Support();
        }
        //劣势 返回我方掩体正后方
        if (Back_From_EneSup_To_My_BarrierB())
        {
            return Area_My_Barrier_B();
        }
        //判断敌方哨兵阵亡
        if (Enemy_Guard_Died_At_EneSup());
        //敌方哨兵阵亡 返回补给区
        if (Back_And_Recover_Favourably())
        {
            return Area_My_Support();
        }
        //敌方哨兵消失 原地攻击
        if (Enemy_Guard_Disappear_At_EneSup() && instruction == 32)
        {
            return Area_Enemy_Support();
        }
        //敌方哨兵消失 返回我方
        if (Back_To_My_DomainL_By_Right())
        {
            switch (instruction)
            {
            case 39:
                return Area_Middle_Right();
            case 40:
                return Area_Middle_Leftest();
            case 41:
                return Area_My_Domain_Left();
            default:
                break;
            }
        }
        //返回我方区域 --在中间右方发现敌人
        if (Find_Enemy_At_Middle_Right())
        {
            return Area_Middle_Right();
        }
        //返回我方区域 --继续守在中间右方攻击
        if (!Leave_Middle_Right() && instruction == 47)
        {
            return Area_Middle_Right();
        }
        //返回我方区域 --回补给区
        if (Back_And_Recover_Adversely())
        {
            return Area_My_Support();
        }
        //返回我方区域 --劣势 返回我方掩体正后方
        if (Back_From_EneSup_To_My_BarrierB())
        {
            return Area_My_Barrier_B();
        }
        //返回我方区域 --回补给区
        if (Back_And_Recover_Favourably())
        {
            return Area_My_Support();
        }
        //返回我方区域 --前进到敌方补给区前(敌方哨兵阵亡)
        if (Advance_To_Enemy_Support())
        {
            return Area_Enemy_Support();
        }
        //返回我方区域 --前往我方区域左方
        if (instruction == 41)
        {
            return Area_My_Domain_Left();
        }
        //前往我方区域 --返回基地入口
        if (Back_To_Base_Enter())
        {
            return Area_Base_Entry();
        }
        //前往我方区域 --留在基地入口攻击
        if (!Leave_Base_Entry() && instruction == 50)
        {
            return Area_Base_Entry();
        }
        //前往我方区域 --返回补给区
        if (Back_And_Recover_Adversely())
        {
            return Area_My_Support();
        }
        //前往我方区域 --前往我方右前方
        if (Advance_To_My_Barrier_FR())
        {
            return Area_My_Barrier_FR();
        }
        //前往我方区域 --在中间左方攻击5s
        if (Find_Enemy_At_Middle_Left())
        {
            return Area_Middle_Leftest();
        }
        //前往我方区域 --留在中间左方攻击
        if (!Leave_Middle_Leftest() && instruction == 48)
        {
            return Area_Middle_Leftest();
        }
        //返回我方区域 --回补给区
        if (Back_And_Recover_Adversely())
        {
            return Area_My_Support();
        }
        //返回我方区域 --劣势 返回我方掩体正后方
        if (Back_From_EneSup_To_My_BarrierB())
        {
            return Area_My_Barrier_B();
        }
        //返回我方区域 --回补给区
        if (Back_And_Recover_Favourably())
        {
            return Area_My_Support();
        }
        //返回我方区域 --前进到敌方补给区前(敌方哨兵阵亡)
        if (Advance_To_Enemy_Support())
        {
            return Area_Enemy_Support();
        }
        //返回我方区域 --前往我方区域左方
        if (instruction == 41)
        {
            return Area_My_Domain_Left();
        }
        //返回我方区域 --留在我方区域左方攻击
        if (Find_Enemy_At_My_Domain_Left())
        {
            return Area_My_Domain_Left();
        }
        //返回我方区域 --返回补给区
        if (Back_And_Recover_Adversely())
        {
            return Area_My_Support();
        }
        //返回我方区域 --前往我方掩体右前方
        if (Advance_To_My_Barrier_FR())
        {
            return Area_My_Barrier_FR();
        }
        //前往敌方掩体左前方
        if (Advance_To_Enemy_Barrier_FL())
        {
            return Area_Enemy_Barrier_FL();
        }
        //留在敌方掩体左方攻击
        if (!Leave_Enemy_Barrier_FL() && instruction == 51)
        {
            return Area_Enemy_Barrier_FL();
        }
        //返回我方补给区
        if (Back_And_Recover_Adversely())
        {
            return Area_My_Support();
        }
        //劣势 返回我方掩体左后方
        if (Inferior_Measure_At_Enemy_Barrier_FL())
        {
            return Area_My_Barrier_BL();
        }
        //敌方哨兵阵亡 前往敌方补给区
        if (Enemy_Guard_Died_At_Enemy_Barrier_FL())
        {
            return Area_Enemy_Support();
        }
        //返回我方区域右出口
        if (Back_To_My_Domain_Entry_R())
        {
            ROS_INFO("%d %d", instruction, 11111);
            return Area_My_Domain_Entry_R();
        }
        //敌方步兵阵亡 留在原地
        if (Enemy_Ally_Died_And_Stay() && instruction == 36)
        {
            return Area_Enemy_Support();
        }
        //敌方步兵阵亡 返回补给区
        if (Back_And_Recover_Favourably())
        {
            return Area_My_Support();
        }
    }



    //过渡阶段
    if (instruction > 1000 && instruction <= 1100)
    {
        if (instruction == 1001 || instruction == 1004)
            instruction = 1109;
        else if (instruction == 1010)
            instruction = 1014;
        else if (instruction == 1012)
            instruction = 1112;
        else if (instruction == 1005 || instruction == 1002 || instruction == 1009 || instruction == 1007 || instruction == 1013)
        {
            if (recent_referee[0].back().my_guard_blood <= 100)
                instruction = 1109;
            else if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet >= 15)
                return Area_Current_Position();
            else if (Enemy_Posi(1) > 0)
            {
                if (instruction == 1005 || instruction == 1002)
                    instruction = 1113;
                else
                    instruction = 1111;
            }
            else
            {
                switch (instruction)
                {
                case 1005:
                    instruction = 1203;
                    break;
                case 1002:
                    instruction = 1113;
                    break;
                case 1009:
                    instruction = 1111;
                    break;
                case 1007:
                    instruction = 1111;
                    break;
                case 1013:
                    instruction = 1201;
                    break;
                default:
                    break;
                }
            }
        }
        else if (instruction = 1011)
            instruction = 1202;
        else if (instruction == 1006 || instruction == 1008)
            instruction = 1302;
        else if (instruction == 1003)
            instruction = 1306;
    }



    //全局阶段
    if (instruction > 1100)
    {
        //补给区
        if (Return_To_Recover())
        {
            return Posi_My_Support();
        }

        //基地左前
        if (Return_To_Base_LF())
        {
            return Posi_My_Base_LF();
        }
        if (!Leave_Base_LF() && instruction == 1102)
        {
            return Posi_My_Base_LF();
        }

        //基地右前
        if (Return_To_Base_RF())
        {
            return Posi_My_Base_RF();
        }
        if (!Leave_Base_RF() && instruction == 1104)
        {
            return Posi_My_Base_RF();
        }

        //基地右后
        if (Return_To_Base_RB())
        {
            return Posi_My_Base_RB();
        }
        if (!Leave_Base_RB() && instruction == 1106)
        {
            return Posi_My_Base_RB();
        }
        
        //基地左后
        if (Return_To_Base_LB())
        {
            return Posi_My_Base_LB();
        }
        if (!Leave_Base_LB() && instruction == 1108)
        {
            return Posi_My_Base_LB();
        }
        
        //掩体右后
        if (Move_To_Barrier_RB())
        {
            return Posi_My_Barrier_RB();
        }
        if (!Leave_Barrier_RB() && instruction == 1112)
        {
            return Posi_My_Barrier_RB();
        }

        //掩体左后
        if (Move_To_Barrier_LB())
        {
            return Posi_My_Barrier_LB();
        }
        if (!Leave_Barrier_LB() && instruction == 1114)
        {
            return Posi_My_Barrier_LB();
        }

        //掩体右
        if (Move_To_Barrier_R())
        {
            return Posi_My_Barrier_R();
        }
        if (!Leave_Barrier_R() && instruction == 1202)
        {
            return Posi_My_Barrier_R();
        }

        //掩体左
        if (Move_To_Barrier_L())
        {
            return Posi_My_Barrier_L();
        }
        if (!Leave_Barrier_L() && instruction == 1204)
        {
            return Posi_My_Barrier_L();
        }

        //中间右
        if (Move_To_Middle_R())
        {
            return Posi_Middle_R();
        }
        if (!Leave_Middle_R() && instruction == 1206)
        {
            return Posi_Middle_R();
        }

        //中间左
        if (Move_To_Middle_L())
        {
            return Posi_Middle_L();
        }
        if (!Leave_Middle_L() && instruction == 1208)
        {
            return Posi_Middle_L();
        }

        //中间中
        if (Move_To_Middle_M())
        {
            return Posi_Middle_M();
        }
        if (!Leave_Middle_M() && instruction == 1210)
        {
            return Posi_Middle_M();
        }

        //敌方掩体右
        if (Move_To_Enemy_Barrier_R())
        {
            return Posi_Enemy_Barrier_R();
        }
        if (!Leave_Enemy_Barrier_R() && instruction == 1302)
        {
            return Posi_Enemy_Barrier_R();
        }

        //敌方掩体左
        if (Move_To_Enemy_Barrier_L())
        {
            return Posi_Enemy_Barrier_L();
        }
        if (!Leave_Enemy_Barrier_L() && instruction == 1304)
        {
            return Posi_Enemy_Barrier_L();
        }

        //敌方掩体右
        if (Move_TO_Enemy_Barrier_F())
        {
            return Posi_Enemy_Barrier_F();
        }
        if (!Leave_Enemy_Barrier_F() && instruction == 1308)
        {
            return Posi_Enemy_Barrier_F();
        }

        //敌方区域右
        if (Move_To_Enemy_Domain_R())
        {
            return Posi_Enemy_Domain_R();
        } 
        if (!Leave_Enemy_Domain_R() && instruction == 1306)
        {
            return Posi_Enemy_Domain_R();
        }

        //敌方基地
        if (Move_To_Enemy_Base())
        {
            return Posi_Enemy_Base();
        }
        if (!Leave_Enemy_Base() && instruction == 1310)
        {
            return Posi_Enemy_Base();
        }

        //巡视 向前
        if (Inspect_Enemy_R_F())
        {
            switch (instruction)
            {
            case 1401:
                return Posi_Enemy_Barrier_F();
            case 1402:
                return Posi_Enemy_Barrier_L();
            case 1403:
                return Posi_My_Barrier_L();
            }
        }

        //巡视 向后
        if (Inspect_Enemy_R_B())
        {
            switch (instruction)
            {
            case 1501:
                return Posi_Middle_R();
            case 1502:
                return Posi_Middle_L();
            }
        }
    }



    //默认目标
    switch (instruction)
    {
    case 1:
        return Area_Enemy_Support();
    case 2:
        return Area_BiL();
    case 3:
        return Area_BiL(); 
    case 4:
        return Area_BiL(); 
    case 5:
        return Area_My_Support();
    case 6:
        return Area_BiL(); 
    case 7:
        return Area_BiL(); 
    case 8:
        return Area_Middle_Leftest();
    case 9:
        return Area_BiL(); 
    case 11:
        return Area_My_Barrier_BL();
    case 12:
        return Area_Enemy_Support();
    case 13:
        return Area_My_Support();
    case 14:
        return Area_My_Domain();
    case 15:
        return Area_BiL(); 
    case 16:
        return Area_Enemy_Barrier_R();
    case 17:
        return Area_Current_Position();
    case 18:
        return Area_My_Support();
    case 19:
        return Area_Enemy_Barrier_R();
    case 20:
        return Area_Middle_Rightest();
    case 21:
        return Area_Current_Position();
    case 22:
        return Area_Middle_Rightest();
    case 23:
        return Area_Middle_Rightest();
    case 24:
        return Area_Middle_Rightest();
    case 25:
        return Area_Middle_Rightest();
    case 26:
        return Area_Current_Position();
    case 27:
        return Area_Middle_Rightest();
    /*case 28*/
    case 29:
        return Area_My_Barrier_BR();
    case 30:
        return Area_Enemy_Barrier_R();
    case 31:
        return Area_Enemy_Support();
    case 32:
        return Area_Enemy_Support();
    case 33:
        return Area_Current_Position();
    case 34:
        return Area_Current_Position();
    case 35:
        return Area_Current_Position();
    case 36:
        return Area_Current_Position();
    case 37:
        return Area_Current_Position();
    case 38:
        return Area_My_Barrier_B();
    case 39:
        return Area_Middle_Right();
    case 40:
        return Area_Middle_Leftest();
    case 41:
        return Area_My_Domain_Left();
    case 42:
        return Area_Current_Position();
    case 43:
        return Area_Base_Entry();
    case 44:
        return Area_Current_Position();
    case 45:
        return Area_Current_Position();
    case 46:
        return Area_Enemy_Barrier_FL();
    case 47:
        return Area_Middle_Right();
    case 48:
        return Area_Middle_Leftest();
    case 49:
        return Area_My_Barrier_FR();
    case 50:
        return Area_Base_Entry();
    case 51:
        return Area_Enemy_Barrier_FL();
    case 52:
        return Area_My_Barrier_BL();
    case 53:
        return Area_Enemy_Support();
    case 54:
        return Area_My_Domain_Entry_R();

    case 1001:
        return Area_My_Support();
    case 1002:
        return Area_My_Barrier_BL();
    case 1003:
        return Area_Enemy_Support();
    case 1004:
        return Area_My_Support();
    case 1005:
        return Area_My_Domain();
    case 1006:
        return Area_Enemy_Barrier_R();
    case 1007:
        return Area_My_Barrier_BR();
    case 1008:
        return Area_Enemy_Barrier_R();
    case 1009:
        return Area_My_Barrier_B();
    case 1010:
        return Area_My_Domain_Left();
    case 1011:
        return Area_My_Barrier_FR();
    case 1012:
        return Area_Base_Entry();
    case 1013:
        return Area_My_Domain_Entry_R();

    case 1101:
        return Posi_My_Base_LF();
    case 1102:
        return Posi_My_Base_LF();
    case 1103:
        return Posi_My_Base_RF();
    case 1104:
        return Posi_My_Base_RF();
    case 1105:
        return Posi_My_Base_RB();
    case 1106:
        return Posi_My_Base_RB();
    case 1107:
        return Posi_My_Base_LB();
    case 1108:
        return Posi_My_Base_LB();
    case 1109:
        return Posi_My_Support();
    case 1110:
        return Posi_My_Support();
    case 1111:
        return Posi_My_Barrier_RB();
    case 1112:
        return Posi_My_Barrier_RB();
    case 1113:
        return Posi_My_Barrier_LB();
    case 1114:
        return Posi_My_Barrier_LB();
    case 1201:
        return Posi_My_Barrier_R();
    case 1202:
        return Posi_My_Barrier_R();
    case 1203:
        return Posi_My_Barrier_L();
    case 1204:
        return Posi_My_Barrier_L();
    case 1205:
        return Posi_Middle_R();
    case 1206:
        return Posi_Middle_R();
    case 1207:
        return Posi_Middle_L();
    case 1208:
        return Posi_Middle_L();
    case 1209:
        return Posi_Middle_M();
    case 1210:
        return Posi_Middle_M();
    case 1301:
        return Posi_Enemy_Barrier_R();
    case 1302:
        return Posi_Enemy_Barrier_R();
    case 1303:
        return Posi_Enemy_Barrier_L();
    case 1304:
        return Posi_Enemy_Barrier_L();
    case 1305:
        return Posi_Enemy_Domain_R();
    case 1306:
        return Posi_Enemy_Domain_R();
    case 1307:
        return Posi_Enemy_Barrier_F();
    case 1308:
        return Posi_Enemy_Barrier_F();
    case 1309:
        return Posi_Enemy_Base();
    case 1310:
        return Posi_Enemy_Base();
    
    case 1401:
        return Posi_Enemy_Barrier_F();
    case 1402:
        return Posi_Enemy_Barrier_L();
    case 1403:
        return Posi_My_Barrier_L();
    case 1501:
        return Posi_Middle_R();
    case 1502:
        return Posi_Middle_L();
    }
    return Area_BiL();
}