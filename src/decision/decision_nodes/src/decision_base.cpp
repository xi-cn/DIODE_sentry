#include <decision_nodes/decision_base.h>


Point::Point(double x, double y)
{
    this->x = x + Point::offset_x;
    this->y = y + Point::offset_y;
}

VisualMsg::VisualMsg(double time, double x, double y)
{
    this->time = time;
    this->robot_pos = Point(x, y);
}

RefereeMsg::RefereeMsg(const referee_system::referee_system& msg)
{
    //我方颜色
    color = msg.color;
    //剩余时间
    remaining_time = msg.remaining_time;

    //我方基地血量
    my_base_blood = color ? msg.red_base_blood : msg.blue_base_blood;
    //我方基地是否有护盾
    my_base_protected = color ? msg.red_base_proteted : msg.blue_base_protected;
    //我方基地护盾量
    my_base_shield = color ? msg.red_base_shield : msg.blue_base_shield;
    //敌方基地血量
    enemy_base_blood = color ? msg.blue_base_blood : msg.red_base_blood;

    //我方哨兵血量
    my_guard_blood = color ? msg.r7_blood : msg.b7_blood;
    //我方哨兵子弹
    my_guard_bullet = color ? msg.r7_bullet : msg.b7_bullet;
    //敌方哨兵血量
    enemy_guard_blood = color ? msg.b7_blood : msg.r7_blood;

    //我方3号步兵血量
    my_robot3 = color ? msg.r3_blood : msg.b3_blood;
    //我方3号步兵子弹
    my_robot3_bullet = color ? msg.r3_bullet : msg.b3_bullet;
    enemy_robot3 = color ? msg.b3_blood : msg.r3_blood;


    if (decision_base::type == 0 || decision_base::type == 1)
    {
        //我方4号步兵血量和子弹
        my_robot4 = color ? msg.r4_blood : msg.b4_blood;
        my_robot4_bullet = color ? msg.r4_bullet : msg.b4_bullet;
    }
    else
    {
        //我方英雄血量和子弹
        my_robot1 = color ? msg.r1_blood : msg.b1_blood;
        my_robot1_bullet = color ? msg.r1_bullet : msg.b1_bullet;
    }

    if (decision_base::type == 0 || decision_base::type == 3)
    {
        //敌方4号步兵血量
        enemy_robot4 = color ? msg.b4_blood : msg.r4_blood;
    }
    else
    {  
        //敌方英雄血量
        enemy_robot1 = color ? msg.b1_blood : msg.r1_blood;
    }

    //我方增益区能量
    my_energy = color ? msg.red_energy : msg.blue_energy;
    //敌方增益区能量
    enemy_energy = color ? msg.blue_energy : msg.blue_energy;
    //增益区是否打开
    benefit_open = msg.benefit_area_open;
}

//某个区域的队友人数 1-我方  2-中间  3-敌方
int decision_base::Ally_Amount_In_Specific_Pos(int area)
{
    double area_f, area_b;
    switch (area)
    {
    case 1:
        //我方区域
        area_f = -1.5, area_b = -6;
        break;
    case 2:
        //中间区域
        area_f = 1.5, area_b = -1.5;
        break;
    case 3:
        //敌方区域
        area_f = 6, area_b = 1.5;
        break;
    }

    int n = 0;
    //3号步兵
    if (!my_r3.empty() && my_r3.back().robot_pos.x >= area_b && my_r3.back().robot_pos.x <= area_f)
        n++;
    //4号步兵
    if (type == 0 || type == 1)
    {
        if (!my_r4.empty() && my_r4.back().robot_pos.x >= area_b && my_r4.back().robot_pos.x <= area_f)
            n++;
    }
    //英雄机器人
    else if (type == 2 || type == 3)
    {
        if (!my_r1.empty() && my_r1.back().robot_pos.x >= area_b && my_r1.back().robot_pos.x <= area_f)
            n++;
    }
    return n;
}

//某个区域敌方人数 1-我方  2-中间  3-敌方
int decision_base::Enemy_Amount_In_Specific_Pos(int area)
{
    double area_f, area_b;
    switch (area)
    {
    case 1:
        //我方区域
        area_f = -1.5, area_b = -6;
        break;
    case 2:
        //中间区域
        area_f = 1.5, area_b = -1.5;
        break;
    case 3:
        //敌方区域
        area_f = 6, area_b = 1.5;
        break;
    }
    int n = 0;
    //敌方哨兵
    if (!enemy_guard.empty() && enemy_guard.back().robot_pos.x >= area_b && enemy_guard.back().robot_pos.x <= area_f)
        n++;
    //3号步兵
    if (!enemy_r3.empty() && enemy_r3.back().robot_pos.x >= area_b && enemy_r3.back().robot_pos.x <= area_f)
        n++;
    //4号步兵
    if (type == 0 || type == 2)
    {
        if (!enemy_r4.empty() && enemy_r4.back().robot_pos.x >= area_b && enemy_r4.back().robot_pos.x <= area_f)
        n++;
    }
    //英雄机器人
    if (type == 1 || type == 3)
    {
        if (!enemy_r1.empty() && enemy_r1.back().robot_pos.x >= area_b && enemy_r1.back().robot_pos.x <= area_f)
            n++;
    }
    return n;
}

//将坐标点转换为消息格式
geometry_msgs::PoseStamped decision_base::Standarize_Pose(double x, double y)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0;

    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;

    return msg;
}

//将坐标点转换为消息格式
geometry_msgs::PoseStamped decision_base::Standarize_Pose(Point p)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.pose.position.x = p.x;
    msg.pose.position.y = p.y;
    msg.pose.position.z = 0;

    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;

    return msg;
}

//设置目标点为当前位置
void decision_base::Stay_Put()
{
    STORED_POSITION = Standarize_Pose(pos);
    GOAL = &STORED_POSITION;
}

//修改目标点至合法点
void decision_base::Modify_Goal()
{
    if (!decision_base::Is_Position_Valid()){
        while (!decision_base::Seek_Valid_Position()){
            decision_base::Add_Max_Radius();
        }
    }
    GOAL = TMP_GOAL;
}

//判断当前点是否合法
bool decision_base::Is_Position_Valid()
{
    if (grid_map.data.empty()){
        return true;
    }
    double origin_x = grid_map.info.origin.position.x;
    double origin_y = grid_map.info.origin.position.y;
    double resolution = grid_map.info.resolution;

    int x_ = (TMP_GOAL->pose.position.x - origin_x) / resolution;
    int y_ = (TMP_GOAL->pose.position.y - origin_y) / resolution;

    return grid_map.data[y_*grid_map.info.width+x_] == 0;
}

//寻找合法点
bool decision_base::Seek_Valid_Position()
{
    ROS_INFO("(%.1f, %.1f) is not valid, trying to find another point at the radius of %.1f",
        TMP_GOAL->pose.position.x, TMP_GOAL->pose.position.y, max_radius);

    double origin_x = grid_map.info.origin.position.x;
    double origin_y = grid_map.info.origin.position.y;
    double resolution = grid_map.info.resolution;
    double width = grid_map.info.width;
    double height = grid_map.info.height;

    double x = (TMP_GOAL->pose.position.x - origin_x) / resolution;
    double y = (TMP_GOAL->pose.position.y - origin_y) / resolution;
    int radius = max_radius / resolution;

    for (int i = 1; i <= radius; ++i){
        double x_tmp = x + i;
        double y_tmp = y + i;
        for (int orientation = 0; orientation < 4; ++orientation){
            for (int k = 0; k < 2*i; ++k){
                if (orientation == 0)
                    y_tmp--;
                else if (orientation == 1)
                    x_tmp--;
                else if (orientation == 2)
                    y_tmp++;
                else
                    x_tmp++;
                int p = y_tmp * width + x_tmp;
                if (p < 0 || p >= width * height || grid_map.data[p] != 0)
                    continue;
                else{
                    x_tmp = x_tmp*resolution + origin_x;
                    y_tmp = y_tmp*resolution + origin_y;
                    ROS_INFO("find a new point at (%.1f, %.1f)", x_tmp, y_tmp);
                    STORED_POSITION = Standarize_Pose(x_tmp, y_tmp);
                    TMP_GOAL = &STORED_POSITION;
                    return true;
                }
            }
        }
    }
    return false;
}

//增大搜索半径
void decision_base::Add_Max_Radius()
{
    max_radius += 0.5;
}

//初始化最大半径
void decision_base::Init_Max_Radius()
{
    max_radius = 0.5;
}

//更新当前位置信息
void decision_base::Set_Current_Position(std::string posi)
{
    current_pos = posi;
}

//获取当前位置信息
std::string decision_base::Get_Current_Position()
{
    return current_pos;
}

//设置栅格地图
void decision_base::Set_Grid_Map(const nav_msgs::OccupancyGrid& map)
{
    grid_map = map;
}

//追踪机器人设置临时点位
void decision_base::Set_Temp_Goal(double x, double y, bool del)
{
    geometry_msgs::PoseStamped *p = TMP_GOAL;

    geometry_msgs::PoseStamped posi = Standarize_Pose(x, y);
    TMP_GOAL = &posi;
    Modify_Goal();
    
    if (del){
        //避免内存泄露
        delete p;
    }
}

//获取追踪机器人的位置
bool decision_base::Get_Trace_Robot_Pos(double &x, double &y, std::string name)
{
    if (name == "enemy_guard"){
        if (enemy_guard.empty()){
            return false;
        }
        x = enemy_guard.back().robot_pos.x;
        y = enemy_guard.back().robot_pos.y;
        return true;
    }
}

//更新哨兵坐标
void decision_base::Update_Current_Pos(double x, double y)
{
    pos = Point(x, y);
}

//更新裁判系统消息
void decision_base::Update_Referee_Msg(const referee_system::referee_system &msg)
{
    RefereeMsg referee_msg(msg);
    //将最新消息插入队尾
    recent_referee[0].push_back(referee_msg);
    recent_referee[1].push_back(referee_msg);
    //维持队列长度
    while (recent_referee[0].front().remaining_time - referee_msg.remaining_time > 3)
        recent_referee[0].pop_front();
    while (recent_referee[1].front().remaining_time - referee_msg.remaining_time > 5)
        recent_referee[1].pop_front();

    //更新统计信息
    elive = 0;
    mlive = 0;
    
    ally_shut = 0;
    ally_blood = 0;
    ally_blood_decrease = 0;
    ally_live = 0;
    my_shut = 0;
    my_blood = 0;
    my_blood_decrease = 0;

    enemy_blood = 0;
    enemy_blood_decrease = 0;
    enemy_manual_live = 0;
    enemy_manual_blood = 0;
    enemy_manual_blood_decrease = 0;
    
    //我方3号步兵
    if (referee_msg.my_robot3 > 0)
        ally_live++;
    ally_shut += recent_referee[0].front().my_robot3_bullet - referee_msg.my_robot3_bullet;
    ally_blood += referee_msg.my_robot3;
    ally_blood_decrease += recent_referee[0].front().my_robot3 - referee_msg.my_robot3;
    //敌方3号步兵
    if (referee_msg.enemy_robot3 > 0)
        enemy_manual_live++;
    enemy_manual_blood += referee_msg.enemy_robot3;
    enemy_manual_blood_decrease += recent_referee[0].front().enemy_robot3 - referee_msg.enemy_robot3;

    //我方4号步兵
    if (type == 0 || type == 1){
        if (referee_msg.my_robot4 > 0)
            ally_live++;
        ally_shut += recent_referee[0].front().my_robot4_bullet - referee_msg.my_robot4_bullet;
        ally_blood += referee_msg.my_robot4;
        ally_blood_decrease += recent_referee[0].front().my_robot4 - referee_msg.my_robot4;
    }
    //我方英雄机器人
    else{
        if (referee_msg.my_robot1 > 0)
            ally_live++;
        ally_shut += recent_referee[0].front().my_robot1_bullet - referee_msg.my_robot1_bullet;
        ally_blood += referee_msg.my_robot1;
        ally_blood_decrease += recent_referee[0].front().my_robot1 - referee_msg.my_robot1;
    }
    //敌方4号步兵
    if (type == 0 || type == 3){
        if (referee_msg.enemy_robot4 > 0)
            enemy_manual_live++;
        enemy_manual_blood += referee_msg.enemy_robot4;
        enemy_manual_blood_decrease += recent_referee[0].front().enemy_robot4 - referee_msg.enemy_robot4;
    }
    //敌方英雄机器人
    else{
        if (referee_msg.enemy_robot1 > 0)
            enemy_manual_live++;
        enemy_manual_blood += referee_msg.enemy_robot1;
        enemy_manual_blood_decrease += recent_referee[0].front().enemy_robot1 - referee_msg.enemy_robot1;
    }
    //我方哨兵
    if (referee_msg.my_guard_blood > 0)
        mlive++;
    my_shut = ally_shut + recent_referee[0].front().my_guard_bullet - referee_msg.my_guard_bullet;
    my_blood = ally_blood + referee_msg.my_guard_blood;
    my_blood_decrease = ally_blood_decrease + recent_referee[0].front().my_guard_blood - referee_msg.my_guard_blood;
    //敌方哨兵
    if (referee_msg.enemy_guard_blood > 0)
        elive++;
    enemy_blood = enemy_blood + referee_msg.enemy_guard_blood;
    enemy_blood_decrease = enemy_manual_blood_decrease + recent_referee[0].front().enemy_guard_blood
                            -referee_msg.enemy_guard_blood;
    
    
}

//到达目标点
bool decision_base::Close_Enough()
{
    ROS_INFO("(%f, %f), (%f, %f)", pos.x, pos.y, GOAL->pose.position.x, GOAL->pose.position.y);
    double dist_square = (pos.x - GOAL->pose.position.x)*(pos.x - GOAL->pose.position.x)
            + (pos.y - GOAL->pose.position.y)*(pos.y - GOAL->pose.position.y);
    double min_square = min_dist*min_dist;

    if (dist_square <= min_square){
        return true;
    }
    else{
        return false;
    }
}

//初始化静态成员变量
void decision_base::Init_Static_Variable()
{
    //目标点
    BENEFIT_LEFT = Standarize_Pose(Point(0, 1));
    MIDDLE = Standarize_Pose(Point(0, 0));
    MIDDLE_LEFT = Standarize_Pose(Point(0, 3));
    MIDDLE_RIGHT = Standarize_Pose(Point(0, -3));

    MY_RECHARGE = Standarize_Pose(Point(-5, 4));
    MY_BASE_ENTRY = Standarize_Pose(Point(-3, -1));
    MY_BASE_UP_LEFT = Standarize_Pose(Point(-3, -2));
    MY_BASE_UP_RIGHT = Standarize_Pose(Point(-3.5, -3.5));
    MY_BASE_DOWN_LEFT = Standarize_Pose(Point(-5.5, -1.5));
    MY_BASE_DOWN_RIGHT = Standarize_Pose(Point(-5.5, -3.5));
    MY_WALL_LEFT = Standarize_Pose(Point(-1, 2.5));
    MY_WALL_RIGHT = Standarize_Pose(Point(-1, -2));
    MY_WALL_BACK_LEFT = Standarize_Pose(Point(-2, 3));
    MY_WALL_BACK_MID = Standarize_Pose(Point(-2, 0));
    MY_WALL_BACK_RIGHT = Standarize_Pose(Point(-2, -2));
    MY_DOMAIN_LEFT = Standarize_Pose(Point(-3, 1.5));
    MY_DOMAIN_MID = Standarize_Pose(Point(-3, 0));
    MY_DOMAIN_RIGHT = Standarize_Pose(Point(-3, -0.5));
    
    ENEMY_WALL_RIGHT = Standarize_Pose(Point(1.5, -3));
    ENEMY_WALL_LEFT = Standarize_Pose(Point(1.5, 2));
    ENEMY_WALL_FRONT_LEFT = Standarize_Pose(Point(2.5, 1));
    ENEMY_WALL_FRONT_MIDDLE = Standarize_Pose(Point(2.5, -0.5));
    ENEMY_WALL_FRONT_RIGHT = Standarize_Pose(Point(3, -3));
    ENEMY_BASE_ENTRY = Standarize_Pose(Point(3.5, 2));
    
    
    //最小到达距离
    min_dist = 0.5;
    //最大浮动半径
    max_radius = 0.5;
    //刚开局阶段
    beginning_state = true;
    //恢复血量
    recover_blood = 0;

    //开局状态锁
    beginning_state_locked = false;
    //占领增益区
    occupy_benefit = false;

    GOAL = &ENEMY_WALL_FRONT_RIGHT;
}

//设置目标
void decision_base::Set_Goal(std::string msg)
{
    if (msg == "benefit_left"){
        TMP_GOAL = &BENEFIT_LEFT;
    }
    else if (msg == "middle"){
        TMP_GOAL = &MIDDLE;
    }
    else if (msg == "middle_left"){
        TMP_GOAL = &MIDDLE_LEFT;
    }
    else if (msg == "middle_right"){
        TMP_GOAL = &MIDDLE_RIGHT;
    }
    else if (msg == "my_recharge"){
        TMP_GOAL = &MY_RECHARGE;
    }
    else if (msg == "my_base_entry"){
        TMP_GOAL = &MY_BASE_ENTRY;
    }
    else if (msg == "my_base_up_left"){
        TMP_GOAL = &MY_BASE_UP_LEFT;
    }
    else if (msg == "my_base_up_right"){
        TMP_GOAL = &MY_BASE_UP_RIGHT;
    }
    else if (msg == "my_base_down_left"){
        TMP_GOAL = &MY_BASE_DOWN_LEFT;
    }
    else if (msg == "my_base_down_right"){
        TMP_GOAL = &MY_BASE_DOWN_RIGHT;
    }
    else if (msg == "my_wall_left"){
        TMP_GOAL = &MY_WALL_LEFT;
    }
    else if (msg == "my_wall_right"){
        TMP_GOAL = &MY_WALL_RIGHT;
    }
    else if (msg == "my_wall_back_left"){
        TMP_GOAL = &MY_WALL_BACK_LEFT;
    }
    else if (msg == "my_wall_back_mid"){
        TMP_GOAL = &MY_WALL_BACK_MID;
    }
    else if (msg == "my_wall_back_right"){
        TMP_GOAL = &MY_WALL_BACK_RIGHT;
    }
    else if (msg == "my_domain_left"){
        TMP_GOAL = &MY_DOMAIN_LEFT;
    }
    else if (msg == "my_domain_mid"){
        TMP_GOAL = &MY_DOMAIN_MID;
    }
    else if (msg == "my_domain_right"){
        TMP_GOAL = &MY_DOMAIN_RIGHT;
    }
    else if (msg == "enemy_wall_left"){
        TMP_GOAL = &ENEMY_WALL_LEFT;
    }
    else if (msg == "enemy_wall_right"){
        TMP_GOAL = &ENEMY_WALL_RIGHT;
    }
    else if (msg == "enemy_wall_front_left"){
        TMP_GOAL = &ENEMY_WALL_FRONT_LEFT;
    }
    else if (msg == "enemy_wall_front_right"){
        TMP_GOAL = &ENEMY_WALL_FRONT_RIGHT;
    }
    else if (msg == "enemy_wall_front_middle"){
        TMP_GOAL = &ENEMY_WALL_FRONT_MIDDLE;
    }
    else if (msg == "enemy_base_entry"){
        TMP_GOAL = &ENEMY_BASE_ENTRY;
    }
    else{
        ROS_WARN("[%s] is not valid", msg.c_str());
        throw std::runtime_error("[" + msg + "] is not area_b keyword");
    }
    Modify_Goal();
}

//设置坐标偏移量
void decision_base::Set_Offset(double x, double y)
{
    Point::offset_x = x;
    Point::offset_y = y;
}

//判断是否结束
bool decision_base::Is_Game_Over()
{
    return recent_referee[0].back().remaining_time <= 0;
}

//判断是否为刚开局阶段
bool decision_base::Is_Beginning_State()
{
    return beginning_state;
}

//结束开局状态
void decision_base::End_Beginning_State()
{
    beginning_state = false;
}

//敌方哨兵阵亡
bool decision_base::Is_Enemy_Guard_Killed()
{
    return recent_referee[0].back().enemy_guard_blood == 0;
}

//我方哨兵残血
bool decision_base::My_Guard_Hurt()
{
    return recent_referee[0].back().my_guard_blood <= 100;
}

//劣势
bool decision_base::Inferior_Situation()
{
    if (beginning_state){
        //我方盟友血量和低于100 敌方手动机器人血量和大于250
        if (ally_blood < 100 && enemy_manual_blood > 250)
            return true;
        //我方盟友存活数少于敌方 我方盟友血量减少比敌方减少大于30
        if (ally_live < enemy_manual_live && ally_blood_decrease - enemy_manual_blood_decrease > 30)
            return true;
        //我方盟友存活数少于敌方 敌方血量为我方两倍
        if (ally_live < enemy_manual_live && enemy_manual_blood / double(ally_blood+1) > 2)
            return true;
        //敌方哨兵血量比我方多300 我方哨兵血量低于200 我方盟友血量低于敌方
        if (recent_referee[0].back().enemy_guard_blood - recent_referee[0].back().my_guard_blood > 300 
            && recent_referee[0].back().my_guard_blood < 200 && ally_blood < enemy_manual_blood)
            return true;
        return false;
    }
    else{
        //我方盟友存活数少于敌方 敌方血量血量为2倍
        if (ally_live < enemy_manual_live && enemy_manual_blood >= ally_blood*2)
        {
            return true;
        }
        //敌方手动机器人血量多200 或者血量为3倍
        if (enemy_manual_blood - ally_blood > 200 || enemy_manual_blood >= ally_blood *3)
        {
            return true;
        }
        return false;
    }
}

//优势
bool decision_base::Favorable_Situation()
{
    if (enemy_manual_live < ally_live && ally_blood >= 2*enemy_manual_blood)
        return true;
    if (ally_blood - enemy_manual_blood > 200 || ally_blood >= enemy_manual_blood*3)
        return true;
    return false;
}

//敌方哨兵消失
bool decision_base::Enemy_Guaed_Dissappear()
{
    if (Is_Enemy_Guard_Killed()){
        return false;
    }
    return enemy_guard.empty();
}

//队友在一起攻击
bool decision_base::Ally_Stay_by()
{
    //队友阵亡
    if (ally_live - 1 <= 0)
        return false;
    
    //队友不在攻击 撤退
    if (type == 0 || type == 1){
        //友军攻击频率低
        if (ally_shut / (mlive - 1) < 10)
            return false;
    }
    else{
        //我方3号步兵阵亡 或友方攻击频率低
        if (recent_referee[0].back().my_robot3 == 0 || ally_shut < 10)
            return false;
    }

    return true;
}

//攻击效率低
bool decision_base::Low_Attack_Efficience()
{
    //子弹不够75
    if (recent_referee[0].back().my_guard_bullet <= 75)
        return false;
    
    //5s内发射子弹数低于5
    if (recent_referee[1].front().my_guard_bullet - recent_referee[1].back().my_guard_bullet <= 5)
        return true;
}

//亚健康
bool decision_base::Sub_Health()
{
    return recent_referee[0].back().my_base_blood <= 300;
}

//血量恢复达到上限
bool decision_base::Recover_Limitted()
{
    return recover_blood >= 600;
}

//敌方手动机器人阵亡
bool decision_base::Enemy_Manual_Killed()
{
    return enemy_manual_live == 0;
}

//哨兵攻击较强
bool decision_base::Attacking_Ragingly()
{
    return recent_referee[0].front().my_guard_bullet - recent_referee[0].back().my_guard_bullet >= 12;
}

//基地被攻击
bool decision_base::Base_Been_Attacked()
{
    //判断我方基地是否被攻击
    bool attacked = false;
    //基地血量或者护盾量减少
    if (recent_referee[0].front().my_base_blood - recent_referee[0].back().my_base_blood > 0
        || recent_referee[0].front().my_base_shield - recent_referee[0].back().my_base_shield > 0)
        attacked = true;
    
    //我方基地没有被攻击
    if  (!attacked)
        return false;
    //我方哨兵正在攻击敌方
    if (!decision_base::Attacking_Ragingly())
        return false;
    //敌方为双步兵阵容 我方基地护盾还在 友军血量大于50
    if ((type == 0 || type == 2) && recent_referee[0].back().my_base_shield > 0 && ally_blood >= 50)
        return false;
    return true;
}

//敌方攻击很慢
bool decision_base::Enemy_Attack_Slowly()
{
    int ave = elive == 0 ? 0 : my_blood_decrease / elive;
    return ave <= 15;
}

//血量低于150
bool decision_base::Low_Blood()
{
    return recent_referee[0].back().my_guard_blood <= 150;
}

//友方状态差 血量和小于150
bool decision_base::Ally_Poor_State()
{
    if (ally_blood <= 150)
        return true;
    return false;
}

//攻击中
bool decision_base::Attacking()
{
    return recent_referee[0].back().my_guard_bullet - recent_referee[0].front().my_guard_bullet > 6;
}

//发现敌方机器人
bool decision_base::Find_Enemy(std::string name)
{
    if (name == "enemy_r1"){
        return enemy_r1.empty();
    }
    else if (name == "enemy_r3"){
        return enemy_r3.empty();
    }
    else if (name == "enemy_r4"){
        return enemy_r4.empty();
    }
    else if (name == "enemy_guard"){
        return enemy_guard.empty();
    }
}

//是否占领增益区
bool decision_base::Whther_Occupy_Benefit()
{
    if (!recent_referee[0].back().benefit_open){
        occupy_benefit = false;
        return false;
    }

    if (occupy_benefit){
        return true;
    }
    else if (Favorable_Situation()){
        if (pos.x > 1.5 && recent_referee[0].back().my_guard_blood < 250){
            occupy_benefit = true;
            return true;
        }
        else if (pos.x <= 1.5 && pos.x >= -1.5){
            occupy_benefit = true;
            return true;
        }
    }
    else if (pos.x <= 1.5 && pos.x >= -1.5){
        occupy_benefit = true;
        return true;
    }
    else if (!Inferior_Situation() && recent_referee[0].back().my_guard_blood >= 250){
        occupy_benefit = true;
        return true;
    }
}

//血量少于250
bool decision_base::Blood_Less_200()
{
    if (recent_referee[0].back().my_guard_blood <= 200){
        return true;
    }
    else{
        return false;
    }
}

//血量恢复完成
bool decision_base::Recover_Finished()
{
    if (recover_blood >= 580){
        return true;
    }
    else if (ally_blood <= 80 && recent_referee[0].back().my_guard_blood >= 400){
        return true;
    }
    else if (recent_referee[0].back().my_guard_blood >= 550){
        return true;
    }
    else{
        return false;
    }
}

//状态锁
bool decision_base::state_locked()
{
    return beginning_state_locked;
}

//锁住状态
void decision_base::lock_state()
{
    beginning_state_locked = true;
}

//敌方哨兵出现在左方
bool decision_base::Enemy_Guard_At_Left()
{
    if (!enemy_guard.empty() && enemy_guard.back().robot_pos.x >= 0 
        && enemy_guard.back().robot_pos.x < 1.5)
        return true;
    return false;
}

//敌方哨兵撤退
bool decision_base::Enemy_Guard_RetratL()
{   
    //返回距离大于1.5
    if (enemy_guard.back().robot_pos.x - enemy_guard.front().robot_pos.x > 1.5)
        return true;
    //位置过于靠后
    if (enemy_guard.back().robot_pos.x >= 1.5)
        return true;
    return false;
}

//敌方哨兵在我方区域
bool decision_base::Enemy_Guard_In_My_Domain()
{
    //x < -1.5
    if (!enemy_guard.empty() && enemy_guard.back().robot_pos.x < -1.5)
        return true;
    return false;
}

//敌方哨兵在敌方区域
bool decision_base::Enemy_Guard_In_Enemy_Doamin()
{
    //x > 1.5
    if (!enemy_guard.empty() && enemy_guard.back().robot_pos.x > 1.5)
        return true;
    return false;
}

//敌方哨兵位置未知
bool decision_base::Enemy_Guard_In_Unknow_Domain()
{
    //视觉信息为空
    if (enemy_guard.empty())
        return true;
    //x介于 [-1.5,1.5]
    else if (enemy_guard.back().robot_pos.x <= 1.5 && enemy_guard.back().robot_pos.x >= -1.5)
        return true;
    return false;
}

//敌方哨兵在正前方
bool decision_base::Enemy_Guard_In_Front()
{
    //y <= -2 & x > 0
    if (!enemy_guard.empty() && enemy_guard.back().robot_pos.y <= -2 && enemy_guard.back().robot_pos.x > 0)
        return true;
    else
        return false;
}

//敌方哨兵前进
bool decision_base::Enemy_Guard_Advanced()
{
    //x变化小于 -1.5
    if (enemy_guard.back().robot_pos.x - enemy_guard.front().robot_pos.x < -1.5)
        return true;
    //x < 0
    if (enemy_guard.back().robot_pos.x < 0)
        return true;
    //y > -1
    if (enemy_guard.back().robot_pos.y > -1)
        return true;
    return false;
}

//敌方哨兵撤退
bool decision_base::Enemy_Guard_Retrait_R()
{
    //x > 3
    if (enemy_guard.back().robot_pos.x > 3)
        return true;
    //x变化大于 1.5
    if (enemy_guard.back().robot_pos.x - enemy_guard.front().robot_pos.x > 1.5)
        return true;
    //y变化大于1 x>1.5
    if (enemy_guard.back().robot_pos.y - enemy_guard.front().robot_pos.y > 1 && enemy_guard.back().robot_pos.x > 1.5)
        return true;
    return false;
}

//位置过半
bool decision_base::Posi_Over_Half()
{
    return pos.x > 0;
}

//敌方哨兵撤退
bool decision_base::Enemy_Guard_RetratR()
{   
    //返回距离大于1.5
    if (enemy_guard.back().robot_pos.x - enemy_guard.front().robot_pos.x > 1.5)
        return true;
    //位置过于靠后
    if (enemy_guard.back().robot_pos.x >= 3)
        return true;
    //往掩体旁撤退
    if (enemy_guard.back().robot_pos.x >= 1.5 && enemy_guard.back().robot_pos.y >= -2)
        return true;
    //deltX > 1.5 deltY > 1 X > 0.5
    if (enemy_guard.back().robot_pos.x >= 0.5 && 
        enemy_guard.back().robot_pos.x - enemy_guard.front().robot_pos.x >= 1.5 &&
        enemy_guard.back().robot_pos.y - enemy_guard.front().robot_pos.y >= 1)
        return true;    
    return false;
}

//血量高于200
bool decision_base::Blood_Over_200()
{
    return recent_referee[0].back().my_guard_blood >= 200;
}

//在我方区域右方发现敌方
bool decision_base::Enemy_Guard_In_My_Domain_Right()
{
    //x<=-1.5  y<1
    if (!enemy_guard.empty() && enemy_guard.back().robot_pos.x <= -1.5 
        && enemy_guard.back().robot_pos.y < 1)
        return true;
    return false;
}

//在中间靠右发现敌方哨兵
bool decision_base::Enemy_Guard_In_Middle_Right()
{
    //x < 1.5  y < 1  x > -1.5
    if (!enemy_guard.empty() && enemy_guard.back().robot_pos.x < 1.5 
        && enemy_guard.back().robot_pos.y < 1 && enemy_guard.back().robot_pos.x > -1.5)
        return true;
    return false;
}

//基地被攻击
bool decision_base::Bese_Been_Attacked()
{
    bool attacked = false;
    if (recent_referee[0].front().my_base_blood - recent_referee[0].back().my_base_blood > 0
        || recent_referee[0].front().my_base_shield - recent_referee[0].back().my_base_shield > 0){
        attacked = true;
    }
    
    //基地没有被攻击
    if (!attacked){
        return false;
    }

    //哨兵正在攻击敌方
    if (recent_referee[0].back().my_guard_bullet - recent_referee[0].front().my_guard_bullet >= 10){
        return false;
    }

    //敌方为双步兵阵容 我方基地护盾还在 友军血量大于50
    if ((type == 0 || type == 2) && ally_blood >= 50
        && recent_referee[0].back().my_base_protected){
        return false;
    }

    //敌方有英雄 但英雄未参与进攻 友军血量大于50
    if ((type == 1 || type == 3) && ally_blood >= 50
        && (recent_referee[0].back().my_base_blood-recent_referee[0].front().my_base_blood < 100
        && recent_referee[0].back().my_base_shield - recent_referee[0].front().my_base_shield < 100)){
        return false;
    }

    return true;
}

//寻找徘徊点
void decision_base::Set_Wander_Point()
{
    double origin_x = grid_map.info.origin.position.x;
    double origin_y = grid_map.info.origin.position.y;
    double resolution = grid_map.info.resolution;
    double width = grid_map.info.width;
    double height = grid_map.info.height;

    double x = (TMP_GOAL->pose.position.x - origin_x) / resolution;
    double y = (TMP_GOAL->pose.position.y - origin_y) / resolution;

    bool flag[grid_map.data.size()] = {false};

}

void decision_base::Is_Wander_Point_Valid(double x, double y)
{

}

void decision_base::Seek_Wander_Point(double x, double y, bool flag[])
{
    
}