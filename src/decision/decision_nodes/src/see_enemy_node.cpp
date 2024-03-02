#include "decision_nodes/see_enemy_node.h"

//敌方哨兵来我方补给区干扰
NodeStatus EnemyGuardCome()
{
    return NodeStatus::SUCCESS;
    if (decision_base::Enemy_Guard_At_Left()){
        ROS_INFO("enemy guard is comming");
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//敌方哨兵在敌方补给区前防护
NodeStatus EnemyGuardDefend()
{
    if (decision_base::Enemy_Guard_In_Front()){
        ROS_INFO("enemy guard is in the front");
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}