#include "decision_nodes/simple_condition_node.h"

//ros正常进行
NodeStatus RosOk()
{
    if (ros::ok()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//位置过半
NodeStatus PositionOverHalf()
{
    if (decision_base::Posi_Over_Half()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//哨兵残血
NodeStatus GuardHurt()
{
    if (decision_base::My_Guard_Hurt()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//劣势
NodeStatus InferiorSituation()
{
    if (decision_base::Inferior_Situation()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//和队友一起作战
NodeStatus AllyStayBy()
{
    if (decision_base::Ally_Stay_by()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//敌方哨兵阵亡
NodeStatus EnemyGuardKilled()
{
    if (decision_base::Is_Enemy_Guard_Killed()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//敌方攻击弱
NodeStatus EnemyAttackSlowly()
{
    if (decision_base::Enemy_Attack_Slowly()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//血量低于150
NodeStatus LowBlood()
{
    if (decision_base::Low_Blood()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//敌方哨兵消失
NodeStatus EnemyGuardDisappear()
{
    if (decision_base::Enemy_Guaed_Dissappear()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//敌方哨兵在我方区域
NodeStatus EnemyGuardInMyDomain()
{
    if (decision_base::Enemy_Guard_In_My_Domain()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//敌方哨兵在敌方区域
NodeStatus EnemyGuardInEnemyDomain()
{
    if (decision_base::Enemy_Guard_In_Enemy_Doamin()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//敌方哨兵在未知区域
NodeStatus EnemyGuardInUnknowDomain()
{
    if (decision_base::Enemy_Guard_In_Unknow_Domain()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//敌方撤退
NodeStatus EnemyGuardRetraitL()
{
    if (decision_base::Enemy_Guard_RetratL()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//攻击效率低
NodeStatus LowEfficience()
{
    if (decision_base::Low_Attack_Efficience()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//敌方撤退
NodeStatus EnemyGuardRetraiR()
{
    if (decision_base::Enemy_Guard_RetratR()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//友方状态差 血量小于150
NodeStatus AllyPoorState()
{
    if (decision_base::Ally_Poor_State()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//哨兵血量大于200
NodeStatus BloodOver200()
{
    if (decision_base::Blood_Over_200()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//锁住状态
NodeStatus StateLocked()
{
    if (decision_base::state_locked()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//攻击中
NodeStatus Attacking()
{
    if (decision_base::Attacking()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//基地被攻击
NodeStatus BaseBeenAttacked()
{
    if (decision_base::Base_Been_Attacked()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//占领增益区
NodeStatus WhtherOccupyBenefit()
{
    if (decision_base::Whther_Occupy_Benefit()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//血量低于250
NodeStatus BloodLess200()
{
    if (decision_base::Blood_Less_200()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//血量恢复完成
NodeStatus RecoverFinished()
{
    if (decision_base::Recover_Finished()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}

//优势
NodeStatus FavorableSituation()
{
    if (decision_base::Favorable_Situation()){
        return NodeStatus::SUCCESS;
    }
    else{
        return  NodeStatus::FAILURE;
    }
}