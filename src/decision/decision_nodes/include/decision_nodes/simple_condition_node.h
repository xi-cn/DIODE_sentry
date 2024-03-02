#ifndef SIMPLE_CONDITION_NODE
#define SIMPLE_CONDITION_NODE

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

//ros正常进行
NodeStatus RosOk();

//位置过半
NodeStatus PositionOverHalf();

//哨兵残血
NodeStatus GuardHurt();

//劣势
NodeStatus InferiorSituation();

//和队友一起作战
NodeStatus AllyStayBy();

//敌方哨兵阵亡
NodeStatus EnemyGuardKilled();

//敌方攻击弱
NodeStatus EnemyAttackSlowly();

//血量低于150
NodeStatus LowBlood();

//敌方哨兵消失
NodeStatus EnemyGuardDisappear();

//敌方哨兵在我方区域
NodeStatus EnemyGuardInMyDomain();

//敌方哨兵在敌方区域
NodeStatus EnemyGuardInEnemyDomain();

//敌方哨兵在未知区域
NodeStatus EnemyGuardInUnknowDomain();

//敌方撤退
NodeStatus EnemyGuardRetraitL();

//攻击效率低
NodeStatus LowEfficience();

//敌方撤退
NodeStatus EnemyGuardRetraiR();

//友方状态差 血量小于150
NodeStatus AllyPoorState();

//哨兵血量大于200
NodeStatus BloodOver200();

//开局状态锁
NodeStatus BeginingStateLocked();

//锁住状态
NodeStatus StateLocked();

//攻击中
NodeStatus Attacking();

//基地被攻击
NodeStatus BaseBeenAttacked();

//占领增益区
NodeStatus WhtherOccupyBenefit();

//血量低于250
NodeStatus BloodLess200();

//血量恢复完成
NodeStatus RecoverFinished();

//优势
NodeStatus FavorableSituation();

#endif