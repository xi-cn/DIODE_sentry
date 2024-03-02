#ifndef SEE_ENEMY_NODE
#define SEE_ENEMY_NODE

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

//敌方哨兵来我方补给区干扰
NodeStatus EnemyGuardCome();

//敌方哨兵在敌方补给区前防护
NodeStatus EnemyGuardDefend();

#endif