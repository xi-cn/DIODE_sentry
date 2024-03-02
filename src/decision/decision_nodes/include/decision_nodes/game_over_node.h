#ifndef GAME_OVER_NODE
#define GAME_OVER_NODE

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

//判断比赛是否结束
NodeStatus IsGameOver();

#endif