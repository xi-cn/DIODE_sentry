#ifndef BEGINNING_STATE_NODE
#define BEGINNING_STATE_NODE

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

//判断是否在刚开局阶段
NodeStatus IsBeginningState();

#endif