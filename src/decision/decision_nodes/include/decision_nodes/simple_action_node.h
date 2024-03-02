#ifndef SIMPLE_ACTION_NODES
#define SIMPLE_ACTION_NODES

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

//锁住状态
NodeStatus LockState();

//结束开局状态
NodeStatus EndBeginningState();

#endif