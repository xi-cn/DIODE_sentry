#ifndef FIND_ENEMY
#define FIND_ENEMY

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

class FindEnemy : public BT::ConditionNode
{
public:
    FindEnemy(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("robot_name") };
    }

    NodeStatus tick() override;
};

#endif