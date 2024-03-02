#ifndef NEAR_POSITION_NODE
#define NEAR_POSITION_NODE

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

class NearPositionNode : public BT::ConditionNode
{
public:
    NearPositionNode(const std::string &name, const BT::NodeConfig& config)
        : ConditionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("pos") };
    }

    NodeStatus tick() override;
};

#endif