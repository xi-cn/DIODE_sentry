#ifndef WALKING_AROUND_NODE
#define WALKING_AROUND_NODE

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

class WalkingAround : public BT::StatefulActionNode
//原地徘徊
{
public:
    WalkingAround(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
    {
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:

};

#endif