#ifndef MOVE_BASE_NODE
#define MOVE_BASE_NODE

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

class MoveBaseNode : public BT::StatefulActionNode
//移动节点 移动到目标点
{
public:
    MoveBaseNode(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("goal") };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    //前往位置
    std::string goal;
};

#endif