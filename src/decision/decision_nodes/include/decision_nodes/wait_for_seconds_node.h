#ifndef WAIT_FOR_SECONDS_NODE
#define WAIT_FOR_SECONDS_NODE

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

class WaitForSecondsNode : public BT::StatefulActionNode
{
public:
    WaitForSecondsNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("sec") };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    std::chrono::system_clock::time_point deadline_;
};

#endif