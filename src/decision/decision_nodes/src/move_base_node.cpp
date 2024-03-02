#include "decision_nodes/move_base_node.h"

NodeStatus MoveBaseNode::onStart()
{
    auto msg = getInput<std::string>("goal", goal);
    if (!msg)
    {
        throw BT::RuntimeError( "missing required input [goal]: ", msg.error() );
    }
    //更新目标点
    ROS_INFO("start move to %s", goal.c_str());
    decision_base::Set_Goal(goal);
    return NodeStatus::RUNNING;
}

NodeStatus MoveBaseNode::onRunning()
{
    ROS_INFO("a");
    decision_base::Modify_Goal();
    ROS_INFO("b");
    if (decision_base::Close_Enough())
    {
        decision_base::Set_Current_Position(goal);
        return NodeStatus::SUCCESS;
    }
    else
    {
        return NodeStatus::RUNNING;
    }
}

void MoveBaseNode::onHalted()
{
    ROS_INFO("successfully reach to %s", goal.c_str());
}