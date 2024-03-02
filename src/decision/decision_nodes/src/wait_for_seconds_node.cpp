#include "decision_nodes/wait_for_seconds_node.h"

NodeStatus WaitForSecondsNode::onStart()
{
    //decision_base::Stay_Put();
    int sec = 0;
    auto msg = getInput<int>("sec", sec);
    if (!sec)
    {
        sec = 5;
    }
    ROS_INFO("start to stay put for %d seconds", sec);
    //设置目标点为当前位置
    decision_base::Stay_Put();
    //计算deadline
    using namespace std::chrono;
    deadline_ = system_clock::now() + milliseconds(sec*1000);
    return NodeStatus::RUNNING;
}

NodeStatus WaitForSecondsNode::onRunning()
{
    if (std::chrono::system_clock::now() >= deadline_)
    {
        return NodeStatus::SUCCESS;
    }
    else
    {
        return NodeStatus::RUNNING;
    }
}

void WaitForSecondsNode::onHalted()
{
    ROS_INFO("timeout");
}