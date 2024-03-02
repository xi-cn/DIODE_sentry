#include "decision_nodes/trace_enemy.h"

NodeStatus TraceEnemy::onStart()
{
    auto msg = getInput<std::string>("robot_name", robot_name);
    if (!msg)
    {
        ROS_ERROR("missing required input [robot_name]: %s", msg.error().c_str());
        throw BT::RuntimeError( "missing required input [robot_name]: ", msg.error() );
    }
    //更新目标点
    ROS_INFO("start to trace %s", robot_name.c_str());
    decision_base::Get_Trace_Robot_Pos(x, y, robot_name);
    decision_base::Set_Temp_Goal(x, y, false);
    return NodeStatus::RUNNING;
}

NodeStatus TraceEnemy::onRunning()
{
    if (decision_base::Get_Trace_Robot_Pos(x, y, robot_name)){
        decision_base::Set_Temp_Goal(x, y, true);
        return NodeStatus::RUNNING;
    }
    else{
        return NodeStatus::SUCCESS;
    }
    
}

void TraceEnemy::onHalted()
{
    ROS_INFO("stop tracing %s", robot_name.c_str());
}