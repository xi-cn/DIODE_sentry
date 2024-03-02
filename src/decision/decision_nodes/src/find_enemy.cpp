#include "decision_nodes/find_enemy.h"

NodeStatus FindEnemy::tick()
{
    std::string name;
    auto msg = getInput<std::string>("goal", name);
    if (!msg)
    {
        ROS_ERROR("missing required input [robot_name]: %s", msg.error().c_str());
        throw BT::RuntimeError( "missing required input [robot_name]: ", msg.error() );
    }

    if (decision_base::Find_Enemy(name)){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}