#include "decision_nodes/near_position_node.h"

NodeStatus NearPositionNode::tick()
{
    std::string pos;
    auto msg = getInput<std::string>("pos", pos);
    if (!msg)
    {
        throw BT::RuntimeError( "missing required input [pos]: ", msg.error() );
    }

    if (decision_base::Get_Current_Position() == pos){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}