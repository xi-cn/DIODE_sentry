#include "decision_nodes/beginning_state_node.h"

//判断是否在刚开局阶段
NodeStatus IsBeginningState()
{
    if (decision_base::Is_Beginning_State()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}