#include "decision_nodes/simple_action_node.h"

//锁住状态
NodeStatus LockState()
{
    decision_base::lock_state();
    return NodeStatus::SUCCESS;
}

//结束开局状态
NodeStatus EndBeginningState()
{
    decision_base::End_Beginning_State();
    return NodeStatus::SUCCESS;
}