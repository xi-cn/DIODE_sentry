#include "decision_nodes/game_over_node.h"

//判断比赛是否结束
NodeStatus IsGameOver()
{
    if (decision_base::Is_Game_Over()){
        return NodeStatus::SUCCESS;
    }
    else{
        return NodeStatus::FAILURE;
    }
}