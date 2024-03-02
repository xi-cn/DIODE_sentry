#include "decision_nodes/_switch_node.h"

namespace BT
{
_SwitchNode::_SwitchNode(const std::string& name) :
    ControlNode::ControlNode(name, {}), child_idx_(0)
{
    setRegistrationID("Switch");
}

void _SwitchNode::halt()
{
  child_idx_ = 0;
  ControlNode::halt();
}

NodeStatus _SwitchNode::tick()
{
    if (child_idx_ != -1){
        NodeStatus status = children_nodes_[child_idx_]->status();
        if (status == NodeStatus::RUNNING){
            return NodeStatus::RUNNING;
        }
        else{
            resetChildren();
            child_idx_ = -1;
        }
    }

    const size_t children_count = children_nodes_.size();
    child_idx_ = 0;
    while (child_idx_ % 2 == 0 && child_idx_ < children_count){
        NodeStatus condition_status = children_nodes_[child_idx_]->executeTick();

        if (condition_status == NodeStatus::RUNNING){
            throw std::logic_error("The condition node must not berunning");
        }
        else if (condition_status == NodeStatus::SUCCESS){
            child_idx_ += 1;
        }
        else if (condition_status == NodeStatus::FAILURE){
            child_idx_ += 2;
        }
    }
    if (child_idx_ == children_count){
        ROS_INFO("switch node has finished, but nothing happened");
        child_idx_ = -1;
        return NodeStatus::FAILURE;
    }

    children_nodes_[child_idx_]->executeTick();
    return NodeStatus::RUNNING;
}
}