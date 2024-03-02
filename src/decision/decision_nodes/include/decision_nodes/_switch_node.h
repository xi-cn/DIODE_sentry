#ifndef SWITCH_NODE
#define SWITCH_NODE

#include "behaviortree_cpp/control_node.h"
#include <ros/ros.h>

namespace BT{

class _SwitchNode : public ControlNode
{
public:
    _SwitchNode(const std::string& name);

    virtual ~_SwitchNode() override = default;

    virtual void halt() override;

private:
  size_t child_idx_ = -1;

  virtual BT::NodeStatus tick() override;
};

}

#endif