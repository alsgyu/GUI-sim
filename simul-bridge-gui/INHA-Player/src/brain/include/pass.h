#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain;
using namespace BT;

void RegisterPassNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class CalcPassDir : public SyncActionNode {
public:
    CalcPassDir(const std::string& name, const NodeConfig &config, Brain* brain)
        : SyncActionNode(name, config), brain(brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("max_pass_threshold", 4.0, "팀원과의 최대 패스 거리"),
            InputPort<double>("min_pass_threshold", 1.0, "팀원과의 최소 패스 거리"),
            InputPort<double>("score_threshold", -5.0, "패스 결정을 위한 최소 점수"),
            OutputPort<bool>("pass_found"),
            OutputPort<double>("pass_speed_limit")
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};
