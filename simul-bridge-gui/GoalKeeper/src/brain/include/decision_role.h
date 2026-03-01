#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterDecisionRoleNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
            InputPort<double>("closer_margin", 0.2, "내가 더 가깝다고 판정할 여유분"),
            InputPort<double>("clearing_max", 2.0, "이 이상 멀어지면 clearing 종료하고 hold 복귀"), 
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class GoalieClearingDecide : public SyncActionNode
{
public:
    GoalieClearingDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
            InputPort<double>("chase_threshold", 0.25, "이 거리보다 멀면 공 추격(Chase) 동작을 실행"),
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

