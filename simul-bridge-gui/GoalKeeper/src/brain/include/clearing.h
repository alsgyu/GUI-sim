#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterClearingNodes(BT::BehaviorTreeFactory &factory, Brain* brain);


class CalcClearingDir : public SyncActionNode
{
public:
    CalcClearingDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("goalx", -5.25, "골대 내부 중앙의 위치"),
            InputPort<double>("goaly", 0.0, "골대 내부 중앙의 위치")
        };
    }

    NodeStatus tick() override;

private:

    Brain *brain;
};


class ClearingChase : public SyncActionNode
{
public:
    ClearingChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.7, "vx의 최대 속도"),
            InputPort<double>("vy_limit", 0.6, "vx의 최대 속도"),
            InputPort<double>("vtheta_limit", 1.0, "vtheta의 최대 속도"),
            InputPort<double>("p_gain", 1.0, "P 제어의 gain"),
            InputPort<double>("dist", 0.3, "공으로부터 떨어질 목표 거리"),
            InputPort<double>("safeDist", 2.0, "원형으로 돌아 들어갈 때의 안전 반경"),
        };
    }

    NodeStatus tick() override;

private:

    Brain *brain;
};

