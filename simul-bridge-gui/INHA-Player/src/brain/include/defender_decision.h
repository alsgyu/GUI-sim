#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterDefenderDecisionNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class DefenderDecision : public SyncActionNode
{
public: 
    DefenderDecision(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "이 거리보다 멀어지면 공을 추격하는 동작을 수행"),
            InputPort<double>("clearing_threshold", 0.8, "이 거리 미만일 시 shouldClearing으로 간주"),
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir = 0.0; 
    rclcpp::Time timeLastTick; 
};

class LeadDefenderDecision : public SyncActionNode
{
public: 
    LeadDefenderDecision(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("clearing_threshold", 0.8, "이 거리 미만일 시 shouldClearing으로 간주"),
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    rclcpp::Time timeLastTick; 
};

class SubDefenderDecision : public SyncActionNode
{
public: 
    SubDefenderDecision(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    rclcpp::Time timeLastTick; 
};

class DefenderShootBlock : public SyncActionNode
{
public:
    DefenderShootBlock(const string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("ctPosx", -4.5, "수비할 골대 중앙 X"),
            InputPort<double>("ctPosy", 0.0, "수비할 골대 중앙 Y"),
            InputPort<double>("block_dist_from_ball", 1.0, "공에서 n미터 떨어진 차단 지점"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// defender_decision.h 에 DefenderShootBlock 아래에 추가
class DefenderPassBlock : public SyncActionNode
{
public:
    DefenderPassBlock(const string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};



class DefenderClearingDecide : public SyncActionNode
{
public: 
    DefenderClearingDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "이 거리보다 멀어지면 공을 추격하는 동작을 수행"),
            InputPort<string>("clearing_decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            OutputPort<string>("clearing_decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir = 0.0; 
    rclcpp::Time timeLastTick; 
};

// Renamed from CalcClearingDir (Verifier conflict with Goalie)
class CalcDefenderClearingDir : public SyncActionNode {
public:
    CalcDefenderClearingDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts(){
        return {
            InputPort<double>("offset_degree", 60.0, "볼-opponent 각도에서 추가로 꺾어차는 각도"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};