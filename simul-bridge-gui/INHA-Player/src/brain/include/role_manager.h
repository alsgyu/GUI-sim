#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain;
using namespace BT;

void RegisterRoleManager(BT::BehaviorTreeFactory &factory, Brain* brain);

class RoleManager : public SyncActionNode {
public:
    RoleManager(const std::string& name, const NodeConfig& config, Brain* _brain);

    static PortsList providedPorts() {
        return {
            OutputPort<std::string>("current_role") // goal_keeper/lead_striker/sub_striker/lead_defender/sub_defender
        };
    }

    NodeStatus tick() override;

private:
    Brain* brain;
    rclcpp::Time timeLastTick;
};

class SetPieceRoleManager : public SyncActionNode {
public:
    SetPieceRoleManager(const std::string& name, const NodeConfig& config, Brain* _brain);

    static PortsList providedPorts() {
        return {
            OutputPort<std::string>("current_role")
        };
    }

    NodeStatus tick() override;

private:
    Brain* brain;
};
