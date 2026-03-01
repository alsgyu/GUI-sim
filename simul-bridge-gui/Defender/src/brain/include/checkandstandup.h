#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

/*
    해당 파일은 MoveHead을 위한 노드와 함수만 모아놓은 헤더 파일입니다.
*/

void RegisterCheckAndStandUpNodes(BT::BehaviorTreeFactory &factory, Brain* brain); // 노드 등록을 위한 함수

class CheckAndStandUp : public SyncActionNode
{
public:
CheckAndStandUp(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};