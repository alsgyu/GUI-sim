#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <string>

class Brain; 
using namespace BT;
using namespace std;


void RegisterOfftheballNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class OffTheBall : public SyncActionNode
{
public:
    OffTheBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_threshold", 0.1, "목표 위치 가까이 도달하면 정지"),
            InputPort<double>("v_limit", 0.5, "최대 속도"),
            InputPort<double>("dist_from_goal", 4.0, "goal 앞에서 대기할 거리"),
        };
    }

    NodeStatus tick() override;


private:
    Brain *brain;
    //std::chrono::steady_clock::time_point scanStartTime = std::chrono::steady_clock::time_point::min();
    //double smoothHeadYaw = 0.0;
};

class InitPos : public SyncActionNode
{
public:
    InitPos(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 0.35, "여기까진 직진으로 성큼성큼 오다가, 여기부터 정면 바라보도록 회전"),
            InputPort<double>("stop_threshold", 0.1, "목표 위치 가까이 도달하면 정지"),
            InputPort<double>("vx_limit", 0.6, "최대 속도"),
            InputPort<double>("vy_limit", 0.3, "최대 속도"),
            InputPort<double>("init_golie_pos_x", -2.0, "골대중앙위치... 보다 살짝 앞에"),
            InputPort<double>("init_golie_pos_y", -1.5, "골대중앙위치"),
            InputPort<double>("init_golie_pos_theta", 0.0, "골대중앙위치"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};