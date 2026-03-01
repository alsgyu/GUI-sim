#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

class Brain; 
using namespace BT;

void RegisterSetpieceNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class CalcKickInPose1 : public SyncActionNode
{
public:
    CalcKickInPose1(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("block_dist", 2.0, "distance from ball to target"),
            InputPort<double>("max_age_sec", 5.0, "ignore opponents older than this"),
            OutputPort<bool>("block_found"),
            OutputPort<double>("block_target_x"),
            OutputPort<double>("block_target_y"),
            OutputPort<double>("block_target_theta"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class CalcKickInPose2 : public SyncActionNode
{
public:
    CalcKickInPose2(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("block_dist", 2.0, "distance from ball to target"),
            InputPort<double>("goal_x", -4.5, "goal x in field coordinates"),
            InputPort<double>("goal_y", 0.0, "goal y in field coordinates"),
            OutputPort<bool>("block_found"),
            OutputPort<double>("block_target_x"),
            OutputPort<double>("block_target_y"),
            OutputPort<double>("block_target_theta"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class CalcCornertKickPose : public SyncActionNode
{
public:
    CalcCornertKickPose(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
        OutputPort<bool>("block_found"),
        OutputPort<double>("block_target_x"),
        OutputPort<double>("block_target_y"),
        OutputPort<double>("block_target_theta")
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class SetpieceChase : public SyncActionNode
{
public:
    SetpieceChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.6, "追球的最大 x 速度"),
            InputPort<double>("vy_limit", 0.4, "追球的最大 y 速度"),
            InputPort<double>("vtheta_limit", 1.0, "追球时, 实时调整方向的速度不大于这个值"),
            InputPort<double>("dist", 0.1, "追球的目标是球后面多少距离"),
            InputPort<double>("safe_dist", 1.5, "circle back 时, 保持的安全距离"),
            InputPort<string>("chase_type", "", ""),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     
    double _dir = 1.0; 
};

class SetpieceAdjust : public SyncActionNode
{
public:
    SetpieceAdjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 3.25, "공의 각도가 이 값보다 크면 먼저 회전하여 공을 정면으로 바라보고 이동은 중단한다"),
            InputPort<double>("vx_limit", 0.1, "Adjust 과정에서 전진/후진 속도 vx의 최대 제한값이다"),
            InputPort<double>("vy_limit", 0.1, "Adjust 과정에서 좌우 이동 속도 vy의 최대 제한값이다"),
            InputPort<double>("vtheta_limit", 0.1, "Adjust 과정에서 회전 속도 vtheta의 최대 제한값이다"),
            InputPort<double>("range", 2.25, "공과 로봇 사이의 목표 거리로 이 값을 유지하려고 한다"),
            InputPort<double>("vtheta_factor", 3.0, "각도 보정 시 vtheta에 곱해지는 계수로 클수록 회전이 빠르다"),
            InputPort<double>("tangential_speed_far", 0.2, "공이 멀 때 각도 보정을 위해 사용하는 접선 방향 이동 속도이다"),
            InputPort<double>("tangential_speed_near", 0.15, "공이 가까울 때 각도 보정을 위해 사용하는 접선 방향 이동 속도이다"),
            InputPort<double>("near_threshold", 0.8, "목표와의 거리가 이 값보다 작으면 near speed를 사용한다"),
            InputPort<double>("no_turn_threshold", 0.1, "각도 오차가 이 값보다 작으면 회전을 수행하지 않는다"),
            InputPort<double>("turn_first_threshold", 0.5, "각도 오차가 이 값보다 크면 이동하지 않고 회전만 먼저 수행한다"),
            InputPort<string>("adjust_type", "", ""),
            // InputPort<double>("kick_y_offset", -0.077, "킥 시 공을 로봇 중심에서 y축으로 얼마나 오프셋 시킬지 결정"),
        };        
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class SetpieceDecide : public BT::SyncActionNode
{
public:
    SetpieceDecide(const std::string& name,
                   const BT::NodeConfig& config,
                   Brain* brain)
        : BT::SyncActionNode(name, config), brain_(brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("chase_threshold", 1.0, "if ballRange > threshold -> chase"),
            BT::InputPort<double>("yaw_threshold", 0.12, "abs(ballYaw) < yaw_threshold -> stop 가능"),
            BT::InputPort<double>("stop_range", 1.5, "ballRange < stop_range -> stop 가능"),
            BT::InputPort<std::string>("decision_in", "", "previous decision"),
            BT::OutputPort<std::string>("decision_out", "chase|adjust|stop"),
            BT::InputPort<std::string>("decide_type", "", ""),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain* brain_;
    rclcpp::Time timeLastTick_{0, 0, RCL_ROS_TIME};
    double lastDeltaDir_ = 0.0;
};