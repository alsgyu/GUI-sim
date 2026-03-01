#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterHoldNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class PredictBallTraj : public SyncActionNode
{
public:
    PredictBallTraj(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            // KF 튜닝에 사용
            InputPort<double>("R_meas", 0.01, "measurement noise (R)"), 
            InputPort<double>("sigma_a", 1.5, "proccess noise (Q)"),
            InputPort<double>("P0_pos", 0.25, "위치 공분산의 초기값"), 
            InputPort<double>("P0_vel", 1.0, "속도 공분산의 초기값"), 
            InputPort<double>("horizon", 0.5, "horizen초 뒤의 공을 예측"),
            // 미탐 시 감속에 사용
            InputPort<double>("drop_time", 1.0, "drop_time이 넘는 시간동안 공이 미탐 상태이면 감속 적용"),
            InputPort<double>("vel_decay", 0.5, "감속 정도"),
            // 시각화에 사용
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
            // 감속 모델에 사용
            InputPort<double>("a_min", 0.8, "고속에서 감속(작게)  (m/s^2)"),
            InputPort<double>("a_max", 0.8, "저속에서 감속(크게)  (m/s^2)"),
            InputPort<double>("k_av", 0.8, "v에 따른 전이 강도"),
        };
    }

    NodeStatus tick() override;

private:
    // ----- 시간 dt 계산용 -----
    bool has_prev_time_{false};
    rclcpp::Time prev_time_{0, 0, RCL_ROS_TIME};

    // ----- Kalman Filter 상태 -----
    bool kf_initialized_{false};
    double x_{0.0}, y_{0.0}, vx_{0.0}, vy_{0.0};
    double P_[4][4]{};  // 0으로 초기화

    // ----- 카메라 들어오는 프레임 판별용 -----
    bool has_last_meas_{false};
    rclcpp::Time last_meas_stamp_{0, 0, RCL_ROS_TIME};

    Brain *brain;
};

class CalcGoliePos : public SyncActionNode
{
public:
    CalcGoliePos(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("golie_radius", 1.0, "골대 중앙을 기준으로 하는 반경"),
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
        };
    }

    NodeStatus tick() override;

private:

    Brain *brain;
};

class GolieMove : public SyncActionNode
{
public:
    GolieMove(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_threshold", 0.05, "목표 위치 가까이 도달하면 정지"),
            InputPort<double>("vx_high", 1.0, "x 방향 최대 속도"),
            InputPort<double>("vx_low", -0.7, "x 방향 최소 속도"),
            InputPort<double>("vy_high", 1.0, "y 방향 최대 속도"),
            InputPort<double>("vy_low", -1.0, "y 방향 최소 속도"),
            InputPort<double>("ctPosx", -4.5, "골대중앙의 위치"),
            InputPort<double>("ctPosy", 0.0, "골대중앙의 위치"),
            InputPort<double>("Kp_theta", 4.0, "각속도의 P gain"),
            InputPort<double>("Kp", 3.0, "선속도의 P gain"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class GolieInitPos : public SyncActionNode
{
public:
    GolieInitPos(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 0.35, "여기까진 직진으로 성큼성큼 오다가, 여기부터 정면 바라보도록 회전"),
            InputPort<double>("stop_threshold", 0.1, "목표 위치 가까이 도달하면 정지"),
            InputPort<double>("vx_limit", 0.6, "최대 속도"),
            InputPort<double>("vy_limit", 0.3, "최대 속도"),
            InputPort<double>("init_golie_pos_x", -4.0, "골대중앙위치... 보다 살짝 앞에"),
            InputPort<double>("init_golie_pos_y", 0.0, "골대중앙위치"),
            InputPort<double>("init_golie_pos_theta", 0.0, "골대중앙위치"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};