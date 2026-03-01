#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterGoalieDecisionNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

// From decision_role.cpp
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


// From clearing.cpp
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

// From hold.cpp
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