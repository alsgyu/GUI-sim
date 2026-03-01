#include "pass_receive.h"
#include "brain.h"
#include "utils/math.h"

using namespace std;
using namespace BT;

NodeStatus PassReceive::onStart()
{
    return NodeStatus::RUNNING;
}

NodeStatus PassReceive::onRunning()
{
    // pass signal false로 초기화
    double targetX = 0.0;
    double targetY = 0.0;

    int myId = brain->config->playerId;
    // If I am 1 (Striker), partner is 2 (Index 1). 
    // If I am 2 (Defender), partner is 1 (Index 0).
    int partnerIdx = (myId == 1) ? 1 : 0;
    
    // Check isAlive to prevent using stale 'true' signal from disconnected partner
    if (brain->data->tmStatus[partnerIdx].isAlive && brain->data->tmStatus[partnerIdx].passSignal)
    {
        targetX = brain->data->tmStatus[partnerIdx].passTargetX;
        targetY = brain->data->tmStatus[partnerIdx].passTargetY;
    }

    brain->log->log("field/pass_target", rerun::Points2D({{static_cast<float>(targetX), static_cast<float>(-targetY)}}).with_colors({0x00FF00FF}).with_labels({"PassTarget"}).with_radii({0.1f}));

    Pose2D targetPoint;
    targetPoint.x = targetX;
    targetPoint.y = targetY;
    targetPoint.theta = 0.0;

    Pose2D target_r = brain->data->field2robot(targetPoint);
    double target_rx = target_r.x;
    double target_ry = target_r.y;

    brain->log->log("debug/pass_calcs", rerun::TextLog(format("dx: {:.2f}, theta: {:.2f}, tgt_rx: {:.2f}", target_r.x, target_r.theta, target_rx)));

    double distToTarget = hypot(target_rx, target_ry);
    
    // 이동 -> 추후 장애물 추가
    double p_gain = 1.0;
    double vx = target_rx * p_gain;
    double vy = target_ry * p_gain;

    // 방향 조절
    double vtheta = 0.0;
    if (brain->data->ballDetected) {
        vtheta = brain->data->ball.yawToRobot * 1.5; // 공이 보이면 공쪽을 보면서
    } else {
        double targetDir = atan2(target_ry, target_rx);
        vtheta = targetDir * 1.0; // 공이 안보이면 목표 지점을 보면서
    }

    // 속도
    double maxSpeed = 0.8;
    double speed = sqrt(vx*vx + vy*vy);
    if (speed > maxSpeed) {
        vx = vx / speed * maxSpeed;
        vy = vy / speed * maxSpeed;
    }

    // 타겟위치 도착
    if (distToTarget < 0.2) {
        vx = 0.01; vy = 0.01; // 정지보단 조금씩 움직이도록
        // 공도 왔는지 확인
        if (brain->data->ballDetected && brain->data->ball.range < 0.5) {
            return NodeStatus::SUCCESS;
        }
    }

    brain->client->setVelocity(vx, vy, vtheta);

    return NodeStatus::RUNNING;
}

void PassReceive::onHalted()
{
}

#define REGISTER_PASSRECEIVE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void RegisterPassReceiveNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_PASSRECEIVE_BUILDER(PassReceive)
}
