#include "brain.h"
#include "brain_tree.h"
#include "clearing.h"

#define REGISTER_CLEARING_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterClearingNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_CLEARING_BUILDER(CalcClearingDir)
    REGISTER_CLEARING_BUILDER(ClearingChase)
}

NodeStatus CalcClearingDir::tick() // kick.cpp의 CalcKickDir 참고
{
	auto bPos = brain->data->ball.posToField; // 공 위치
    double goalx, goaly;
    getInput("goalx", goalx);
    getInput("goaly", goaly);
    
    // goal -> ball 벡터 방향
    const double dx = bPos.x - goalx;
    const double dy = bPos.y - goaly;
    
    brain->data->clearingDir = atan2(dy, dx);

    brain->log->setTimeNow();
    brain->log->log(
        "field/clearing_dir",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->clearingDir), -10 * sin(brain->data->clearingDir)}})
            .with_origins({{bPos.x, -bPos.y}})
            .with_colors(0xFFFFFFFF)
            .with_radii(0.01)
            .with_draw_order(31)
    );

    return NodeStatus::SUCCESS;
}

NodeStatus ClearingChase::tick() // chase.cpp의 Chase 참고
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Chase4", rerun::TextLog(msg));
    };
    log("ticked");

    double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist); // 공으로부터 떨어질 목표 거리
    getInput("safeDist", safeDist); // 원형으로 돌아 들어갈 때의 안전 반경

	// 공 근처에서 속도 줄이기
    if (brain->data->ball.range < brain->config->nearBallRange) 
    {
        vxLimit = min(brain->config->nearBallSpeedLimit, vxLimit);
    }

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    double kickDir = brain->data->clearingDir;
    double theta_br = atan2(
        brain->data->robotPoseToField.y - brain->data->ball.posToField.y,
        brain->data->robotPoseToField.x - brain->data->ball.posToField.x);
    double theta_rb = brain->data->robotBallAngleToField;
    auto ballPos = brain->data->ball.posToField;

    double vx, vy, vtheta;
    Pose2D target_f, target_r; 
    string targetType = "direct"; 
    static double circleBackDir = 1.0; 
    double dirThreshold = M_PI / 2;
    double cbDirThreshold = 0.0; 
    if (targetType == "direct") dirThreshold *= 1.1;

    // calculate target point
    if (fabs(toPInPI(kickDir - theta_rb)) < dirThreshold) {
        targetType = "direct";
        log("targetType = direct");
        target_f.x = ballPos.x - dist * cos(kickDir);
        target_f.y = ballPos.y - dist * sin(kickDir);
    } 
    else {
        targetType = "circle_back";
        cbDirThreshold -= 0.2 * circleBackDir; 
        circleBackDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
        log(format("targetType = circle_back, circleBackDir = %.1f", circleBackDir));
        double tanTheta = theta_br + circleBackDir * acos(min(1.0, safeDist/max(ballRange, 1e-5))); 
        target_f.x = ballPos.x + safeDist * cos(tanTheta);
        target_f.y = ballPos.y + safeDist * sin(tanTheta);
    }
    target_r = brain->data->field2robot(target_f);
    brain->log->setTimeNow();
    brain->log->logBall("field/chase_target", Point({target_f.x, target_f.y, 0}), 0xFFFFFFFF, false, false);
           
    // P 제어
    double p_gain;
    getInput("p_gain", p_gain);
    vx = target_r.x * p_gain;
    vy = target_r.y * p_gain;
    vtheta = ballYaw;   
    
    double speed = sqrt(vx*vx + vy*vy);
    if (speed > vxLimit) {
        vx = vx / speed * vxLimit;
        vy = vy / speed * vxLimit;}

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}