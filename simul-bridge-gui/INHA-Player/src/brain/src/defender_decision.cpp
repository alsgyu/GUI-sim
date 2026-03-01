#include "brain.h"
#include "brain_tree.h"
#include "defender_decision.h"
#include "utils/math.h" // For pointMinDistToLine, etc

#include <cstdlib>
#include <ctime>
#include <cmath>

#define REGISTER_DEFENDER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfiguration &config) { return make_unique<Name>(name, config, brain); });


void RegisterDefenderDecisionNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_DEFENDER_BUILDER(DefenderDecision)
    REGISTER_DEFENDER_BUILDER(LeadDefenderDecision)
    REGISTER_DEFENDER_BUILDER(SubDefenderDecision)
    REGISTER_DEFENDER_BUILDER(DefenderShootBlock) // TODO: 좌표 계산하고 제어(path planning module) 호출
    REGISTER_DEFENDER_BUILDER(DefenderPassBlock) // TODO: 패스각 차단하는 위치 계산하고 제어 호출
    REGISTER_DEFENDER_BUILDER(DefenderClearingDecide)
    REGISTER_DEFENDER_BUILDER(CalcDefenderClearingDir)

}

// ==========================================
// DefenderDecision (from DefenderDecide in decision_role.cpp)
// ==========================================
NodeStatus DefenderDecision::tick() {
    double paramChaseSpeed = 0.8;
    double paramDefenseLineX = -10.0; // 기본값: 제한 없음
    double paramKickThreshold = 0.3;  
    
    // Default 값
    if (auto val = brain->tree->getEntry<double>("Strategy.param_chase_speed_limit")) paramChaseSpeed = val;
    if (auto val = brain->tree->getEntry<double>("Strategy.param_defense_line_x")) paramDefenseLineX = val;
    if (auto val = brain->tree->getEntry<double>("Strategy.param_kick_threshold")) paramKickThreshold = val;
    
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision;
    getInput("decision_in", lastDecision);
    double clearingThreshold;
    getInput("clearing_threshold", clearingThreshold);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    const double goalpostMargin = 0.5; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");
    
    // 장애물 회피 로직
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < -2.0 
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;


    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < 0.1
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    bool passFound = brain->tree->getEntry<bool>("pass_found");
    bool isLead = brain->tree->getEntry<bool>("is_lead");

    auto pose = brain->data->robotPoseToField;

    // ===clearing condition===
    bool shouldClearing = false;
    auto Opponents = brain->data->getRobots(); 
    double OpponentBallDist = 999999.0;
    for (const auto& opponent : Opponents){
        if (opponent.label != "Opponent") continue; // 상대팀이 아니면 스킵
        double dist = norm(opponent.posToField.x - ball.posToField.x, opponent.posToField.y - ball.posToField.y);
        if(dist<OpponentBallDist) OpponentBallDist = dist;
    } // calculate nearest ball-opponent distance
    shouldClearing = (OpponentBallDist < clearingThreshold) ? true : false;
    // =========================

    // 1) 공을 모르면 -> find
    if (!(iKnowBallPos || tmBallPosReliable)) {
        newDecision = "find";
        color = 0xFFFFFFFF;
    }
    // 2) non-lead인데 레인 밖이면 -> return (레인 복귀)
    else if (!isLead) {
        newDecision = "return";
        color = 0xFFFF00FF;
        
        if (lastDecision != "return") {
        brain->tree->setEntry("return_x", pose.x);

        // 화면 로그로 확인 (콘솔/화면)
        brain->log->logToScreen(
            "debug/ReturnTarget",
            format("Saved return_x=%.2f (pose=(%.2f,%.2f,%.2f)) target=(%.2f,-2.5)",
                   pose.x, pose.x, pose.y, pose.theta, pose.x),
            0xFFFF00FF
        );
        }
    }
    // 3) lead인데 opponent가 너무 가까우면 clearing
    else if (shouldClearing) {
        newDecision = "clearing";
        color = 0xFFFF00FF;
    }
    // 4) clearing 할 상황은 아닌데 lead이면 -> (기존대로) chase / pass / adjust
    else if (isLead) {
        // 멀면 chase
        bool wasChasing = (lastDecision == "chase");
        if (ballRange > chaseRangeThreshold * (wasChasing ? 0.9 : 1.0)) {
            newDecision = "chase";
            color = 0x0000FFFF;
        }
        // 킥(패스) 조건
        else if (
            (reachedKickDir) &&
            brain->data->ballDetected &&
            std::fabs(brain->data->ball.yawToRobot) < 0.1 &&
            !avoidKick &&
            ball.range < 1.5
        ) {
            if (passFound) newDecision = "pass";
            else newDecision = "kick";
            color = 0x00FF00FF;
            brain->data->isFreekickKickingOff = false;
        }
        // 그 외 adjust
        else {
            newDecision = "adjust";
            color = 0xFFFF00FF;
        }
    }
    // 5) non-lead면서 레인 안이면 -> side_chase (항상)
    // TODO: 수비 포지셔닝 추가
    else {
        newDecision = "side_chase";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, brain->data->tmImLead
        ),
        color
    );
    return NodeStatus::SUCCESS;
}

NodeStatus LeadDefenderDecision::tick() {
    auto ball = brain->data->emaball;
    auto pose = brain->data->robotPoseToField;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");

    double clearingThreshold;
    getInput("clearing_threshold", clearingThreshold);
    
    // ===clearing condition===
    bool shouldClearing = false;
    auto Opponents = brain->data->getRobots(); 
    double OpponentBallDist = 999999.0;
    for (const auto& opponent : Opponents){
        if (opponent.label != "Opponent") continue; // 상대팀이 아니면 스킵
        double dist = norm(opponent.posToField.x - ball.posToField.x, opponent.posToField.y - ball.posToField.y);
        if(dist<OpponentBallDist) OpponentBallDist = dist;
    } // calculate nearest ball-opponent distance
    shouldClearing = (OpponentBallDist < clearingThreshold) ? true : false;
    // =========================

    if (!(iKnowBallPos || tmBallPosReliable)) {
        newDecision = "find";
        color = 0xFFFFFFFF;
    }
    else if (shouldClearing){
        newDecision = "clearing";
        color = 0xFFFF00FF;
    }
    else{ // 슛각 차단
        newDecision = "shoot_block";
        color = 0x0000FFFF;
    }


    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "LeadDefenderDecision: %s ballrange: %.2f ballyaw: %.2f", 
            newDecision.c_str(), ballRange, ballYaw
        ),
        color
    );
    return NodeStatus::SUCCESS;
}

NodeStatus SubDefenderDecision::tick() {
    auto ball = brain->data->emaball;
    auto pose = brain->data->robotPoseToField;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");

    if (!(iKnowBallPos || tmBallPosReliable)) {
        newDecision = "find";
        color = 0xFFFFFFFF;
    }
    else{ // 패스각 차단
        newDecision = "pass_block";
        color = 0x0000FFFF;
    }


    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "SubDefenderDecision: %s ballrange: %.2f ballyaw: %.2f", 
            newDecision.c_str(), ballRange, ballYaw
        ),
        color
    );
    return NodeStatus::SUCCESS;
}

NodeStatus DefenderShootBlock::tick() {
    double ctPosx, ctPosy, blockDistFromBall;
    getInput("ctPosx", ctPosx);
    getInput("ctPosy", ctPosy);
    getInput("block_dist_from_ball", blockDistFromBall);

    auto bPos = brain->data->emaball.posToField;
    auto pose = brain->data->robotPoseToField;
    double bx = bPos.x, by = bPos.y;
    double gx = ctPosx, gy = ctPosy;

    // 공 -> 골대중앙 방향 벡터
    double dx = gx - bx;
    double dy = gy - by;
    double d = norm(dx, dy);

    // 기본값: 공 위치
    double tx = bx, ty = by;

    if (d > 1e-6) {
        double step = blockDistFromBall;
        if (step < 0.0) step = 0.0;
        if (step > d) step = d;  // 선분 밖으로 나가지 않게 clamp

        double ux = dx / d;
        double uy = dy / d;

        tx = bx + step * ux;
        ty = by + step * uy;
    }

    // TODO: tx, ty를 타겟으로 해서 제어하기
    // GolieMove 스타일 제어 (포트 없이 하드코딩)
    const double stopThreshold = 0.10;
    const double Kp = 2.0;
    const double KpTheta = 4.0;
    const double vxHigh = 1.0;
    const double vxLow  = -1.0;
    const double vyHigh = 0.5;
    const double vyLow  = -0.5;
    const double vthetaHigh = 1.0;
    const double vthetaLow  = -1.0;

    double rx = pose.x;
    double ry = pose.y;
    double rtheta = pose.theta;

    // field 기준 오차
    double ex = tx - rx;
    double ey = ty - ry;
    double dist = norm(ex, ey);
    double theta = atan2(ey, ex);

    // 타겟에서는 공을 바라보도록 회전 목표 설정
    double targetTheta = atan2(by - ry, bx - rx);

    double vtheta = toPInPI((theta - rtheta) + (targetTheta - theta));
    vtheta *= KpTheta;

    // field -> robot 좌표계 제어량 변환
    double controlx = ex * cos(rtheta) + ey * sin(rtheta);
    double controly = -ex * sin(rtheta) + ey * cos(rtheta);

    // 가까워질수록 감속
    double linearFactor = Kp / (1.0 + exp(-3.0 * (dist - 0.3)));
    controlx *= linearFactor;
    controly *= linearFactor;

    // 제한
    controlx = cap(controlx, vxHigh, vxLow);
    controly = cap(controly, vyHigh, vyLow);
    vtheta = cap(vtheta, vthetaHigh, vthetaLow);

    // 도착 시 정지
    if (dist < stopThreshold) {
        controlx = 0.0;
        controly = 0.0;
        vtheta = 0.0;
    }

    brain->client->setVelocity(controlx, controly, vtheta, false, false, false);

    // 로그
    brain->log->setTimeNow();
    brain->log->log(
        "field/defender_shoot_block_dir",
        rerun::Arrows2D::from_vectors({{tx - bx, -(ty - by)}})
            .with_origins({{bx, -by}})
            .with_colors({0x00FFFFFF})
            .with_radii(0.015)
            .with_draw_order(32)
    );
    brain->log->log(
        "field/defender_shoot_block_target",
        rerun::Points2D({{(float)tx, -(float)ty}})
            .with_colors({0x00FFFFFF})
            .with_radii(0.05f)
    );
    brain->log->logToScreen(
        "tree/DefenderShootBlock",
        format("ShootBlock target=(%.2f, %.2f) ball=(%.2f, %.2f) goal=(%.2f, %.2f) n=%.2f",
               tx, ty, bx, by, gx, gy, blockDistFromBall),
        0x00FFFFFF
    );

    return NodeStatus::SUCCESS;
}

// defender_decision.cpp 에 구현 추가
NodeStatus DefenderPassBlock::tick() {
    auto ballPos = brain->data->emaball.posToField;
    auto pose = brain->data->robotPoseToField;
    auto Opponents = brain->data->getRobots();

    double bx = ballPos.x;
    double by = ballPos.y;

    int nearestIdx = -1;
    int secondIdx = -1;
    double nearestDist = 1e9;
    double secondDist = 1e9;
    int validOpponentCount = 0;

    for (int i = 0; i < (int)Opponents.size(); ++i) {
        const auto& opponent = Opponents[i];
        if (opponent.label != "Opponent") continue;

        validOpponentCount++;
        double dist = norm(opponent.posToField.x - bx, opponent.posToField.y - by);

        if (dist < nearestDist) {
            secondDist = nearestDist;
            secondIdx = nearestIdx;
            nearestDist = dist;
            nearestIdx = i;
        } else if (dist < secondDist) {
            secondDist = dist;
            secondIdx = i;
        }
    }

    int targetOppIdx = secondIdx;
    const char* usedRank = "second";
    if (targetOppIdx < 0) {
        targetOppIdx = nearestIdx;
        usedRank = "nearest";
    }

    double tx = bx;
    double ty = by;

    if (targetOppIdx >= 0) {
        const auto& targetOpp = Opponents[targetOppIdx];
        double ox = targetOpp.posToField.x;
        double oy = targetOpp.posToField.y;

        // 공-상대 선분의 중점
        tx = 0.5 * (bx + ox);
        ty = 0.5 * (by + oy);

        brain->log->setTimeNow();
        brain->log->log(
            "field/defender_pass_block_dir",
            rerun::Arrows2D::from_vectors({{tx - bx, -(ty - by)}})
                .with_origins({{bx, -by}})
                .with_colors({0x00FFFFFF})
                .with_radii(0.015)
                .with_draw_order(32)
        );
    }

    // TODO: tx, ty를 타겟으로 해서 제어하기
    // GolieMove 스타일 제어 (포트 없이 하드코딩)
    const double stopThreshold = 0.10;
    const double Kp = 2.0;
    const double KpTheta = 4.0;
    const double vxHigh = 1.0;
    const double vxLow  = -1.0;
    const double vyHigh = 0.5;
    const double vyLow  = -0.5;
    const double vthetaHigh = 1.0;
    const double vthetaLow  = -1.0;

    double rx = pose.x;
    double ry = pose.y;
    double rtheta = pose.theta;

    // field 기준 오차
    double ex = tx - rx;
    double ey = ty - ry;
    double dist = norm(ex, ey);
    double theta = atan2(ey, ex);

    // 타겟에서는 공을 바라보도록 회전 목표 설정
    double targetTheta = atan2(by - ry, bx - rx);

    double vtheta = toPInPI((theta - rtheta) + (targetTheta - theta));
    vtheta *= KpTheta;

    // field -> robot 좌표계 제어량 변환
    double controlx = ex * cos(rtheta) + ey * sin(rtheta);
    double controly = -ex * sin(rtheta) + ey * cos(rtheta);

    // 가까워질수록 감속
    double linearFactor = Kp / (1.0 + exp(-3.0 * (dist - 0.3)));
    controlx *= linearFactor;
    controly *= linearFactor;

    // 제한
    controlx = cap(controlx, vxHigh, vxLow);
    controly = cap(controly, vyHigh, vyLow);
    vtheta = cap(vtheta, vthetaHigh, vthetaLow);

    // 도착 시 정지
    if (dist < stopThreshold) {
        controlx = 0.0;
        controly = 0.0;
        vtheta = 0.0;
    }

    brain->client->setVelocity(controlx, controly, vtheta, false, false, false);

    brain->log->setTimeNow();
    brain->log->log(
        "field/defender_pass_block_target",
        rerun::Points2D({{(float)tx, -(float)ty}})
            .with_colors({0x00FFFFFF})
            .with_radii(0.05f)
    );
    brain->log->logToScreen(
        "tree/DefenderPassBlock",
        format("PassBlock target=(%.2f, %.2f) oppCount=%d use=%s", tx, ty, validOpponentCount, usedRank),
        0x00FFFFFF
    );

    return NodeStatus::SUCCESS;
}



// ==========================================
// DefenderClearingDecide
// ==========================================
NodeStatus DefenderClearingDecide::tick() {
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision;
    getInput("clearing_decision_in", lastDecision);

    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField;

    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir =
        deltaDir * lastDeltaDir <= 0 &&
        fabs(deltaDir) < 0.1 &&
        dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");

    string newDecision;
    auto color = 0xFFFFFFFF;

    if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0)) {
        newDecision = "chase";
        color = 0x0000FFFF;
    } else if (reachedKickDir &&
               brain->data->ballDetected &&
               fabs(ballYaw) < 0.1 &&
               ball.range < 1.5) {
        newDecision = "kick";
        color = 0x00FF00FF;
    } else {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    setOutput("clearing_decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Clearing",
        format("ClearingDecision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f",
               newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f),
        color
    );
    return NodeStatus::SUCCESS;
}

// ==========================================
// CalcDefenderClearingDir (Renamed from CalcClearingDir in kick.cpp of Defender repo)
// ==========================================
NodeStatus CalcDefenderClearingDir::tick() {
    double OffsetDegree;
    getInput("offset_degree", OffsetDegree);
    
    // auto ballPos = brain->data->ball.posToField;
    // Note: Defender repo used emaball. Assuming Striker repo has emaball or I should use ball.
    // Let's use ball for safety if emaball is not guaranteed in Striker brain data struct, 
    // BUT if Unified means merging all, I assume brain_data.h is consistent.
    // Accessing emaball from brain->data.
    auto ballPos = brain->data->emaball.posToField; 
    auto opponents = brain->data->getRobots();

    int nearestIdx = -1;
    double minDist = 1e9;

    for (size_t i = 0; i < opponents.size(); ++i) {
        const auto& opponent = opponents[i];
        if (opponent.label != "Opponent") continue;
        double dist = norm(opponent.posToField.x - ballPos.x,
                           opponent.posToField.y - ballPos.y);
        if (dist < minDist) {
            minDist = dist;
            nearestIdx = static_cast<int>(i);
        }
    }

    double clearingDir = 0.0; // 기본은 +x 방향
    if (nearestIdx >= 0) {
        const auto& nearestOpponent = opponents[nearestIdx];
        double angleToOpponent = atan2(
            nearestOpponent.posToField.y - ballPos.y,
            nearestOpponent.posToField.x - ballPos.x
        );
        const double offset = deg2rad(OffsetDegree);
        // 상대 각도에서 +x 방향(0rad) 쪽으로 30도 꺾기
        clearingDir = angleToOpponent + (angleToOpponent > 0.0 ? -offset : offset);
        clearingDir = toPInPI(clearingDir);
    }

    brain->data->kickDir = clearingDir;

    brain->log->setTimeNow();
    brain->log->log(
        "field/clearing_dir",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{ballPos.x, -ballPos.y}})
            .with_colors({0x00FF00FF})
            .with_radii(0.01)
            .with_draw_order(31)
    );

    return NodeStatus::SUCCESS;
}