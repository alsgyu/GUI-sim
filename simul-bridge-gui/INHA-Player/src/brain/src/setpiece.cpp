#include "brain.h"
#include "gotopose.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

#define REGISTER_SETPIECE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterSetpieceNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_SETPIECE_BUILDER(CalcKickInPose1) // block pass dir
    REGISTER_SETPIECE_BUILDER(CalcKickInPose2) // block kick dir
    REGISTER_SETPIECE_BUILDER(CalcCornertKickPose) // block kick dir

    REGISTER_SETPIECE_BUILDER(SetpieceChase) 
    REGISTER_SETPIECE_BUILDER(SetpieceAdjust) 
    REGISTER_SETPIECE_BUILDER(SetpieceDecide) 
}

NodeStatus CalcKickInPose1::tick(){
    double block_dist = 2.0;
    double max_age_sec = 5.0;
    getInput("block_dist", block_dist);
    getInput("max_age_sec", max_age_sec);

    auto bPos = brain->data->emaball.posToField;
    double bx = bPos.x, by = bPos.y;

    auto robots = brain->data->getRobots();

    int nearestIdx = -1;
    double bestD = 1e9;

    rclcpp::Time now = brain->get_clock()->now();

    // 가장 가까운(최근 관측된) opponent 찾기
    for (int i = 0; i < (int)robots.size(); ++i) {
        const auto& r = robots[i];
        if (r.label != "Opponent") continue;

        // 오래된 관측 제외
        double elapsed = (now - r.timePoint).seconds();
        if (elapsed > max_age_sec) continue;

        double d = norm(r.posToField.x - bx, r.posToField.y - by);
        if (d < bestD) {
            bestD = d;
            nearestIdx = i;
        }
    }

    bool found = (nearestIdx >= 0);
    setOutput("block_found", found);

    double tx = bx, ty = by;

    if (found) {
        const auto& opp = robots[nearestIdx];

        // direction = (Ball - Opp)  => Opp -> Ball 방향으로 block_dist 만큼 나간 점
        double dx = bx - opp.posToField.x;
        double dy = by - opp.posToField.y;
        double L = std::hypot(dx, dy);

        if (L > 1e-6) {
            double step = std::max(0.0, block_dist);
            tx = bx + step * (dx / L);
            ty = by + step * (dy / L);
        } else {
            found = false;
            setOutput("block_found", false);
            // tx, ty는 기본값(bx, by) 그대로
        }
    }

    // 목표 위치에서 공을 바라보는 방향
    double theta = atan2(by - ty, bx - tx);

    setOutput("block_target_x", tx);
    setOutput("block_target_y", ty);
    setOutput("block_target_theta", theta);


    return BT::NodeStatus::SUCCESS;
}

NodeStatus CalcKickInPose2::tick(){ // 일단은 공-골대 막는 위치로 선정. 더 나은 위치 선정이 있을지 고민 필요
    double block_dist = 2.0;
    double goal_x = -4.5;
    double goal_y = 0.0;

    getInput("block_dist", block_dist);
    getInput("goal_x", goal_x);
    getInput("goal_y", goal_y);

    auto bPos = brain->data->emaball.posToField;
    double bx = bPos.x, by = bPos.y;

    double dx = goal_x - bx;
    double dy = goal_y - by;
    double L = std::hypot(dx, dy);

    bool found = (L > 1e-6);
    setOutput("block_found", found);

    double tx = bx, ty = by;

    if (found) {
        double step = std::max(0.0, block_dist);
        tx = bx + step * (dx / L);
        ty = by + step * (dy / L);
    }

    double theta = atan2(by - ty, bx - tx);

    setOutput("block_target_x", tx);
    setOutput("block_target_y", ty);
    setOutput("block_target_theta", theta);

    // // 로그/시각화
    // brain->log->setTimeNow();

    // brain->log->log(
    //     "field/block_ball_goal_dir",
    //     rerun::Arrows2D::from_vectors({{(float)(tx - bx), (float)(-(ty - by))}})
    //         .with_origins({{(float)bx, (float)(-by)}})
    //         .with_colors({0x00FF00FF})
    //         .with_radii(0.015f)
    //         .with_draw_order(32)
    // );

    // brain->log->logToScreen(
    //     "tree/CalcBlockTargetBallGoal",
    //     format("Ball-Target-Goal found=%d target=(%.2f,%.2f) ball=(%.2f,%.2f) goal=(%.2f,%.2f) dist=%.2f",
    //            (int)found, tx, ty, bx, by, goal_x, goal_y, block_dist),
    //     0x00FFFFFF
    // );

    return BT::NodeStatus::SUCCESS;
}

NodeStatus CalcCornertKickPose::tick(){ 
    double target_x = -4.0;
    double target_y_pos = 1.0;
    double target_y_neg = -1.0;

    auto bPos = brain->data->emaball.posToField;
    double bx = bPos.x, by = bPos.y;

    // 공 y 부호에 따라 타겟 선택
    double tx = target_x;
    double ty = (by >= 0.0) ? target_y_pos : target_y_neg;
    double theta = atan2(by - ty, bx - tx);

    setOutput("block_found", true);
    setOutput("block_target_x", tx);
    setOutput("block_target_y", ty);
    setOutput("block_target_theta", theta);

    return BT::NodeStatus::SUCCESS;
}

NodeStatus SetpieceChase::tick(){
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Chase4", rerun::TextLog(msg));
    };
    log("ticked");

    if (brain->tree->getEntry<string>("striker_state") != "chase") return NodeStatus::SUCCESS;
    
    double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
    string chasetype;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);
    getInput("safe_dist", safeDist);
    getInput("chase_type", chasetype);

    // 전술에서 설정한 속도 제한이 있으면 보정, Param값이 있으면 그것을 우선시하도록 (전술적 의도 반영)
    if (auto val = brain->tree->getEntry<double>("Strategy.param_chase_speed_limit")) {
        vxLimit = val; // Overwrite
        // vyLimit은 보통 vxLimit보다 작거나 같으므로 비율조정 or cap
        if (vyLimit > vxLimit) vyLimit = vxLimit; 
    }

    bool avoidObstacle = true;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    double oaSafeDist = 1.5;
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

    if (
        brain->config->limitNearBallSpeed
        && brain->data->ball.range < brain->config->nearBallRange
    ) {
        vxLimit = min(brain->config->nearBallSpeedLimit, vxLimit);
    }

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    double Dir;

    if (chasetype == "Pass") {
        Dir = brain->data->passDir;
    }
    else {
        Dir = brain->data->kickDir;
    }
    
    // robot(opponent)도 추가
    // auto robots = brain->data->getRobots();
    // obstacles.insert(obstacles.end(), robots.begin(), robots.end());
    

    double theta_br = atan2(
        brain->data->robotPoseToField.y - brain->data->ball.posToField.y,
        brain->data->robotPoseToField.x - brain->data->ball.posToField.x
    );
    double theta_rb = brain->data->robotBallAngleToField;
    auto ballPos = brain->data->ball.posToField;


    double vx, vy, vtheta;
    Pose2D target_f, target_r; 
    static string targetType = "direct"; 
    static double circleBackDir = 1.0; 
    double dirThreshold = M_PI / 2;

    // one_touch 상황이라면 몸을 골대쪽으로 할 수 있게
    double distToGoal = norm(
        brain->data->robotPoseToField.x - (brain->config->fieldDimensions.length / 2),
        brain->data->robotPoseToField.y
    );
    if (distToGoal < 2.0) {
        dirThreshold = M_PI / 6; // 30 degrees
    }

    if (targetType == "direct") dirThreshold *= 1.2;


    // calculate target point
    if (fabs(toPInPI(Dir - theta_rb)) < dirThreshold) {
        log("targetType = direct");
        targetType = "direct";
        
        target_f.x = ballPos.x - dist * cos(Dir);
        target_f.y = ballPos.y - dist * sin(Dir);
    } 
    else {
        // targetType = "circle_back";
        double cbDirThreshold = 0.0; 
        cbDirThreshold -= 0.2 * circleBackDir; 
        circleBackDir = toPInPI(theta_br - Dir) > cbDirThreshold ? 1.0 : -1.0;
        log(format("targetType = circle_back, circleBackDir = %.1f", circleBackDir));
        double tanTheta = theta_br + circleBackDir * acos(min(1.0, safeDist/max(ballRange, 1e-5))); 
        target_f.x = ballPos.x + safeDist * cos(tanTheta);
        target_f.y = ballPos.y + safeDist * sin(tanTheta);
    }
    target_r = brain->data->field2robot(target_f);
    brain->log->setTimeNow();
    brain->log->logBall("field/chase_target", Point({target_f.x, target_f.y, 0}), 0xFFFFFFFF, false, false);
            
    double targetDir = atan2(target_r.y, target_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);
    
    if (avoidObstacle && distToObstacle < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 0.5;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = ballYaw;
        log(format("avoidDir = %.2f", avoidDir));
    } 

    else {
        
        double p_gain = 1.0;
        vx = target_r.x * p_gain;
        vy = target_r.y * p_gain;

        vtheta = ballYaw;   
        
        double speed = sqrt(vx*vx + vy*vy);
        if (speed > vxLimit) {
            vx = vx / speed * vxLimit;
            vy = vy / speed * vxLimit; 
        }
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    static double smoothVx = 0.0;
    static double smoothVy = 0.0;
    static double smoothVtheta = 0.0;
    smoothVx = smoothVx * 0.7 + vx * 0.3;
    smoothVy = smoothVy * 0.7 + vy * 0.3;
    smoothVtheta = smoothVtheta * 0.7 + vtheta * 0.3;

    // chase 멈춤 조건
    // bool chaseDone = brain->data->ball.range < dist * 1.2 && fabs(toPInPI(Dir - theta_rb)) < M_PI / 3;
    // if (chaseDone){
    //     brain->tree->setEntry("striker_state", "adjust");
    //     log("chase -> adjust");
    // }
    log(format("distToObstacle = %.2f, targetDir = %.2f", distToObstacle, targetDir));
    
    // brain->client->setVelocity(smoothVx, smoothVy, smoothVtheta, false, false, false);
    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus SetpieceAdjust::tick(){
    auto log = [=](string msg) { 
        brain->log->setTimeNow();
        brain->log->log("debug/adjust5", rerun::TextLog(msg)); 
    };
    log("enter");
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }
    // 승재욱 추가
    // if (brain->tree->getEntry<string>("striker_state") != "adjust") return NodeStatus::SUCCESS;

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, range, st_far, st_near, vtheta_factor, NEAR_THRESHOLD;
    std::string adjusttype;
    getInput("near_threshold", NEAR_THRESHOLD);
    getInput("tangential_speed_far", st_far);
    getInput("tangential_speed_near", st_near);
    getInput("vtheta_factor", vtheta_factor);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("range", range);
    double kickYOffset;
    // if(!getInput("kick_y_offset", kickYOffset)) kickYOffset = 0.077;
    kickYOffset = 0.0; // 당분간 사용 안 함

    log(format("ballX: %.1f ballY: %.1f ballYaw: %.1f", brain->data->ball.posToRobot.x, brain->data->ball.posToRobot.y, brain->data->ball.yawToRobot));
    double NO_TURN_THRESHOLD, TURN_FIRST_THRESHOLD;
    getInput("no_turn_threshold", NO_TURN_THRESHOLD);
    getInput("turn_first_threshold", TURN_FIRST_THRESHOLD);


    double vx = 0, vy = 0, vtheta = 0;

    getInput("adjust_type", adjusttype);
    double Dir;
    if (adjusttype == "Pass") {
        Dir = brain->data->passDir;
    }
    else {
        Dir = brain->data->kickDir;
    }

    double dir_rb_f = brain->data->robotBallAngleToField; 
    // double deltaDir = toPInPI(kickDir - dir_rb_f);
    double deltaDirVal = toPInPI(Dir - dir_rb_f);
    double ballRange = brain->data->ball.range;

    // 한 발로 차기 위해 공을 로봇 중심보다 옆(kickYOffset)에 두도록 정렬
    // deltaDir 각도 에러 수정
    double deltaDir = toPInPI(Dir - dir_rb_f + kickYOffset);

    double ballYaw = brain->data->ball.yawToRobot;
    // double st = cap(fabs(deltaDir), st_far, st_near);
    double st = st_far; 
    double R = ballRange; 
    double r = range;
    double sr = cap(R - r, 0.5, -0.2); // 0.2는 너무 가까워질 때 후진도 가능하도록 -> 게걸음 방지
    // R: 현재 공 거리, r: 목표 거리 (0.6), sr: 앞으로 가는 속도 (R-r)
    log(format("R: %.2f, r: %.2f, sr: %.2f, offset: %.2f", R, r, sr, kickYOffset));

    log(format("deltaDir = %.1f", deltaDir));
    if (fabs(deltaDir) * R < NEAR_THRESHOLD) {
        log("use near speed");
        st = st_near;
        // sr = 0.;
        // vxLimit = 0.1;
    }

    // 골문 가까이서 더 빠르게
    double distToGoal = norm(
        brain->data->robotPoseToField.x - (brain->config->fieldDimensions.length / 2),
        brain->data->robotPoseToField.y
    );
    
    // 골대와 가까우면(3m) 더 과감하게 움직임 (= 속도 증가)
    if (distToGoal < 3.0) {
        log("Near Goal Mode: Boost Speed");
        st *= 1.5; // 횡이동 속도 1.5배
        sr = cap(sr, 0.5, -0.5); // 전후진 속도 제한도 품
    }

    double theta_robot_f = brain->data->robotPoseToField.theta; 
    double thetat_r = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - theta_robot_f; 
    double thetar_r = dir_rb_f - theta_robot_f; 

    vx = st * cos(thetat_r) + sr * cos(thetar_r); 
    vy = st * sin(thetat_r) + sr * sin(thetar_r); 
    // vtheta = toPInPI(ballYaw + st / R * (deltaDir > 0 ? 1.0 : -1.0)); 
    vtheta = ballYaw;
    vtheta *= vtheta_factor; 
    if (fabs(ballYaw) < NO_TURN_THRESHOLD) vtheta = 0.;
    
    // 방향이 많이 틀어졌거나 위치가 많이 벗어났으면 일단 제자리 회전
    if (
        fabs(ballYaw) > TURN_FIRST_THRESHOLD 
        && fabs(deltaDir) < M_PI / 4
    ) { 
        vx = 0;
        vy = 0;
    }

    vx = cap(vx, vxLimit, -0.);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
    
    log(format("vx: %.1f vy: %.1f vtheta: %.1f", vx, vy, vtheta));
    brain->client->setVelocity(vx, vy, vtheta);

    // 승재욱 추가
    double successDeltaDir = 0.1; // 기본 5.7도
    if (distToGoal < 3.0) successDeltaDir = 0.25; // 14도 정도로 완화

    bool adjustDone = fabs(deltaDir) <= successDeltaDir && fabs(ballYaw) <= 0.1 && ballRange < range + 0.1;
    if (adjustDone){
        // brain->tree->setEntry("striker_state", "kick");
        log("adjust -> kick (ready)");
    }
    log(format("deltaDir = %.1f", deltaDir));

    return NodeStatus::SUCCESS;
}

NodeStatus SetpieceDecide::tick()
{
    double chaseRangeThreshold = 1.2;
    double yawThreshold = 0.12;
    double stopRange = 1.5;
    std::string lastDecision;
    std::string decidetype;

    getInput("chase_threshold", chaseRangeThreshold);
    getInput("yaw_threshold", yawThreshold);
    getInput("stop_range", stopRange);
    getInput("decision_in", lastDecision);
    getInput("decide_type", decidetype);

    double Dir;
    if (decidetype == "Pass") {
        Dir = brain_->data->passDir;
    }
    else {
        Dir = brain_->data->kickDir;
    }

    const double dir_rb_f  = brain_->data->robotBallAngleToField;
    const auto&  ball      = brain_->data->ball;

    const double ballRange = ball.range;
    const double ballYaw   = ball.yawToRobot;

    double deltaDir = toPInPI(Dir - dir_rb_f);
    auto now = brain_->get_clock()->now();
    auto dt  = brain_->msecsSince(timeLastTick_);
    bool reachedDir =
        (deltaDir * lastDeltaDir_ <= 0) &&
        (std::fabs(deltaDir) < 0.1) &&
        (dt < 100);
    reachedDir = reachedDir || (std::fabs(deltaDir) < 0.1);
    timeLastTick_ = now;
    lastDeltaDir_ = deltaDir;

    std::string newDecision;
    uint32_t color = 0xFFFFFFFF;

    // 1) 멀면 chase (히스테리시스 살짝)
    bool wasChasing = (lastDecision == "chase");
    if (ballRange > chaseRangeThreshold * (wasChasing ? 0.9 : 1.0)) {
        newDecision = "chase";
        color = 0x0000FFFF;
    }
    // 2) stop 조건 
    else if (
        reachedDir &&
        brain_->data->ballDetected &&
        std::fabs(ballYaw) < yawThreshold &&
        ballRange < stopRange
    ) {
        newDecision = "stop";
        color = 0x00FF00FF;
    }
    // 3) 그 외 adjust
    else {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    setOutput("decision_out", newDecision);

    brain_->log->logToScreen(
        "tree/SetpieceDecide",
        format("Decision:%s ballR:%.2f ballYaw:%.2f dDir:%.2f reached:%d",
               newDecision.c_str(), ballRange, ballYaw, deltaDir, (int)reachedDir),
        color
    );

    return NodeStatus::SUCCESS;
}