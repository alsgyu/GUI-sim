#include "brain.h"
#include "offtheball.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

// BehaviorTree Factory에 Test 노드를 생성하는 함수를 등록하는 역할 -> 코드 양 줄일 수 있음
#define REGISTER_OFFTHEBALL_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterOfftheballNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_OFFTHEBALL_BUILDER(OffTheBall) // Gotopose
    REGISTER_OFFTHEBALL_BUILDER(InitPos)
}

NodeStatus OffTheBall::tick(){
    double stop_Threshold;
    double vLimit;
    double distFromGoal;
    getInput("stop_threshold", stop_Threshold);
    getInput("v_limit", vLimit);
    getInput("dist_from_goal", distFromGoal);

    // =========================
    // Offtheball "위치 선정" 로직 이식 (targetx, targety, targettheta 계산)
    // =========================
    double targetx, targety, targettheta;
    auto fd = brain->config->fieldDimensions;

    double goalX = (fd.length / 2.0); // 풀코트/반코트에 따라...
    double baseX = goalX - distFromGoal; // 풀코트/반코트에 따라...

    double maxY = fd.width / 2.0 - 0.5;
    double bestX = baseX;
    double bestY = 0.0;
    double maxScore = -1e9;

    // 현재 로봇 위치
    auto rPos = brain->data->robotPoseToField;
    double robotX = rPos.x;
    double robotY = rPos.y;
    double robotTheta = rPos.theta;

    // 상대 선수들
    auto Opponents = brain->data->getRobots();

    // 골대 주변 수비수 인덱스 수집
    std::vector<int> defenderIndices;
    for (size_t idx = 0; idx < Opponents.size(); idx++) {
        if (Opponents[idx].label != "Opponent") continue;
        if (std::abs(Opponents[idx].posToField.x - goalX) < 4.0) {
            defenderIndices.push_back(static_cast<int>(idx));
        }
    }

    // opponent 대칭점 계산
    double symTargetY = 0.0;
    if (!defenderIndices.empty()) {
        double totalOppY = 0.0;
        for (int idx : defenderIndices) {
            totalOppY += Opponents[idx].posToField.y;
        }
        symTargetY = -(totalOppY / defenderIndices.size());
    }

    // 후보 지점 탐색 (Offtheball과 동일)
    for (double x = baseX - 2.0; x <= baseX + 2.0; x += 0.1) {
        for (double y = -maxY; y <= maxY; y += 0.1) {

            double distToDefender = 0.0;
            double normalizer = (defenderIndices.size() > 0 ? defenderIndices.size() : 1.0);

            for (const auto& defenderIndex : defenderIndices) {
                double dist = norm(y - Opponents[defenderIndex].posToField.y,
                                   x - Opponents[defenderIndex].posToField.x);
                dist = cap(dist, 3.0, 0.0);
                distToDefender += dist;
            }

            distToDefender /= normalizer;

            double score = 0.0;
            score -= (fabs(x - baseX) * 4.0);
            score -= (fabs(y) * 4.5);
            score -= (fabs(x - robotX) * 2.5);
            score -= (fabs(y - robotY) * 2.5);
            score += (distToDefender * 20.0);

            if (!defenderIndices.empty()) {
                score -= std::abs(y - symTargetY) * 7.5;
            }

            double distToBall = norm(x - brain->data->ball.posToField.x, y - brain->data->ball.posToField.y);
            score -= std::abs(distToBall - 2.5) * 4.5;

            score += (x) * 0.5; // 풀코트/반코트에 따라...

            Line passPath = {brain->data->ball.posToField.x, brain->data->ball.posToField.y, x, y};
            Line shotPath = {baseX, y, goalX, 0.0};

            for (const auto& opponent : Opponents) {
                if (opponent.label != "Opponent") continue;

                rclcpp::Time now = brain->get_clock()->now();
                double elapsed = (now - opponent.timePoint).seconds();
                double confidenceFactor = std::max(0.0, (5.0 - elapsed) / 5.0);
                if (confidenceFactor <= 0.0) continue;

                double timeSinceBall = brain->msecsSince(brain->data->ball.timePoint);

                if (timeSinceBall < 3000) {
                    double distToPassPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, passPath);
                    if (distToPassPath < 1.5) {
                        score -= (1.5 - distToPassPath) * 15.0 * confidenceFactor;
                    }
                }

                double distToShotPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, shotPath);
                if (distToShotPath < 1.5) {
                    score -= (1.5 - distToShotPath) * 3.0 * confidenceFactor;
                }

                Line movementPath = {robotX, robotY, x, y};
                double distRobotTarget = norm(x - robotX, y - robotY);
                if (distRobotTarget > 0.1) {
                    double distToMovementPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, movementPath);
                    if (distToMovementPath < 1.5) {
                        score -= (1.5 - distToMovementPath) * 50.0 * confidenceFactor;
                    }
                }
            }

            double halfGoalW = brain->config->fieldDimensions.goalWidth / 2.0;
            double distToLeftPost = norm(x - goalX, y - halfGoalW);
            double distToRightPost = norm(x - goalX, y + halfGoalW);

            if (distToLeftPost < 0.5) {
                score -= (0.5 - distToLeftPost) * 20.0;
            }
            if (distToRightPost < 0.5) {
                score -= (0.5 - distToRightPost) * 20.0;
            }

            if (score > maxScore) {
                maxScore = score;
                bestX = x;
                bestY = y;
            }
        }
    }

    // Offtheball 결과 목표
    targetx = bestX;
    targety = bestY;

    // targettheta도 Offtheball과 동일한 방식으로 계산
    double angleToGoal = atan2(0.0 - robotY, goalX - robotX);
    double angleToBall;
    if (brain->data->ballDetected) {
        angleToBall = brain->data->ball.yawToRobot + robotTheta;
    } else {
        angleToBall = atan2(brain->data->ball.posToField.y - robotY,
                            brain->data->ball.posToField.x - robotX);
    }

    double angleDiff = toPInPI(angleToGoal - angleToBall);
    targettheta = toPInPI(angleToBall + angleDiff * 0.5);

    // =========================
    // 아래는 GoToPose "기존 이동 제어" 그대로 사용
    // =========================
    double gx = robotX, gy = robotY, gtheta = robotTheta;

    double errorx = targetx - gx;
    double errory = targety - gy;
    double errortheta = toPInPI(targettheta - gtheta);

    double dist = norm(errorx, errory);
    double controlx, controly, controltheta;
    double Kp = 4.0;
    double linearFactor = 1.0 / (1.0 + exp(-6.0 * (dist - 0.5)));

    if(dist > stop_Threshold){
        controltheta = errortheta * Kp;
        controltheta = cap(controltheta, 1.2, -1.2);

        controlx = errorx*cos(gtheta) + errory*sin(gtheta);
        controly = -errorx*sin(gtheta) + errory*cos(gtheta);
        controlx *= linearFactor;
        controly *= linearFactor;
        controlx = cap(controlx, vLimit, -vLimit*0.5);
        controly = cap(controly, vLimit*0.5, -vLimit*0.5);
    }
    else if (fabs(errortheta) > 0.2) {
        controlx = 0;
        controly = 0;
        controltheta = errortheta * Kp;
    }
    else{
        controlx = 0;
        controly = 0;
        controltheta = 0;
    }

    auto color = 0xFFFFFFFF;
    brain->log->setTimeNow();
    brain->log->log(
        "field/defender_offtheball",
        rerun::Arrows2D::from_vectors({{(targetx - gx), -(targety - gy)}})
            .with_origins({{gx, -gy}})
            .with_colors({color}) // Cyan color for pass
            .with_radii(0.01)
    );

    brain->client->setVelocity(controlx, controly, controltheta);
    return NodeStatus::SUCCESS;
}

NodeStatus InitPos::tick(){
    double turn_Threshold;
    double stop_Threshold;
    double vxLimit;
    double vyLimit;
    getInput("turn_threshold", turn_Threshold); 
    getInput("stop_threshold", stop_Threshold); 
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    // 골대중앙위치
    double targetx, targety, targettheta;
    getInput("init_golie_pos_x", targetx); 
    getInput("init_golie_pos_y", targety); 
    getInput("init_golie_pos_theta", targettheta); 

    // 본인 위치
    auto gPos = brain->data->robotPoseToField;
    double gx = gPos.x, gy = gPos.y, gtheta = gPos.theta;

    double errorx = targetx - gx;
    double errory = targety - gy;
    double targetdir = atan2(errory, errorx); // 내 위치에서 골대중앙을 이은 벡터의 각도
    double errortheta = targetdir - gtheta; // 이걸 P제어한다면 골대중앙을 쳐다볼것.

    double dist = norm(errorx, errory); // 골대중앙까지의 거리
    double controlx, controly, controltheta;
    double Kp = 4.0;
    double linearFactor = 1.0 / (1.0 + exp(-6.0 * (dist - 0.5)));

    // 수정
    if(dist > turn_Threshold){ // 직진
      controlx = errorx*cos(gtheta) + errory*sin(gtheta);
      controly = -errorx*sin(gtheta) + errory*cos(gtheta);
      controlx *= linearFactor;
      controly *= linearFactor;
      controlx = cap(controlx, vxLimit, -vxLimit*0.5);    
      controly = cap(controly, vyLimit, -vyLimit);
      controltheta = errortheta * Kp;
    }
    else if(dist < turn_Threshold && dist > stop_Threshold){ // 선회
		  controlx = errorx*cos(gtheta) + errory*sin(gtheta);
      controly = -errorx*sin(gtheta) + errory*cos(gtheta);
      controlx *= linearFactor;
      controly *= linearFactor;
      controlx = cap(controlx, vxLimit, -vxLimit*0.5);    
      controly = cap(controly, vyLimit, -vyLimit);
	    controltheta = (targettheta - gtheta) * Kp; // 이러면 gtheta(로봇방향)이 targettheta를 바라봄
    }
    
    else if(dist < turn_Threshold && dist < stop_Threshold){ // 정지
        controlx = 0;
        controly = 0;
        controltheta = 0;
    }

	brain->client->setVelocity(controlx, controly, controltheta, false, false, false);
    return NodeStatus::SUCCESS;
}