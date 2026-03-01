#include "brain.h"
#include "brain_tree.h"
#include "pass.h"
#include "utils/math.h"

#define REGISTER_PASS_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void RegisterPassNodes(BT::BehaviorTreeFactory &factory, Brain* brain) {
    REGISTER_PASS_BUILDER(CalcPassDir)
}

NodeStatus CalcPassDir::tick(){
    double maxpassThreshold;
    double minpassThreshold;
    double scoreThreshold;
    
    // Default values if input is missing
    if (!getInput("max_pass_threshold", maxpassThreshold)) maxpassThreshold = 4.0;
    if (!getInput("min_pass_threshold", minpassThreshold)) minpassThreshold = 1.0;
    if (!getInput("score_threshold", scoreThreshold)) scoreThreshold = -5.0;

    auto bPos = brain->data->ball.posToField; // 공 위치
    int bestTeammateIdx = -1;
    int myPlayerId = brain->config->playerId - 1; // 1,2,3 -> 0,1,2
    double maxTmSelectScore = -99999.0;
    auto fd = brain->config->fieldDimensions; // 필드 정보
    auto color = 0xFFFFFFFF;

    // 가장 가까운(혹은 적절한) 팀원 찾기
    for(int i=0; i<HL_MAX_NUM_PLAYERS; i++){
        // 나 자신 제외, 살아있는 팀원 확인
        if(i + 1 == brain->config->playerId) continue;
        if(!brain->data->tmStatus[i].isAlive) continue; 
        
        // 팀원 위치
        auto tmPos = brain->data->tmStatus[i].robotPoseToField;
        
        // 거리 계산
        double dist = norm(bPos.x - tmPos.x, bPos.y - tmPos.y);

        // 유효 거리 내에 있고, 가장 score가 높은 팀원 선택
        // 나랑 가까울 수록 높은 score, 전방에 있는(x좌표가 낮음) 선수일수록 높은 score
        if( dist > minpassThreshold && dist < maxpassThreshold){
            double score = -dist * 1.0 + tmPos.x * 1.2; // 풀코트/반코트 따라...
            if(score > maxTmSelectScore){
                maxTmSelectScore = score;
                bestTeammateIdx = i;
            }
        }
    }

    // 패스가 가능한지 ? 아니라면 다음으로 (bestTeammateIdx가 한명이라도 있고, 목표 범위 내에 로봇이 왔는지 확인)
    bool passFound = (bestTeammateIdx != -1);
    setOutput("pass_found", passFound);

    if(!passFound){
        // Output default pass speed limit just in case
        setOutput("pass_speed_limit", 1.0); 
        brain->data->tmStatus[myPlayerId].passSignal = false;
        brain->data->tmStatus[myPlayerId].passTargetX = 0.;
        brain->data->tmStatus[myPlayerId].passTargetY = 0.;
        return NodeStatus::SUCCESS;
    }
    
    brain->data->kickType = "pass"; // 킥 타입 설정
    auto tmPos = brain->data->tmStatus[bestTeammateIdx].robotPoseToField; // 패스를 주기로 결정한 그 팀원 위치
    auto Opponents = brain->data->getRobots(); // Opponents. 사용하려면 label == Opponent 확인해야함
    
    double fieldlimitx = fd.length / 2.0;
    double fieldlimity = fd.width / 2.0;
    double maxScore = -99999.0;
    double tx, ty; // 타겟 좌표
    
    for (double x = tmPos.x - 3; x <= tmPos.x + 3; x += 0.2){
        for (double y = tmPos.y - 2.5; y <= tmPos.y + 2.5; y += 0.2){
            if (fabs(x) > fieldlimitx || fabs(y) > fieldlimity) continue; // 필드 범위를 벗어난다면 continue
            double passDist = norm(x - bPos.x, y - bPos.y);
            if (passDist < minpassThreshold || passDist > maxpassThreshold) continue; // 너무 숏패스, 롱패스는 시도안함
            
            double score = 10.0
                        - (fabs(x - tmPos.x) * 1.1) // 팀원과의 거리
                        - (fabs(y - tmPos.y) * 0.7) // 팀원과의 거리
                        + x * 0.95 // x좌표가 높을수록 높은 score... 풀코트/반코트 따라.
                        - (fabs(y) * 0.45); // y좌표가 낮을수록 높은 score
            
            Line passPath = {bPos.x, bPos.y, x, y};

            for (const auto& opponent : Opponents){
                if (opponent.label != "Opponent") continue; // 상대팀이 아니면 스킵
                
                rclcpp::Time now = brain->get_clock()->now();
                double elapsed = (now - opponent.timePoint).seconds(); // 수비수를 마지막으로 본지 몇초 지났나
                double confidenceFactor = std::max(0.0, (5.0 - elapsed) / 5.0); // 시간이 지날수록 신뢰도 떨어지게
                if (confidenceFactor <= 0.0) continue;

                double distToPassPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, passPath);
                if (distToPassPath < 1.15){ // 경로선분 1.15미터 이내에 opponent가 있다면
                    score -= (1.15 - distToPassPath) * 24.0 * confidenceFactor; // 추가
                }
                
            }

            // score가 가장 높은 타겟 선택
            if (score > maxScore){
                maxScore = score;
                tx = x;
                ty = y;
            }
        }
    }
    if (maxScore < scoreThreshold){ // 패스결심이 안선다면 패스하지 않음
        setOutput("pass_found", false);
        brain->data->tmStatus[myPlayerId].passSignal = false;
        brain->data->tmStatus[myPlayerId].passTargetX = 0.;
        brain->data->tmStatus[myPlayerId].passTargetY = 0.;
        // rerun log
        brain->log->log("field/pass_dir", 
            rerun::TextLog(format("Decided not to pass. Score: %.2f", maxScore))
        );
        return NodeStatus::SUCCESS;
    }
    brain->data->tmStatus[myPlayerId].passSignal = true;
    brain->data->tmStatus[myPlayerId].passTargetX = tx;
    brain->data->tmStatus[myPlayerId].passTargetY = ty;

    // 공 -> 타겟 방향으로 킥
    brain->data->kickDir = atan2(ty - bPos.y, tx - bPos.x);
    brain->data->passDir = atan2(ty - bPos.y, tx - bPos.x);
    
    // 거리에 비례한 speed limit 설정
    double d = norm(bPos.x - tx, bPos.y - ty);

    // 파라미터
    double k = 1.7;        // 감도

    // 경험적 식...
    double passSpeed = k * std::sqrt(d / 10.0);

    passSpeed = std::min(passSpeed, 2.0);

    // BT로 전달 (Kick에서 speed_limit으로 사용)
    setOutput("pass_speed_limit", passSpeed);

    // 디버그 로그
    brain->log->logToScreen(
        "debug/pass_speed",
        format("PassPower d=%.2f -> speed_limit=%.2f", d, passSpeed),
        0x00FFFFFF
    );


    brain->log->logToScreen("debug/Pass", format("Passing to TM %d to (%.2f, %.2f)", bestTeammateIdx+1, tx, ty), 0x00FF00FF);

    // 시각화
    brain->log->setTimeNow();
    brain->log->log(
        "field/pass_dir",
        rerun::Arrows2D::from_vectors({{(tx - bPos.x), -(ty - bPos.y)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({0x00FFFFFF}) // Cyan color for pass
            .with_radii(0.01)
    );
    brain->log->log("field/pass_dir", 
            rerun::TextLog(format("Decided to make a pass. Score: %.2f", maxScore))
    );

    return NodeStatus::SUCCESS;
}
