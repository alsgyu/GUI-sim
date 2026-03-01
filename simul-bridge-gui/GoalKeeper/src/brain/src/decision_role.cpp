#include "brain.h"
#include "decision_role.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

#define REGISTER_DECISION_ROLE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterDecisionRoleNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_DECISION_ROLE_BUILDER(GoalieDecide)
    REGISTER_DECISION_ROLE_BUILDER(GoalieClearingDecide)
}

NodeStatus GoalieDecide::tick()
{
    string lastDecision;
    double ctPosx, ctPosy;
    double closer_margin;
    double clearing_max;
    getInput("decision_in", lastDecision); 
    getInput("ctPosx", ctPosx);
    getInput("ctPosy", ctPosy);
    getInput("closer_margin", closer_margin);
    getInput("clearing_max", clearing_max);

    // 공 위치 신뢰 (아직 사용 X)
    bool iKnowBallPos      = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    bool ballKnown = (iKnowBallPos || tmBallPosReliable);

    // 기본값: hold
    string newDecision = "hold";
    auto color = 0xFFFFFFFF;

    // 위치들 (필드 좌표계)
    auto bPos = brain->data->ball.posToField; 
    auto gPos = brain->data->robotPoseToField; 

    // 거리 계산
    double distGKToBall   = norm(bPos.x - gPos.x, bPos.y - gPos.y);
    double distBallToGoal = norm(bPos.x - ctPosx, bPos.y - ctPosy);
    double distGKToGoal = norm(gPos.x - ctPosx, gPos.y - ctPosy);

    // 상대 수집
    auto rPos = brain->data->getRobots();
    double distOppToBallMin = 1e9; // 상대 로봇들 중에서 공에 가장 가까운 상대까지의 최소 거리
    bool hasOpponent = false;
    for (const auto& r : rPos) {
        if (r.label != "Opponent") continue;
        hasOpponent = true;
        double d = norm(bPos.x - r.posToField.x, bPos.y - r.posToField.y);
        if (d < distOppToBallMin) distOppToBallMin = d;
    }

    // -----------------------------
    // 판정 기준
    // -----------------------------
    // 공이 골대로부터 일정 거리 이내로 들어옴
    bool ballInClearingZone = (distBallToGoal < clearing_max * (lastDecision == "normal_clearing" ? 1.2 : 1.0)); 
	// 공이 적보다 골키퍼와 가까이에 위치한다
    bool iAmCloser = (!hasOpponent) ? true : (distGKToBall + closer_margin < distOppToBallMin); 
    // 로봇이 골대로부터 너무 멀리 나가지 않도록
    bool StopClearing = distGKToGoal > clearing_max;
    // 로봇이 공을 그냥 걷어낼 수 있다
    bool FastClearOk = std::abs(gPos.x) > std::abs(bPos.x); 

    // -----------------------------
    // 판정 결과
    // -----------------------------
	if ((!ballKnown) && (lastDecision == "normal_clearing" || lastDecision == "critical_clearing" || lastDecision == "find")) {
	    newDecision = "find";
	}
	else if (ballInClearingZone && iAmCloser && (!StopClearing)) {
	    newDecision = FastClearOk ? "normal_clearing" : "critical_clearing";
	}
	else {
	    newDecision = "hold";
	}

    setOutput("decision_out", newDecision);
    
    // 간략한 로그
    brain->log->logToScreen(
        "tree/GoalieDecide",
        format("Decision:%s",
        newDecision.c_str()),
        color
    );
    
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieClearingDecide::tick()
{
    double chaseRangeThreshold;
    double ctPosx,ctPosy;
    auto color = 0xFFFFFFFF;
    getInput("chase_threshold", chaseRangeThreshold);
    getInput("ctPosx", ctPosx);
    getInput("ctPosy", ctPosy);

    std::string lastDecision;
    getInput("decision_in", lastDecision);

    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw   = ball.yawToRobot;

    std::string newDecision;

    // 멀면 chase 유지(히스테리시스 약간)
    if (ballRange > chaseRangeThreshold * (lastDecision == "clearing_chase" ? 0.9 : 1.0)) {
        newDecision = "clearing_chase";
        color = 0xFFFFFFFF;
    } else {
        // 가까우면 원터치 kick (adjust 없음)
        newDecision = "clearing_kick";
        color = 0x00FF00FF;
    }
    
    brain->log->logToScreen(
        "tree/GoalieClearingDecide",
        format("Decision:%s",
        newDecision.c_str()),
        color
    );

    setOutput("decision_out", newDecision);
    return NodeStatus::SUCCESS;
}
