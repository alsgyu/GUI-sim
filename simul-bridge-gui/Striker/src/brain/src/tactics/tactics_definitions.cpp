#include "brain.h"
#include "tactics/tactics_definitions.h"
#include "utils/math.h"
#include <chrono>

using namespace std;

// ==========================================
// 전략 1. 압박 전략 (Pressing)
// ==========================================
PortsList TacticPressing::providedPorts()
{
    return {};
}

NodeStatus TacticPressing::tick()
{    
    // Context Flags from TacticSelector
    bool isCounter = brain->tree->getEntry<bool>("Context.isCounterOpportunity");
    bool isWinning = brain->tree->getEntry<bool>("Context.isWinningComfortably");

    double chaseSpeed = 0.8; // Default

    if (isCounter) {
        chaseSpeed = 1.0;
    } else if (isWinning) {
        chaseSpeed = 0.4;
    }

    // Set Parameter: Chase Speed
    brain->tree->setEntry("Strategy.param_chase_speed_limit", chaseSpeed);
    
    // Set Parameter: Aggression (Simplified visual log or flag)
    brain->log->logToScreen("tactics", isCounter ? "Aggression: HIGH" : "Aggression: NORMAL", 0x00FF00FF);

    return NodeStatus::SUCCESS; // Always SUCCESS for Sequence
}

// ==========================================
// 전략 2. 수비라인 메이킹 (Line Defense - Striker Mid-Block)
// ==========================================
PortsList TacticLineDefense::providedPorts()
{
    return {
        InputPort<double>("dist_from_goal", 3.0, "수비 라인 거리")
    };
}

NodeStatus TacticLineDefense::tick()
{
    // Context Flags
    bool isDesperate = brain->tree->getEntry<bool>("Context.isDesperate");
    bool isWinning = brain->tree->getEntry<bool>("Context.isWinningComfortably");
    bool isCounter = brain->tree->getEntry<bool>("Context.isCounterOpportunity");

    double defenseLine = 0.0; // Default (Half line)

    if (isDesperate) {
        defenseLine = 4.5; // Stay at opponent goal
    } else if (isWinning) {
        defenseLine = -3.0; // Deep defense
    } else if (isCounter) {
        defenseLine = 0.0; // Push up
    } else {
        // Normal dynamic line
        defenseLine = 1.0; // Standard pressing line
    }

    // Set Parameter: Defense Line
    brain->tree->setEntry("Strategy.param_defense_line_x", defenseLine);

    // 디버깅: 설정된 파라미터 시각화
    brain->log->log("debug/tactics/param_defense_line", 
         rerun::Arrows2D::from_vectors({{0.0, 1.0}}) 
         .with_origins({{defenseLine, 0.0}})
         .with_colors(0x0000FFFF)
         .with_labels({"DesiredLine"})
    );

    return NodeStatus::SUCCESS;
}

// ==========================================
// 전략 3. 결정력 제어 (Finishing)
// ==========================================
PortsList TacticFinishing::providedPorts()
{
    return {};
}

NodeStatus TacticFinishing::tick()
{
    // Context Flags
    bool isDesperate = brain->tree->getEntry<bool>("Context.isDesperate");
    bool isCounter = brain->tree->getEntry<bool>("Context.isCounterOpportunity");

    double kickThreshold = 0.3; // Default

    if (isDesperate) {
        kickThreshold = 0.5; // Shoot on sight
    } else if (isCounter) {
        kickThreshold = 0.2; // Precise finish needed for counter
    }

    // Set Parameter: Kick Threshold
    brain->tree->setEntry("Strategy.param_kick_threshold", kickThreshold);

    return NodeStatus::SUCCESS;
}

// ==========================================
// 전략 4. 템포 컨트롤 (Tempo Control)
// ==========================================
PortsList TacticTempoControl::providedPorts()
{
    return {};
}

NodeStatus TacticTempoControl::tick()
{    
    // 천천히 움직임
    brain->tree->setEntry("Strategy.param_chase_speed_limit", 0.4);
    
    // 공을 오래 소유 (킥 임계값 매우 높게 -> 드리블 유도)
    brain->tree->setEntry("Strategy.param_kick_threshold", 0.05); 
    
    // 수비 라인은 현 위치 유지
    brain->tree->setEntry("Strategy.param_defense_line_x", brain->data->robotPoseToField.x);

    setStatus(NodeStatus::RUNNING);
    return NodeStatus::RUNNING;
}

// ==========================================
// 전략 5. 총공격 (ALL_OUT_ATTACK)
// ==========================================
PortsList TacticAllOut::providedPorts()
{
    return {};
}

NodeStatus TacticAllOut::tick()
{
    // 1. 최대 속도
    brain->tree->setEntry("Strategy.param_chase_speed_limit", 1.2);

    // 2. 킥 임계값 대폭 완화
    brain->tree->setEntry("Strategy.param_kick_threshold", 0.5); 

    // 3. 수비 라인 극단적 전진 -> 상대 골라인(4.5m) 까지 전진 - 사실상 수비 복귀 안함
    brain->tree->setEntry("Strategy.param_defense_line_x", 4.5);

    // 디버깅
    brain->log->logToScreen("tactics", "!!! ALL OUT ATTACK !!!", 0xFF0000FF); // 빨간색

    return NodeStatus::SUCCESS; // SyncActionNode이므로 SUCCESS/FAILURE 반환
}

// ==========================================
// 전략 6. 역습 (Counter Attack - Strategy: OFFENSIVE)
// ==========================================
NodeStatus TacticCounterAttack::tick()
{
    // 빠른 역습: 속도 최대
    brain->tree->setEntry("Strategy.param_chase_speed_limit", 1.0);
    
    // 슛은 신중하게 (확실한 찬스 전까진 드리블 유도)
    brain->tree->setEntry("Strategy.param_kick_threshold", 0.2); 

    // 수비 라인은 하프라인 근처로 설정하여 즉시 압박 준비
    brain->tree->setEntry("Strategy.param_defense_line_x", 0.0);

    return NodeStatus::SUCCESS;
}

// ==========================================
// 전략 7. 텐백 수비 (Deep Defense - Strategy: DEFENSIVE / TIME_WASTING)
// ==========================================
NodeStatus TacticDeepDefense::tick()
{
    // 골대 앞을 지키는 것에 집중 (움직임 최소화)
    brain->tree->setEntry("Strategy.param_chase_speed_limit", 0.3);
    
    // 걷어내기 위주 (각도 상관없이 뻥 차기)
    brain->tree->setEntry("Strategy.param_kick_threshold", 0.8); 

    // 골라인 바로 앞까지 수비 라인 내림 (Park the Bus)
    brain->tree->setEntry("Strategy.param_defense_line_x", -4.0);
    
    // 디버깅
    brain->log->logToScreen("tactics", "PARK THE BUS", 0x0000FFFF);

    return NodeStatus::SUCCESS;
}
