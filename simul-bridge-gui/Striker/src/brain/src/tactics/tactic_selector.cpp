#include "brain.h"
#include "tactics/tactic_selector.h"

PortsList TacticSelector::providedPorts()
{
    return { OutputPort<string>("active_tactic") };
}

NodeStatus TacticSelector::tick()
{
    // 1. 블랙보드에서 현재 전략 읽어오기
    string strategyMode = brain->tree->getEntry<string>("Strategy.currentMode");

    // 2. Context Flag 초기화
    bool isDesperate = false;
    bool isCounter = false;
    bool isWinning = false;

    // 3. 변수 선언
    int myScore = brain->data->gameControlData.score[0];
    int oppScore = brain->data->gameControlData.score[1];
    int scoreDiff = myScore - oppScore;
    double ballX = brain->data->ball.posToField.x;

    // =============================== [Strategy -> Context Analysis] ===============================
    if (strategyMode == "ALL_OUT_ATTACK") { 
        isDesperate = true;
    } 
    else if (strategyMode == "OFFENSIVE") { 
        if (scoreDiff <= -3) {
            isDesperate = true;
        } 
        else if (ballX < -1.0) {
            isCounter = true;
        }
    } 
    else if (strategyMode == "DEFENSIVE") { 
        if (scoreDiff >= 3) {
            isWinning = true; 
        } 
    } 
    else if (strategyMode == "TIME_WASTING") {
        isWinning = true;
    }

    // 4. 블랙보드에 Context Flags 주입
    brain->tree->setEntry("Context.isDesperate", isDesperate);
    brain->tree->setEntry("Context.isCounterOpportunity", isCounter);
    brain->tree->setEntry("Context.isWinningComfortably", isWinning);

    return NodeStatus::SUCCESS;
}
