#include "brain.h"
#include "brain_tree.h"
#include "role_manager.h"

using namespace BT;

#define REGISTER_ROLEMANAGER_BUILDER(Name)     \
    factory.registerBuilder<Name>(             \
        #Name,                                 \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void RegisterRoleManager(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_ROLEMANAGER_BUILDER(RoleManager)
    REGISTER_ROLEMANAGER_BUILDER(SetPieceRoleManager)
}

RoleManager::RoleManager(const std::string& name, const NodeConfig& config, Brain* _brain)
: SyncActionNode(name, config),
  brain(_brain),
  timeLastTick(_brain->get_clock()->now())   // 또는 now - 3초
{}

SetPieceRoleManager::SetPieceRoleManager(const std::string& name, const NodeConfig& config, Brain* _brain)
: SyncActionNode(name, config),
  brain(_brain)
{}


NodeStatus RoleManager::tick() {
    auto dt = brain->msecsSince(timeLastTick);
    if(dt<1000) return NodeStatus::SUCCESS;
    timeLastTick = brain->get_clock()->now();

    // 1. 현재 게임 정보를 가져온다
    auto ball = brain->data->ball;
    double ballX = ball.posToField.x;

    int NumIsRace=0, NumIsPossession=0, NumAintRace=0, NumAintPossession=0;
    bool isRace=false, isPossession=false;

    int numOfPlayers = brain->config->numOfPlayers;
    bool isLead = brain->tree->getEntry<bool>("is_lead");

    int numAliveVotes = 0;
    for (int Idx = 0; Idx < numOfPlayers; ++Idx) {
        const auto& tm = brain->data->tmStatus[Idx];
        if (!tm.isAlive) continue;

        ++numAliveVotes;
        if (tm.isRace) ++NumIsRace;
        else ++NumAintRace;

        if (tm.isPossession) ++NumIsPossession;
        else ++NumAintPossession;
    }

    if (numAliveVotes > 0) {
        isRace = (NumIsRace >= NumAintRace);
        isPossession = (NumIsPossession >= NumAintPossession);
    } else {
        // alive 투표가 하나도 없으면 self 상태(없으면 직전 BB 값)로 fallback
        const int selfIdx = brain->config->playerId - 1;
        if (selfIdx >= 0 && selfIdx < numOfPlayers && brain->data->tmStatus[selfIdx].isAlive) {
            isRace = brain->data->tmStatus[selfIdx].isRace;
            isPossession = brain->data->tmStatus[selfIdx].isPossession;
        } else {
            isRace = brain->tree->getEntry<bool>("is_race");
            isPossession = brain->tree->getEntry<bool>("is_possession");
        }
    }


    brain->tree->setEntry("is_race", isRace);
    brain->tree->setEntry("is_possession", isPossession);

    // 2. 역할 결정 로직
    // find details in notion page -> 'Dynamic Role Change를 위한 BT대개조'

    std::string currentRole;
    std::string defaultRole = brain->config->playerRole;
    std::string Role = brain->tree->getEntry<std::string>("player_role");
    // Role and defaultRole are normally same... 
    // unless you are the last surviver. 
    // In this case brain.cpp will change your role into goal_keeper no matter what your defaultRole was.

    if(defaultRole == "goal_keeper" || Role == "goal_keeper") currentRole = "goal_keeper";
    else if(isRace){
        if(isLead){
            //TODO : if 드리블가능->드리블. else->걷어내기
            //일단은 미구현상태니깐...
            currentRole = "lead_defender";
        }
        else{ // non-lead
            //TODO : 협력
            currentRole = "sub_defender";
        }
    }
    else{
        if(isLead){
            if(isPossession) currentRole = "lead_striker";
            else currentRole = "lead_defender";
        }
        else{ // non-lead case
            if(isPossession) currentRole = "sub_striker";
            else currentRole = "sub_defender";
        }
    }

    // 블랙보드 업데이트
    setOutput("current_role", currentRole);

    brain->log->logToScreen(
    "tree/RoleManager",
    format("Current Role: %s, Lead: %s, Possession: %s",
           currentRole.c_str(),
           isLead ? "true" : "false",
           isPossession ? "true" : "false"),
    0xFF00FFFF);

    return NodeStatus::SUCCESS;
}

NodeStatus SetPieceRoleManager::tick() {
    const bool isRace = brain->tree->getEntry<bool>("is_race");
    const bool isPossession = brain->tree->getEntry<bool>("is_possession");
    const bool isLead = brain->tree->getEntry<bool>("is_lead");

    const std::string defaultRole = brain->config->playerRole;
    const std::string role = brain->tree->getEntry<std::string>("player_role");

    std::string currentRole;
    if (defaultRole == "goal_keeper" || role == "goal_keeper") {
        currentRole = "goal_keeper";
    } else if (isRace) {
        currentRole = isLead ? "lead_defender" : "sub_defender";
    } else {
        if (isLead) currentRole = isPossession ? "lead_striker" : "lead_defender";
        else        currentRole = isPossession ? "sub_striker" : "sub_defender";
    }

    // ---- 추가 ----
    std::string desiredRole = currentRole;
    static std::string stableRole = desiredRole;
    static std::string lastDesired = desiredRole;
    static int sameCount = 0;

    if (desiredRole == stableRole) {
        sameCount = 0; 
    } else {
        if (desiredRole == lastDesired) {
            sameCount++;
        } else {
            sameCount = 1;
            lastDesired = desiredRole;
        }

        if (sameCount >= 50) {  
            stableRole = desiredRole;
            sameCount = 0;
        }
    }

    setOutput("current_role", stableRole);

    // brain->log->logToScreen("tree/SetPieceRoleManager", "Current Role: " + stableRole, 0x00FFFFFF);
    brain->log->logToScreen(
    "tree/SetpieceRoleManager",
    format("Current Role: %s, Lead: %s, Possession: %s",
           stableRole.c_str(),
           isLead ? "true" : "false",
           isPossession ? "true" : "false"),
    0xFF00FFFF);

    return NodeStatus::SUCCESS;
}
