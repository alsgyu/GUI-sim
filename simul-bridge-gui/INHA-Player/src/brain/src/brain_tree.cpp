#include <cmath>
#include <cstdlib>

#include "brain_tree.h"
#include "locator.h"
#include "brain.h"

#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"

#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <ios>
#include "gotopose.h"

// Strategy
#include "strategy/strategy_nodes.h"
// Tactics
#include "tactics/tactics_nodes.h"

#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void BrainTree::init(){
    BehaviorTreeFactory factory;

    brain->registerWalkNodes(factory); // walk 관련 노드 등록
    brain->registerMoveHeadNodes(factory); // head move 관련 노드 등록
    brain->registerLocatorNodes(factory); // locator 관련 노드 등록
    brain->registerChaseNodes(factory); // chase 관련 노드 등록
    RegisterKickNodes(factory, brain); // kick 관련 노드 등록
    brain->registerAdjustNodes(factory); // adjust 관련 노드 등록
    brain->registerSpeakNodes(factory); // speak 관련 노드 등록
    brain->registerOfftheballNodes(factory); // offtheball 관련 노드 등록
    brain->registerDefenderDecisionNodes(factory); // defender decision 관련 노드 등록
    brain->registerStrikerDecisionNodes(factory); // striker decision 관련 노드 등록
    brain->registerGoalieDecisionNodes(factory); // goalie decision 관련 노드 등록
    brain->registerRoleManager(factory); // role manager 관련 노드 등록
    RegisterGotoposeNodes(factory, brain); // gotopose node registration
    brain->registerPassReceiveNodes(factory); // pass receive 노드 등록
    brain->registerPassNodes(factory); // pass direction node 등록
    brain->registerSetpieceNodes(factory); // setpiece node 등록

    
    // NEW: Register Strategy & Tactics Nodes
    RegisterStrategyNodes(factory, brain);
    RegisterTacticsNodes(factory, brain);
    
    
    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // 여기서 블랙보드가 초기화됨
    initEntry();
}

void BrainTree::initEntry(){
    // 여기서 블랙보드를 초기화하면 됨 
    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<string>("gc_game_state", ""); // 현재 ready,set,play,end 중 하나일 것 
    setEntry<string>("gc_game_secondary_state", "NONE");
    setEntry<string>("gc_game_secondary_state_info", "STOP");
    setEntry<bool>("gc_is_our_kickoff", false); // 우리 팀 킥오프인지
    setEntry<bool>("gc_is_our_secondary_state", false);
    setEntry<bool>("gc_is_under_penalty", false);
    setEntry<int>("control_state", 3); // control_state == 1 이면 단순 걷기로 (1->3 play.xml test)
    
    setEntry<string>("decision", ""); // pass / chase / adjust / kick / assist 더 추가 예정
    setEntry<bool>("wait_for_opponent_kickoff", false);
    setEntry<int>("player_id", brain->config->playerId); // 1 or 2 or 3.
    setEntry<string>("player_role", brain->config->playerRole); // play.xml (field_player / goal_keeper)
    setEntry<string>("current_role", "none"); // goal_keeper/lead_striker/sub_striker/lead_defender/sub_defender/none
    setEntry<bool>("is_race", false); // 경합상황
    setEntry<bool>("is_possession", true); // 볼 점유 여부
    brain->log->logToScreen("debug/Blackboard", "BB Role: " + brain->config->playerRole, 0x00FF00FF);


    // 실제 경기 중 상황 
    // 공 
    setEntry<bool>("ball_out", false); // 공이 필드 밖으로 나갔는지  
    setEntry<bool>("ball_location_known", false); // 공 위치를 알고 있는지
    setEntry<bool>("tm_ball_pos_reliable", false); // 팀원 공 위치 정보가 신뢰할 만한지 확인
    setEntry<double>("ball_range", 0); // 공과의 거리

    //  승재욱 추가 : chase -> adjust -> kick
    setEntry<string>("striker_state", "chase");   

    // 위치 추정 보정 여부
    setEntry<bool>("odom_calibrated", false);

    // tactics 비활성화에 따른 파라미터 초기값 하드코딩
    setEntry<double>("Strategy.param_chase_speed_limit", 1.0);
}

void BrainTree::tick(){ tree.tickOnce(); }
