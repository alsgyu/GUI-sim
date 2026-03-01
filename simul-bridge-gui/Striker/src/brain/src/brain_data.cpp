#include "brain_data.h"
#include "utils/math.h"

BrainData::BrainData(){ std::fill(std::begin(penalty), std::end(penalty), SUBSTITUTE); }

vector<GameObject> BrainData::getMarkingsByType(set<string> types) {
    if (types.size() == 0) return getMarkings();

    // else
    vector<GameObject> res = {}; 
    auto markings = getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        if (types.count(markings[i].label) > 0) res.push_back(markings[i]);
    }
    
    return res;
}

vector<FieldMarker> BrainData::getMarkersForLocator(){
    vector<FieldMarker> res;
    auto markings = getMarkings();
    for (size_t i = 0; i < markings.size(); i++){
        auto label = markings[i].label;
        auto x = markings[i].posToRobot.x;
        auto y = markings[i].posToRobot.y;
        auto confidence = markings[i].confidence;

        char markerType;
        if (label == "LCross")
            markerType = 'L';
        else if (label == "TCross")
            markerType = 'T';
        else if (label == "XCross")
            markerType = 'X';
        else if (label == "PenaltyPoint")
            markerType = 'P';

        res.push_back(FieldMarker{markerType, x, y, confidence});
    }
    return res;
}

Pose2D BrainData::robot2field(const Pose2D &poseToRobot){
    Pose2D poseToField;
    transCoord(
        poseToRobot.x, poseToRobot.y, poseToRobot.theta,
        robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
        poseToField.x, poseToField.y, poseToField.theta);
    poseToField.theta = toPInPI(poseToField.theta);
    return poseToField;
}

Pose2D BrainData::field2robot(const Pose2D &poseToField){
    Pose2D poseToRobot;
    double xfr, yfr, thetafr; // fr = field to robot
    yfr = sin(robotPoseToField.theta) * robotPoseToField.x - cos(robotPoseToField.theta) * robotPoseToField.y;
    xfr = -cos(robotPoseToField.theta) * robotPoseToField.x - sin(robotPoseToField.theta) * robotPoseToField.y;
    thetafr = -robotPoseToField.theta;
    transCoord(
        poseToField.x, poseToField.y, poseToField.theta,
        xfr, yfr, thetafr,
        poseToRobot.x, poseToRobot.y, poseToRobot.theta);
    return poseToRobot;
}

// 1.10 - 로봇 메모리 업데이트
void BrainData::updateRobots(const vector<GameObject>& newObservations, double retentionTime) {
    std::lock_guard<std::mutex> lock(_robotsMutex);
    
    // 현재 시간 (가장 최신 관측값의 시간 사용, 없으면 시스템 시간)
    rclcpp::Time now;
    if (!newObservations.empty()) {
        now = newObservations[0].timePoint;
    } else {
        now = rclcpp::Clock(RCL_ROS_TIME).now();
    }

    // 1. Clean up stale robots first
    // retentionTime이 0보다 크면 사용
    if (retentionTime > 0) {
        for (auto it = _robots.begin(); it != _robots.end(); ) {
            double age = (now - it->timePoint).seconds();
            if (age > retentionTime) {
                it = _robots.erase(it);
            } else {
                ++it;
            }
        }
    }

    // 2. 새 관측값으로 기존 로봇 업데이트 혹은 추가
    // Best Match Strategy: 가장 가까운 로봇 하나만 업데이트
    for (const auto& newObj : newObservations) {
        int bestIdx = -1;
        double minDistance = 1.0; // 매칭 임계값 (1m)

        for (int i = 0; i < _robots.size(); i++) {
            double dist = norm(newObj.posToField.x - _robots[i].posToField.x, 
                               newObj.posToField.y - _robots[i].posToField.y);
            if (dist < minDistance) {
                minDistance = dist;
                bestIdx = i;
            }
        }

        if (bestIdx != -1) {
             _robots[bestIdx] = newObj;
        } else {
            _robots.push_back(newObj);
        }
    }

    // 3. 중복 제거 (Self-Merge)
    // 메모리 상의 로봇들이 서로 너무 가까우면 병합
    for (int i = 0; i < (int)_robots.size(); i++) {
        for (int j = i + 1; j < (int)_robots.size(); ) {
             double dist = norm(_robots[i].posToField.x - _robots[j].posToField.x, 
                                _robots[i].posToField.y - _robots[j].posToField.y);
             
             if (dist < 1.0) { // 1m 이내면 같은 로봇으로 간주
                 // 최신 정보로 병합 (Keep the newer one)
                 if (_robots[i].timePoint.nanoseconds() < _robots[j].timePoint.nanoseconds()) {
                     _robots[i] = _robots[j]; 
                 }
                 // Remove j
                 _robots.erase(_robots.begin() + j);
             } else {
                 j++;
             }
        }
    }
}