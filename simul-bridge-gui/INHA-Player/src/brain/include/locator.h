
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <map>
#include <rerun.hpp>
#include <string>
#include <unordered_set>
#include <vector>

#include "types.h"
#include "utils/hungarian.h"

using namespace std;
namespace chr = std::chrono;

class Locator {
public:
  struct Particle {
    double x;
    double y;
    double theta;
    double weight;
    int id = -1;
  };

  Pose2D lastOdom = {0, 0, 0};
  Pose2D bestPose = {0, 0, 0};
  Pose2D pose = {0, 0, 0};

  FieldDimensions fieldDimensions;
  HungarianAlgorithm hungarian;

  vector<Particle> particles;
  vector<FieldMarker> fieldMarkers;
  vector<int> assignment;
  vector<double> flatCostMatrix;
  vector<FieldMarker> obsInField;
  vector<double> cdf;
  map<char, vector<FieldMarker>> obsByType;
  map<char, vector<FieldMarker>> mapByType;

  bool hasPose = false;
  bool isInitialized = false;
  int numParticles = 150;
  double initFieldMargin = 1.0;
  double alpha1 = 0.08;
  double alpha2 = 0.005;
  double alpha3 = 0.005;
  double alpha4 = 0.005;
  double clusterDistThr = 0.3;
  double clusterThetaThr = 0.35;
  double clusterMinWeight = 0.05;
  double orientationGatingThr = 1.6;
  double smoothAlpha = 0.4;
  double enableLog = true;
  double invNormVar = 1.4;
  double invPerpVar = 4.0;
  double likelihoodWeight = 0.3;
  double unmatchedPenaltyConfThr = 0.6;
  double essThreshold = 0.4;

  rerun::RecordingStream *logger = nullptr;

  bool getIsInitialized() const { return isInitialized; }
  void calcFieldMarkers(FieldDimensions fd);
  void init(FieldDimensions fd, bool enableLogParam, string logIPParam);
  void logParticles(double);
  void setLog(rerun::RecordingStream *stream);
  void globalInit(Pose2D currentOdom);
  void predict(Pose2D currentOdom);
  void correct(const vector<FieldMarker> markers);
  void clusterParticles();
  void setParams(int numParticles, double initMargin, std::vector<double> alphas, double smoothAlpha, double invObsVarX, double invObsVarY,
                 double likelihoodWeight, double unmatchedPenaltyConfThr, double essThreshold, double clusterDistThr, double clusterThetaThr,
                 double clusterMinWeight, double orientationGatingThr);

  Pose2D findBestWeight();
  Pose2D getEstimate();

  string logIP = "127.0.0.1:9876";
};

class Brain;
using namespace BT;

void RegisterLocatorNodes(BT::BehaviorTreeFactory &factory, Brain *brain);

class SelfLocateEnterField : public SyncActionNode {
public:
  SelfLocateEnterField(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

  NodeStatus tick() override;

  static PortsList providedPorts() {
    return {
        InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
    };
  };

private:
  Brain *brain;
};