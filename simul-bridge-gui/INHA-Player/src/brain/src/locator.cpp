#include "locator.h"
#include "brain.h"
#include "brain_tree.h"
#include "utils/hungarian.h"
#include "utils/math.h"
#include "utils/misc.h"
#include "utils/print.h"
#include <map>
#include <random>
#include <set>

#define REGISTER_LOCATOR_BUILDER(Name)                                                                                                                         \
  factory.registerBuilder<Name>(#Name, [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

std::mt19937 &getRandomEngine() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  return gen;
}

double gaussianRandom(double mean, double stddev) {
  static std::normal_distribution<double> N01(0.0, 1.0);
  return mean + stddev * N01(getRandomEngine());
}

double uniformRandom(double min, double max) {
  static std::uniform_real_distribution<double> U01(0.0, 1.0);
  return min + (max - min) * U01(getRandomEngine());
}

constexpr double BIG = 1e12;

void RegisterLocatorNodes(BT::BehaviorTreeFactory &factory, Brain *brain) { REGISTER_LOCATOR_BUILDER(SelfLocateEnterField); }

void Locator::init(FieldDimensions fd, bool enableLogParam, string logIPParam) {
  fieldDimensions = fd;
  calcFieldMarkers(fd);
  enableLog = enableLogParam;
  logIP = logIPParam;
  // if (enableLog) {
  // logger = &log;
  // auto connectError = log.connect(logIP);
  // if (connectError.is_err()) prtErr(format("Rerun log connect Error: %s", connectError.description.c_str()));
  // auto saveError = log.save("/home/booster/log.rrd");
  // if (saveError.is_err()) prtErr(format("Rerun log save Error: %s", saveError.description.c_str()));
  // }
}

void Locator::calcFieldMarkers(FieldDimensions fd) {

  fieldMarkers.push_back(FieldMarker{'X', 0.0, 0.0, 0.0});

  fieldMarkers.push_back(FieldMarker{'G', fd.length / 2, -fd.goalWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'G', fd.length / 2, fd.goalWidth / 2, 0.0});

  fieldMarkers.push_back(FieldMarker{'G', -fd.length / 2, -fd.goalWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'G', -fd.length / 2, fd.goalWidth / 2, 0.0});

  fieldMarkers.push_back(FieldMarker{'X', 0.0, -fd.circleRadius, 0.0});
  fieldMarkers.push_back(FieldMarker{'X', 0.0, fd.circleRadius, 0.0});

  fieldMarkers.push_back(FieldMarker{'P', fd.length / 2 - fd.penaltyDist, 0.0, 0.0});
  fieldMarkers.push_back(FieldMarker{'X', -fd.length / 2 + fd.penaltyDist, 0.0, 0.0});

  fieldMarkers.push_back(FieldMarker{'T', 0.0, fd.width / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', 0.0, -fd.width / 2, 0.0});

  fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, -fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, -fd.penaltyAreaWidth / 2, 0.0});

  fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, -fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, -fd.goalAreaWidth / 2, 0.0});

  fieldMarkers.push_back(FieldMarker{'L', fd.length / 2, fd.width / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', fd.length / 2, -fd.width / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -fd.length / 2, fd.width / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -fd.length / 2, -fd.width / 2, 0.0});

  mapByType.clear();
  for (auto &m : fieldMarkers) {
    mapByType[m.type].push_back(m);
  }
}

void Locator::setParams(int numParticles, double initMargin, std::vector<double> alphas, double smoothAlpha, double invNormVar, double invPerpVar,
                        double likelihoodWeight, double unmatchedPenaltyConfThr, double essThreshold, double clusterDistThr, double clusterThetaThr,
                        double clusterMinWeight, double orientationGatingThr) {
  this->numParticles = numParticles;
  this->initFieldMargin = initMargin;
  this->alpha1 = alphas[0];
  this->alpha2 = alphas[1];
  this->alpha3 = alphas[2];
  this->alpha4 = alphas[3];
  this->smoothAlpha = smoothAlpha;
  this->invNormVar = invNormVar;
  this->invPerpVar = invPerpVar;
  this->likelihoodWeight = likelihoodWeight;
  this->unmatchedPenaltyConfThr = unmatchedPenaltyConfThr;
  this->essThreshold = essThreshold;
  this->clusterDistThr = clusterDistThr;
  this->clusterThetaThr = clusterThetaThr;
  this->clusterMinWeight = clusterMinWeight;
  this->orientationGatingThr = orientationGatingThr;
}

void Locator::globalInit(Pose2D currentOdom) {
  prtWarn(format("[PF][globalInit] | initialized=%d | pfN=%zu | odom=(%.2f %.2f %.2f)", isInitialized, particles.size(), currentOdom.x, currentOdom.y,
                 rad2deg(currentOdom.theta)));
  double xMin = -fieldDimensions.length / 2.0 - initFieldMargin;
  double xMax = 0;
  double yMin = fieldDimensions.width / 2.0 - initFieldMargin;
  double yMax = fieldDimensions.width / 2.0 + initFieldMargin;

  isInitialized = true;
  lastOdom = currentOdom;

  int num = numParticles;
  particles.resize(num);
  double thetaSpread = deg2rad(30.0);

  for (int i = 0; i < num / 2; i++) {
    particles[i].x = uniformRandom(xMin, xMax);
    particles[i].y = uniformRandom(yMin, yMax);
    double thetaCenter = -M_PI / 2.0;
    particles[i].theta = toPInPI(thetaCenter + uniformRandom(-thetaSpread, thetaSpread));
    particles[i].weight = 1.0 / num;
  }
  for (int i = num / 2; i < num; i++) {
    particles[i].x = uniformRandom(xMin, xMax);
    particles[i].y = -uniformRandom(yMin, yMax);
    double thetaCenter = M_PI / 2.0;
    particles[i].theta = toPInPI(thetaCenter + uniformRandom(-thetaSpread, thetaSpread));
    particles[i].weight = 1.0 / num;
  }
}

void Locator::predict(Pose2D currentOdom) {
  if (!isInitialized) {
    lastOdom = currentOdom;
    // BT에 섞으면 selfLocateEnterField에서만 globalInit()
    globalInit(currentOdom);
    return;
  }

  double dx = currentOdom.x - lastOdom.x;
  double dy = currentOdom.y - lastOdom.y;
  double dtheta = toPInPI(currentOdom.theta - lastOdom.theta);

  // 변화가 너--무 작으면 조기종료
  if (fabs(dx) < 0.0001 && fabs(dy) < 0.0001 && fabs(dtheta) < 0.0087) {
    lastOdom = currentOdom;
    return;
  }

  double c = cos(lastOdom.theta);
  double s = sin(lastOdom.theta);

  double trans_x = c * dx + s * dy; // 로봇좌표계로
  double trans_y = -s * dx + c * dy;

  double rot1 = atan2(trans_y, trans_x);
  double trans = sqrt(trans_x * trans_x + trans_y * trans_y);
  double rot2 = (dtheta - rot1);

  double rot1_dev = alpha1 * fabs(rot1) + alpha2 * trans;
  double trans_dev = alpha3 * trans + alpha4 * (fabs(rot1) + fabs(rot2));
  double rot2_dev = alpha1 * fabs(rot2) + alpha2 * trans;

  for (auto &p : particles) {
    double n_rot1 = rot1 + (gaussianRandom(0, rot1_dev));
    double n_trans = trans + (gaussianRandom(0, trans_dev));
    double n_rot2 = rot2 + (gaussianRandom(0, rot2_dev));

    p.x += n_trans * cos(p.theta + n_rot1);
    p.y += n_trans * sin(p.theta + n_rot1);
    p.theta = toPInPI(p.theta + n_rot1 + n_rot2);
  }

  lastOdom = currentOdom;
}

void Locator::correct(const vector<FieldMarker> markers) {

  double totalWeight = 0;

  const double xMinConstraint = -fieldDimensions.length / 2.0 - initFieldMargin;
  const double xMaxConstraint = fieldDimensions.length / 2.0 + initFieldMargin;
  const double yMinConstraint = -fieldDimensions.width / 2.0 - initFieldMargin;
  const double yMaxConstraint = fieldDimensions.width / 2.0 + initFieldMargin;
  const int nMap = fieldMarkers.size();
  const int nObs = markers.size();
  obsInField.reserve(nObs);

  for (auto &p : particles) {
    double px = p.x;
    double py = p.y;
    double pt = p.theta;
    double c = cos(pt);
    double s = sin(pt);

    if (px < xMinConstraint || px > xMaxConstraint || py < yMinConstraint || py > yMaxConstraint) {
      p.weight = 0.0;
    } else {
      if (hasPose) {
        // orientation이 현재 final pose보다 +-120도 초과 시 skip
        if (fabs(toPInPI(pt - pose.theta)) > orientationGatingThr) continue;
      }

      // detected landmark를 field 좌표계로 변환
      obsInField.clear();
      for (const auto &m_r : markers) {
        double ox = c * m_r.x - s * m_r.y + px;
        double oy = s * m_r.x + c * m_r.y + py;
        obsInField.push_back(FieldMarker{m_r.type, ox, oy, m_r.confidence});
      }

      // assignment problem
      if (flatCostMatrix.size() < nObs * nMap) flatCostMatrix.resize(nObs * nMap);
      std::fill_n(flatCostMatrix.begin(), nObs * nMap, BIG);

      for (int i = 0; i < nObs; ++i) {
        char obsType = obsInField[i].type;
        for (int j = 0; j < nMap; ++j) {
          // Type Mismatch Check
          if (fieldMarkers[j].type != obsType) continue;

          double dx = obsInField[i].x - fieldMarkers[j].x;
          double dy = obsInField[i].y - fieldMarkers[j].y;
          flatCostMatrix[i * nMap + j] = dx * dx + dy * dy;
        }
      }
      assignment.clear();
      assignment.reserve(max(nObs, nMap));
      hungarian.Solve(flatCostMatrix, nObs, nMap, assignment);
      auto calCost = [&](int i, int j) -> double {
        // marker_f, marker_r : observed markers on respective frame
        // map_marker : true info

        // d = marker_f - map_marker
        // R = [[(map_marker - robotPositionOnField) / norm(map_marker - robotPositionOnField)]^T,[(map_marker - ropotPositionOnField) / norm(map_marker -
        // ropotPositionOnField)]^-1^T]
        // sigma = [[k1 * (map_marker - robotPositionOnField)^2, 0],
        //          [0, k2 * (map_marker - robotPositionOnField)^2]]
        // D^2 = d^T * R * sigma^-1 * R^T * d

        double dx = obsInField[i].x - fieldMarkers[j].x;
        double dy = obsInField[i].y - fieldMarkers[j].y;

        double vx = fieldMarkers[j].x - px;
        double vy = fieldMarkers[j].y - py;
        double dist = max(1e-6, sqrt(vx * vx + vy * vy));
        double ux = vx / dist;
        double uy = vy / dist;

        double dn = ux * dx + uy * dy;
        double dp = -uy * dx + ux * dy;

        double sigma_n = invNormVar * dist + 0.05;
        double sigma_p = invPerpVar * dist + 0.05;

        double D2 = dn * dn / (sigma_n * sigma_n) + dp * dp / (sigma_p * sigma_p);
        return D2;
      };
      // likelihood update
      double sumCost = 0.0;
      for (int i = 0; i < nObs; ++i) {
        int j = assignment[i];
        sumCost += calCost(i, j);
      }

      // 마커 개수로 normalize해야하나?
      double logLikelihood = -0.5 * sumCost;

      double likelihood = exp(logLikelihood * likelihoodWeight);
      p.weight *= likelihood;
    }
    totalWeight += p.weight;
  }
  // Normalization
  if (totalWeight < 1e-10) {
    for (auto &p : particles)
      p.weight = 1.0 / particles.size();
    totalWeight = 1.0;
  } else {
    for (auto &p : particles)
      p.weight /= totalWeight;
  }

  // ess calculation
  double sqSum = 0;
  for (auto &p : particles)
    sqSum += p.weight * p.weight;
  double ess = 1.0 / (sqSum + 1e-9);

  // ESS resampling
  if (ess < particles.size() * essThreshold) {
    vector<Particle> newParticles;
    newParticles.reserve(numParticles);

    // CDF construction
    cdf.resize(particles.size());
    cdf[0] = particles[0].weight;
    for (size_t i = 1; i < particles.size(); ++i) {
      cdf[i] = cdf[i - 1] + particles[i].weight;
    }
    // Ensure last is 1.0 (handle float errors)
    cdf.back() = 1.0;

    // Helper to pick from CDF
    auto sampleCDF = [&]() -> const Particle & {
      double r = uniformRandom(0.0, 1.0);
      auto it = std::upper_bound(cdf.begin(), cdf.end(), r);
      int idx = std::distance(cdf.begin(), it);
      if (idx >= particles.size()) idx = particles.size() - 1;
      return particles[idx];
    };

    // Resample
    for (int i = 0; i < numParticles; ++i) {
      Particle pNew;
      pNew = sampleCDF();
      pNew.weight = 1.0 / numParticles;
      newParticles.push_back(pNew);
    }
    particles = newParticles;
  }
}

Pose2D Locator::getEstimate() {
  if (!hasPose) {
    // 첫빠따에는 그냥 weight 가장 큰 애 찾음
    hasPose = true;
    return findBestWeight();
  }
  // 이전 포즈가 존재해서 일루 들어감
  clusterParticles();

  // 클러스터링해서 찾은 current best pose와 이전 포즈를 스무딩
  Pose2D raw = bestPose;

  // EMA Smoothing

  double alpha = smoothAlpha;

  pose.x = alpha * raw.x + (1.0 - alpha) * pose.x;
  pose.y = alpha * raw.y + (1.0 - alpha) * pose.y;

  double dTheta = toPInPI(raw.theta - pose.theta);
  pose.theta = toPInPI(pose.theta + alpha * dTheta);

  return pose;
}

Pose2D Locator::findBestWeight() {
  int bestIdx = -1;
  double maxWeight = 0.0;
  for (int i = 0; i < particles.size(); ++i) {
    if (particles[i].weight > maxWeight) {
      bestIdx = i;
      maxWeight = particles[i].weight;
    }
  }
  return {particles[bestIdx].x, particles[bestIdx].y, particles[bestIdx].theta};
}

void Locator::clusterParticles() {
  // 기존 pose를 reference pose로 선정
  Pose2D ref_p = pose;
  // 주변 particle 찾기
  vector<int> inlierIdx;
  double wSum = 0.0;
  for (int i = 0; i < particles.size(); ++i) {
    double d_xy_sq = (particles[i].x - ref_p.x) * (particles[i].x - ref_p.x) + (particles[i].y - ref_p.y) * (particles[i].y - ref_p.y);
    double d_th_ab = abs(toPInPI(particles[i].theta - ref_p.theta));
    if (d_xy_sq <= clusterDistThr && d_th_ab <= clusterThetaThr) {
      inlierIdx.push_back(i);
      wSum += particles[i].weight;
    }
  }
  // 클러스터 형성이 충분치 않으면
  if (wSum < clusterMinWeight) {
    // weight 가장 높은 애를 포즈로 업데이트
    bestPose = findBestWeight();
    return;
  } else { // 클러스터링이 됐다면 가중평균을 bestpose로 업데이트
    double xSum = 0;
    double ySum = 0;
    double sinSum = 0;
    double cosSum = 0;
    for (auto &i : inlierIdx) {
      double w = particles[i].weight;
      xSum += w * particles[i].x;
      ySum += w * particles[i].y;
      sinSum += w * sin(particles[i].theta);
      cosSum += w * cos(particles[i].theta);
    }
    bestPose.x = xSum / wSum;
    bestPose.y = ySum / wSum;
    double sinAvg = sinSum / wSum;
    double cosAvg = cosSum / wSum;
    bestPose.theta = atan2(sinAvg, cosAvg);
  }
  return;
}

void Locator::setLog(rerun::RecordingStream *stream) { logger = stream; }

void Locator::logParticles(double time_sec) {
  if (!enableLog || logger == nullptr) return;

  const size_t N = particles.size();

  std::vector<rerun::Position2D> origins;
  std::vector<rerun::Vector2D> vectors;
  std::vector<rerun::Color> colors;
  std::vector<float> radii;

  origins.reserve(N);
  vectors.reserve(N);
  colors.reserve(N);
  radii.reserve(N);

  const float len = 0.1f;

  for (const auto &p : particles) {
    float x0 = static_cast<float>(p.x);
    float y0 = static_cast<float>(p.y);
    float dx = len * std::cos(p.theta);
    float dy = len * std::sin(p.theta);

    origins.push_back({x0, -y0}); // Flip Y for display
    vectors.push_back({dx, -dy}); // Flip Y for display

    // Dynamic Alpha: Base 50, scales up with weight
    // multiplier 1000 ensures that even small weights get some boost, but max out at 200
    uint8_t alpha = static_cast<uint8_t>(std::clamp(50.0 + p.weight * 2000.0, 50.0, 200.0));
    colors.push_back(rerun::Color{0, 255, 255, alpha});

    // Base radius 0.005, max radius 0.05
    float r = 0.005f + 0.045f * (float)(p.weight);
    radii.push_back(r);
  }

  logger->log("field/particles", rerun::Arrows2D::from_vectors(vectors).with_origins(origins).with_colors(colors).with_radii(radii).with_draw_order(19.0));
}

// 나중에 모드를 설정해서 initial particle 영역을 달리해야댐
NodeStatus SelfLocateEnterField::tick() {
  if (!brain->locator->getIsInitialized()) { brain->locator->globalInit(brain->data->robotPoseToOdom); }
  return NodeStatus::SUCCESS;
}