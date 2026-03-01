#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using namespace std;

class HungarianAlgorithm {
public:
  HungarianAlgorithm() {}
  ~HungarianAlgorithm() {}

  void Solve(const vector<double> &DistMatrix, int nRows, int nMap, vector<int> &Assignment) {
    if (nRows == 0 || nMap == 0) return;

    Assignment.assign(nRows, -1);
    int n = max(nRows, nMap);

    int sizeNeeded = n + 1;
    if (u.size() < sizeNeeded) {
      u.resize(sizeNeeded);
      v.resize(sizeNeeded);
      p.resize(sizeNeeded);
      way.resize(sizeNeeded);
      minv.resize(sizeNeeded);
      used.resize(sizeNeeded);
    }

    return SolveInternal(DistMatrix, n, nRows, nMap, Assignment);
  }

private:
  void SolveInternal(const vector<double> &DistMatrix, int n, int nRows, int nMap, vector<int> &Assignment) {
    // double cost = 0.0;

    fill(u.begin(), u.end(), 0.0);
    fill(v.begin(), v.end(), 0.0);
    fill(p.begin(), p.end(), 0);
    fill(way.begin(), way.end(), 0);

    for (int i = 1; i <= n; ++i) {
      p[0] = i;
      int j0 = 0;
      // Reset minv and used for this iteration step
      // We only strictly need to reset up to n
      fill(minv.begin(), minv.begin() + n + 1, std::numeric_limits<double>::infinity());
      fill(used.begin(), used.begin() + n + 1, false);

      do {
        used[j0] = true;
        int i0 = p[j0], j1 = 0;
        double delta = std::numeric_limits<double>::infinity();

        for (int j = 1; j <= n; ++j) {
          if (!used[j]) {
            double curCost = 0.0;
            // Map 1-based i0, j to 0-based index and check bounds
            if ((i0 - 1) < nRows && (j - 1) < nMap) { curCost = DistMatrix[(i0 - 1) * nMap + (j - 1)]; }

            double cur = curCost - u[i0] - v[j];
            if (cur < minv[j]) {
              minv[j] = cur;
              way[j] = j0;
            }
            if (minv[j] < delta) {
              delta = minv[j];
              j1 = j;
            }
          }
        }

        for (int j = 0; j <= n; ++j) {
          if (used[j]) {
            u[p[j]] += delta;
            v[j] -= delta;
          } else {
            minv[j] -= delta;
          }
        }
        j0 = j1;
      } while (p[j0] != 0);

      do {
        int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0 != 0);
    }

    for (int j = 1; j <= n; ++j) {
      if (p[j] != 0) {
        int row = p[j] - 1;
        int col = j - 1;
        if (row < nRows && col < nMap) { Assignment[row] = col; }
      }
    }

    return;
  }

private:
  // Persistent buffers to avoid allocation
  vector<double> u, v, minv;
  vector<int> p, way;
  vector<uint8_t> used;
};