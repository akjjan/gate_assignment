#pragma once
#include "gurobi_c++.h"
#include <cstddef>
#include <utility>

enum FlightType { ARRIVAL, DEPARTURE }; // 枚举类型表示航班类型

enum AircraftSize { MEDIUM, LARGE }; // 枚举类型表示飞机大小

struct Flight {
  int flight_id;              // 航班ID
  FlightType flight_type;     // 到达/离开类型
  AircraftSize aircraft_size; // 飞机大小
  int scheduled_time;         // 计划时间
};

inline bool operator<(const Flight &a, const Flight &b) {
  return a.scheduled_time < b.scheduled_time;
}

struct yKey {
  int i, j, k;
  bool operator==(const yKey &other) const {
    return i == other.i && j == other.j && k == other.k;
  }
};

struct yKeyHash { // 此哈希函数适用于i,j,k均小于16384的情况
  std::size_t operator()(const yKey &key) const {
    return (std::size_t(key.i) << 28) | (std::size_t(key.j) << 14) |
           std::size_t(key.k);
  }
};

struct zKey {
  int i, delta_i, u, v;
  bool operator==(const zKey &other) const {
    return i == other.i && delta_i == other.delta_i && u == other.u &&
           v == other.v;
  }
};

struct zKeyHash { // 此哈希函数适用于i,delta_i,u,v均小于16384的情况
  std::size_t operator()(const zKey &key) const {
    return (std::size_t(key.i) << 42) | (std::size_t(key.delta_i) << 28) |
           (std::size_t(key.u) << 14) | std::size_t(key.v);
  }
};

struct Row {
  GRBConstr constr;
  double constant = 0.0;
  double x_coeff = 0.0;
  double y_coeff = 0.0;
  double z_coeff = 0.0;
  std::pair<int, int> x_var = {-1, -1}; // x[i][k] 的索引
  yKey y_var = {-1, -1, -1};            // y[i][j][k] 的索引
  zKey z_var = {-1, -1, -1, -1};        // z[i][delta_i][u][v] 的索引
};