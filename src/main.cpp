#include "gurobi_c++.h"

#include "case_data.hpp"
#include "modelDef.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define GREEN "\033[32m"
#define RESET "\033[0m"

namespace fs = std::filesystem;

using std::map;
using std::to_string;
using std::unordered_map;
using std::unordered_set;
using std::vector;

caseData makeSampleCaseData() {
  int flightNumber = 100;      // 航班数量
  int gateNumber = 10;         // 登机口数量
  int apronIndex = gateNumber; // 明确停机坪索引
  int bufferTime = 30;         // 缓冲时间
  vector<double> apronPenaltyCost(flightNumber,
                                  100.0); // 每个航班的停机坪惩罚成本
  vector<double> delayPenaltyCost(flightNumber,
                                  100.0); // 每个航班的延误惩罚成本
  vector<Flight> flights(flightNumber);   // 航班信息列表
  vector<int> noDepartArr;   // \underline{F_a}  不离开的到达航班索引
  vector<int> haveDepartArr; // \overline{F_a}   有离开的到达航班索引
  vector<int> departFlights; // 离开航班索引  F_d
  map<int, int> delta;       //  \delta 映射 ： 到达航班ID -> 离开航班ID
  vector<int> mediumGates;   // 中等大小登机口集合
  vector<int> largeFlights;  // 大型飞机航班集合
  vector<vector<double>> towCost(
      gateNumber + 1, vector<double>(gateNumber + 1, 50.0)); // 拖行成本
  vector<vector<double>> towTime(
      gateNumber + 1, vector<double>(gateNumber + 1, 30.0)); // 拖行时间

  unordered_map<int, int> departure_pert;

  return makeCaseData(flightNumber, gateNumber, bufferTime, apronPenaltyCost,
                      delayPenaltyCost, flights, noDepartArr, haveDepartArr,
                      departFlights, delta, mediumGates, largeFlights, towCost,
                      towTime, departure_pert);
}

map<int, vector<int>> get_flights_in_gate_map(const vector<vector<GRBVar>> &x) {
  map<int, vector<int>> flights_in_gate;
  int FlightNumber = x.size();
  int GateNumber = x[0].size() - 1;
  for (int i = 0; i < FlightNumber; ++i) {
    for (int j = 0; j <= GateNumber; ++j) {
      if (x[i][j].get(GRB_DoubleAttr_X) > 0.5) {
        flights_in_gate[j].push_back(i);
      }
    }
  }

  return flights_in_gate;
}

using sparseSuccession = unordered_set<yKey, yKeyHash>;
// y_i_j_k，连续执行关系的类型别名，稀疏存储

sparseSuccession determine_Y(const vector<vector<GRBVar>> &x,
                             const vector<Flight> &flights) {
  // 根据 x 计算 Y
  sparseSuccession Y;
  // {i，j，k} in Y 表示 航班 i 和 j 在登机口 k 连续执行

  auto flights_in_gate = get_flights_in_gate_map(x);

  for (auto &[gate, indices] : flights_in_gate) {
    // 按计划时间排序
    auto cmp = [&](int a, int b) {
      return flights[a].scheduled_time < flights[b].scheduled_time;
    };
    std::sort(indices.begin(), indices.end(), cmp);
    for (size_t idx = 0; idx + 1 < indices.size(); ++idx) {
      int i = indices[idx];
      int j = indices[idx + 1];
      Y.insert({i, j, gate});
    }
  }

  return Y;
}

/*
map<int, int> get_idx_map(const vector<int> &F_a_up)
{
    map<int, int> idx_of;
    for (size_t k = 0; k < F_a_up.size(); ++k)
    {
        idx_of[F_a_up[k]] = k;
    }
    return idx_of;
}
*/

using sparseZ = unordered_set<zKey, zKeyHash>;
// z_i_delta_i_u_v，稀疏存储

double calculateTowCost(const GRBEnv &env, const caseData &d,
                        const vector<vector<GRBVar>> &x,
                        const sparseSuccession &Y, sparseZ &Z) {

  auto &F_a_up = d.haveDepartArr; // 有离开的到达航班索引
                                  // 起个别名方便书写

  double tow_cost = 0.0;

  int GateNumber = d.gateNumber;

  for (auto i : F_a_up) {
    auto delta_i = d.delta.at(i); // 到港航班 i 对应的离开航班ID
    for (int u = 0; u <= GateNumber; ++u) {
      for (int v = 0; v <= GateNumber; ++v) {
        if (u == v) {
          if (x[i][u].get(GRB_DoubleAttr_X) > 0.5 and
              x[delta_i][u].get(GRB_DoubleAttr_X) > 0.5 and
              Y.count({i, delta_i, u}) == 0) {
            tow_cost += d.towCost[u][u]; // 同一登机口拖行成本
            Z.insert({i, delta_i, u, v});
          }
        } else {
          if (x[i][u].get(GRB_DoubleAttr_X) > 0.5 and
              x[delta_i][v].get(GRB_DoubleAttr_X) > 0.5) {
            tow_cost += d.towCost[u][v]; // 不同登机口拖行成本
            Z.insert({i, delta_i, u, v});
          }
        }
      }
    }
  }

  return tow_cost;
}

double calculateDelayCost(const GRBEnv &env, const caseData &d,
                          const vector<vector<GRBVar>> &x,
                          const sparseSuccession &Y, const sparseZ &Z) {
  // 计算延误成本
  auto sub_m = GRBModel(env);
  sub_m.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // 最小化

  int flightNumber = x.size();

  vector<GRBVar> decisionTime(flightNumber);
  vector<GRBVar> arrivalDelay(flightNumber);

  double delay_cost = 0.0;

  for (int i = 0; i < flightNumber; ++i) {
    if (d.flightMap.at(i).flight_type == DEPARTURE) {
      decisionTime[i] = sub_m.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS);
      arrivalDelay[i] = sub_m.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
      sub_m.addConstr(decisionTime[i] >= d.flightMap.at(i).scheduled_time);
      delay_cost -= d.flightMap.at(i).scheduled_time;
    } else {
      decisionTime[i] = sub_m.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
      arrivalDelay[i] = sub_m.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS);
      sub_m.addConstr(decisionTime[i] >= d.flightMap.at(i).scheduled_time);
      sub_m.addConstr(arrivalDelay[i] >=
                      decisionTime[i] - d.flightMap.at(i).scheduled_time);
    }
  }

  for (auto &[i, delta_i, u, v] : Z) {
    sub_m.addConstr(decisionTime[delta_i] >=
                    decisionTime[i] + d.bufferTime + d.towTime[u][v]);
  }

  for (auto &[i, j, k] : Y) {
    sub_m.addConstr(decisionTime[j] >= decisionTime[i] + d.bufferTime);
  }

  sub_m.optimize();

  delay_cost += sub_m.get(GRB_DoubleAttr_ObjVal);

  return delay_cost;
}

class Solver {
public:
  Solver(const caseData &standard)
      : d_(standard), env_(createEnv("gate_assignment.log")), master_(env_),
        sub_env_(createEnv("gate_assignment_sub.log")) {}

  void build_master_problem(int senarios_num) {
    int flight_number = d_.flightNumber;
    int gate_number = d_.gateNumber;
    const auto &no_depart_arr = d_.noDepartArr;

    x_.resize(flight_number, vector<GRBVar>(gate_number + 1));
    // 决策变量 x[i][j]，表示航班 i 是否分配到登机口 j
    // ,x[i][d.apronIndex]表示航班 i 分配停机坪
    for (int i = 0; i < flight_number; ++i) {
      for (int k = 0; k <= gate_number; ++k) {
        auto varName = "x_" + to_string(i) + "_" + to_string(k);
        // 分配到停机坪的惩罚成本
        x_[i][k] = master_.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
      }
    }

    // 决策变量 gate_used_[k]，表示登机口 k 是否被使用
    gate_used_.resize(gate_number);
    for (int k = 0; k < gate_number; ++k) {
      auto varName = "gate_used_" + to_string(k);
      gate_used_[k] = master_.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
    }

    //---------------------------
    // 决策变量  y[i][j][k]，表示航班 i 和 j 在登机口 k 连续执行, k不包括停机坪
    for (int i = 0; i < flight_number; ++i) {
      for (int j = 0; j < flight_number; ++j) {
        for (int k = 0; k < gate_number; ++k) {
          auto varName =
              "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k);
          y_[{i, j, k}] = master_.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
        }
      }
    }

    //---------------------------- z_i_j_u_v
    // 决策变量 z[i][j][u][v]，表示关联航班 i 和 j 从 登机口 u 拖行到 v，uv
    // 包括停机坪
    for (int i : d_.haveDepartArr) {
      int j = d_.delta.at(i); // 到港航班 i 对应的离开航班ID

      for (int u = 0; u <= gate_number; ++u) {
        for (int v = 0; v <= gate_number; ++v) {
          auto varName = "z_" + to_string(i) + "_" + to_string(j) + "_" +
                         to_string(u) + "_" + to_string(v);

          z_[{i, j, u, v}] = master_.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
        }
      }
    }

    // 非线性部分
    // eta_ = master_.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "eta");
    eta_.resize(senarios_num);
    for (int s = 0; s < senarios_num; ++s) {
      eta_[s] = master_.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                               "eta_" + to_string(s));
    }

    // 主问题约束------------------------

    // 提前筛除明显错误的分配y
    for (int i = 0; i < flight_number; ++i) {
      for (int j = 0; j < flight_number; ++j) {
        for (int k = 0; k < gate_number; ++k) {
          if (d_.flightMap.at(i).scheduled_time >=
              d_.flightMap.at(j).scheduled_time) {
            master_.addConstr(y_[{i, j, k}] == 0,
                              "TimeOrderConstr_" + to_string(i) + "_" +
                                  to_string(j) + "_" + to_string(k));
          }
        }
      }
    }

    // 每个航班分配一个登机口或停机坪
    for (int i = 0; i < flight_number; ++i) {
      GRBLinExpr expr = 0;
      for (int k = 0; k <= gate_number; ++k) {
        expr += x_[i][k];
      }
      master_.addConstr(expr == 1, "AssignConstr_" + to_string(i));
    }

    // 大型飞机不能分配到中等大小登机口
    for (int i : d_.largeFlights) {
      for (int k : d_.mediumGates) {
        master_.addConstr(x_[i][k] == 0, "Flight-GateConstr_" + to_string(i) +
                                             "_" + to_string(k));
      }
    }

    // 禁止自环
    for (int k = 0; k < gate_number; ++k) {
      for (int i = 0; i < flight_number; ++i) {
        master_.addConstr(y_[{i, i, k}] == 0);
      }
    }

    // 原始版本消除子环约束
    for (int k = 0; k < gate_number; ++k) {
      for (int i = 0; i < flight_number; ++i) {
        for (int j = 0; j < flight_number; ++j) {
          master_.addConstr(y_[{i, j, k}] + y_[{j, i, k}] <= 1,
                            "NoCycleConstr_" + to_string(i) + "_" +
                                to_string(j) + "_" + to_string(k));
        }
      }
    }

    // 被选中才能出边入边
    for (int i = 0; i < flight_number; ++i) {
      for (int k = 0; k < gate_number; ++k) {
        GRBLinExpr expr1 = 0;
        GRBLinExpr expr2 = 0;
        for (int j = 0; j < flight_number; ++j) {
          expr1 += y_[{i, j, k}];
          expr2 += y_[{j, i, k}];
        }
        master_.addConstr(expr1 <= x_[i][k],
                          "Y_X_Constr1_" + to_string(i) + "_" + to_string(k));
        master_.addConstr(expr2 <= x_[i][k],
                          "Y_X_Constr2_" + to_string(i) + "_" + to_string(k));

        master_.addConstr(expr1 <= 1, "Y_OneOutConstr_" + to_string(i) + "_" +
                                          to_string(k));
        master_.addConstr(expr2 <= 1,
                          "Y_OneInConstr_" + to_string(i) + "_" + to_string(k));
      }
    }

    for (int i = 0; i < flight_number; ++i) {
      for (int j = 0; j < flight_number; ++j) {
        for (int k = 0; k < gate_number; ++k) {
          master_.addConstr(y_[{i, j, k}] <= x_[i][k],
                            "Y_X_Constr3_" + to_string(i) + "_" + to_string(j) +
                                "_" + to_string(k));
          master_.addConstr(y_[{i, j, k}] <= x_[j][k],
                            "Y_X_Constr4_" + to_string(i) + "_" + to_string(j) +
                                "_" + to_string(k));
        }
      }
    }

    for (int k = 0; k < gate_number; ++k) {
      GRBLinExpr rhs = 0;
      for (int i = 0; i < flight_number; ++i) {
        rhs += x_[i][k];
      }
      master_.addConstr(gate_used_[k] <= rhs, "GateUsedConstr_" + to_string(k));
      master_.addConstr(gate_used_[k] * flight_number >= rhs,
                        "GateUsedConstr2_" + to_string(k));
    }

    // 边数=点数-1
    for (int k = 0; k < gate_number; ++k) {
      GRBLinExpr lhs = 0;
      GRBLinExpr rhs = 0;
      for (int i = 0; i < flight_number; ++i) {
        rhs += x_[i][k];
        for (int j = 0; j < flight_number; ++j) {
          lhs += y_[{i, j, k}];
        }
      }
      master_.addConstr(lhs == rhs - 1, "Y_ConsistencyConstr_" + to_string(k));
    }

    // 拖行约束
    for (int i : d_.haveDepartArr) {
      int delta_i = d_.delta.at(i);
      for (int u = 0; u <= gate_number; ++u) {
        for (int v = 0; v <= gate_number; ++v) {
          master_.addConstr(
              z_[{i, delta_i, u, v}] >= x_[i][u] + x_[delta_i][v] - 1,
              "TowConstr1_" + to_string(i) + "_" + to_string(delta_i) + "_" +
                  to_string(u) + "_" + to_string(v));
          master_.addConstr(z_[{i, delta_i, u, v}] <= x_[i][u],
                            "TowConstr2_" + to_string(i) + "_" +
                                to_string(delta_i));
          master_.addConstr(z_[{i, delta_i, u, v}] <= x_[delta_i][v],
                            "TowConstr3_" + to_string(i) + "_" +
                                to_string(delta_i) + "_" + to_string(v));
        }
      }
    }

    // 影子约束
    for (int i : d_.largeFlights) {
      for (int j : d_.largeFlights) {
        if (j != i) {
          for (int k = 0; k < gate_number; ++k) {
            master_.addConstr(x_[i][k] + x_[j][(k + 1) % gate_number] <= 1,
                              "ShadowConstr_" + to_string(i) + "_" +
                                  to_string(j) + "_" + to_string(k));
          }
        }
      }
    }

    GRBLinExpr obj_func = 0.0;

    for (int i = 0; i < flight_number; ++i) {
      obj_func += d_.apronPenaltyCost[i] * x_[i][d_.apronIndex];
    }

    for (int i : d_.haveDepartArr) {
      int delta_i = d_.delta.at(i);
      for (int u = 0; u <= gate_number; ++u) {
        for (int v = 0; v <= gate_number; ++v) {
          obj_func += d_.towCost[u][v] * z_[{i, delta_i, u, v}];
        }
      }
    }

    for (int i : d_.noDepartArr) {
      for (int k = 0; k < gate_number; ++k) {
        obj_func += d_.towCost[k][d_.apronIndex] * x_[i][k];
      }
    }

    // obj_func += eta_;
    double prob_weight = 1.0 / senarios_num;
    for (int s = 0; s < senarios_num; ++s) {
      obj_func += prob_weight * eta_[s];
    }

    master_.setObjective(obj_func);
  }

  std::pair<std::unique_ptr<GRBModel>, vector<Row>>
  build_single_senario_subproblem(
      caseData &data, unordered_map<yKey, double, yKeyHash> &y_val,
      unordered_map<zKey, double, zKeyHash> &z_val) {

    // === 添加调试输出 ===
    std::cout << GREEN;
    std::cout << "=== build_single_senario_subproblem ===" << std::endl;
    std::cout << "d_.flightNumber = " << d_.flightNumber << std::endl;
    std::cout << "d_.departFlights.size() = " << d_.departFlights.size()
              << std::endl;
    std::cout << "d_.noDepartArr.size() = " << d_.noDepartArr.size()
              << std::endl;
    std::cout << "d_.haveDepartArr.size() = " << d_.haveDepartArr.size()
              << std::endl;
    std::cout << "data.departure_pert.size() = " << data.departure_pert.size()
              << std::endl;
    std::cout << "y_val.size() = " << y_val.size() << std::endl;
    std::cout << "z_val.size() = " << z_val.size() << std::endl;
    if (!d_.departFlights.empty()) {
      std::cout << "d_.departFlights[0] = " << d_.departFlights[0] << std::endl;
    }
    std::cout << RESET; // 重置颜色
    // ====================

    auto sub_ptr = std::make_unique<GRBModel>(sub_env_);
    GRBModel &sub = *sub_ptr; // 获取引用以便后续使用

    // GRBModel sub(sub_env_);

    sub.set(GRB_IntParam_InfUnbdInfo, 1);
    sub.set(GRB_IntParam_DualReductions, 0);

    int flight_number = d_.flightNumber;
    auto departure_pert = data.departure_pert;

    unordered_map<int, GRBVar> departure_time;
    unordered_map<int, GRBVar> arrival_time;
    unordered_map<int, GRBVar> arrival_delay;

    for (int i : d_.departFlights) {
      departure_time[i] =
          sub.addVar(0.0, GRB_INFINITY, d_.delayPenaltyCost[i], GRB_CONTINUOUS);
    }

    for (int i : d_.noDepartArr) {
      arrival_time[i] = sub.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
      arrival_delay[i] =
          sub.addVar(0.0, GRB_INFINITY, d_.delayPenaltyCost[i], GRB_CONTINUOUS);
    }
    for (int i : d_.haveDepartArr) {
      arrival_time[i] = sub.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
      arrival_delay[i] =
          sub.addVar(0.0, GRB_INFINITY, d_.delayPenaltyCost[i], GRB_CONTINUOUS);
    }

    vector<Row> rows; // 记录每个约束的常数项、系数和变量索引

    // 约束3-18
    for (int i : d_.haveDepartArr) {
      int delta_i = d_.delta.at(i);
      for (int u = 0; u <= d_.gateNumber; ++u) {
        for (int v = 0; v <= d_.gateNumber; ++v) {
          GRBLinExpr left_hand_side = departure_time[delta_i] - arrival_time[i];
          GRBLinExpr right_hand_side =
              departure_pert.at(delta_i) +
              d_.towTime[u][v] * z_val[{i, delta_i, u, v}];
          Row row;
          row.constr = sub.addConstr(left_hand_side >= right_hand_side,
                                     "SubTowConstr_" + to_string(i) + "_" +
                                         to_string(delta_i) + "_" +
                                         to_string(u) + "_" + to_string(v));

          row.constant = static_cast<double>(departure_pert.at(delta_i));

          row.z_coeff = d_.towTime[u][v];

          row.z_var = {i, delta_i, u, v};
          rows.push_back(row);
        }
      }
    }

    // 3-19
    for (int i : d_.noDepartArr) {
      for (int j : d_.departFlights) {
        for (int k = 0; k < d_.gateNumber; ++k) {
          GRBLinExpr left_hand_side = departure_time[j] - arrival_time[i];
          GRBLinExpr right_hand_side = BIG_M * (y_val[{i, j, k}] - 1) +
                                       d_.bufferTime + departure_pert.at(j);
          Row row;
          row.constr =
              sub.addConstr(left_hand_side >= right_hand_side,
                            "SubSuccessionConstr_" + to_string(i) + "_" +
                                to_string(j) + "_" + to_string(k));
          row.constant = d_.bufferTime +
                         static_cast<double>(departure_pert.at(j)) -
                         static_cast<double>(BIG_M);

          row.y_coeff = static_cast<double>(BIG_M);

          row.y_var = {i, j, k};

          rows.push_back(row);
        }
      }
    }

    // 3-19
    for (int i : d_.haveDepartArr) {
      int delta_i = d_.delta.at(i);
      for (int j : d_.departFlights) {
        /*
        if (j == delta_i)
          continue;
        */
        for (int k = 0; k < d_.gateNumber; ++k) {
          GRBLinExpr left_hand_side = departure_time[j] - arrival_time[i];
          GRBLinExpr right_hand_side = BIG_M * (y_val[{i, j, k}] - 1) +
                                       d_.bufferTime + departure_pert.at(j);
          Row row;
          row.constr =
              sub.addConstr(left_hand_side >= right_hand_side,
                            "SubSuccessionConstr_" + to_string(i) + "_" +
                                to_string(j) + "_" + to_string(k));
          row.constant = d_.bufferTime +
                         static_cast<double>(departure_pert.at(j)) -
                         static_cast<double>(BIG_M);

          row.y_coeff = static_cast<double>(BIG_M);

          row.y_var = {i, j, k};

          rows.push_back(row);
        }
      }
    }

    // 3-20
    vector<int> arrival_flights;
    arrival_flights.insert(arrival_flights.end(), d_.noDepartArr.begin(),
                           d_.noDepartArr.end());
    arrival_flights.insert(arrival_flights.end(), d_.haveDepartArr.begin(),
                           d_.haveDepartArr.end());

    for (int i : arrival_flights) {
      for (int j : arrival_flights) {
        if (j == i)
          continue;
        for (int k = 0; k < d_.gateNumber; ++k) {
          GRBLinExpr left_hand_side = arrival_time[j] - arrival_time[i];
          GRBLinExpr right_hand_side =
              BIG_M * (y_val[{i, j, k}] - 1) + d_.bufferTime;

          Row row;
          row.constr =
              sub.addConstr(left_hand_side >= right_hand_side,
                            "SubArrivalSuccessionConstr_" + to_string(i) + "_" +
                                to_string(j) + "_" + to_string(k));

          row.constant = d_.bufferTime - static_cast<double>(BIG_M);

          row.y_coeff = static_cast<double>(BIG_M);

          row.y_var = {i, j, k};

          rows.push_back(row);
        }
      }
    }

    // 3-21
    for (int i : d_.departFlights) {
      for (int j : arrival_flights) {
        for (int k = 0; k < d_.gateNumber; ++k) {
          GRBLinExpr left_hand_side = arrival_time[j] - departure_time[i];
          GRBLinExpr right_hand_side =
              BIG_M * (y_val[{i, j, k}] - 1) + d_.bufferTime;

          Row row;
          row.constr =
              sub.addConstr(left_hand_side >= right_hand_side,
                            "SubDepartArrivalSuccessionConstr_" + to_string(i) +
                                "_" + to_string(j) + "_" + to_string(k));

          row.constant = d_.bufferTime - static_cast<double>(BIG_M);

          row.y_coeff = static_cast<double>(BIG_M);

          row.y_var = {i, j, k};

          rows.push_back(row);
        }
      }
    }

    // 3-22
    for (int i : d_.departFlights) {
      for (int j : d_.departFlights) {
        if (j == i)
          continue;
        for (int k = 0; k < d_.gateNumber; ++k) {
          GRBLinExpr left_hand_side = departure_time[j] - departure_time[i];
          GRBLinExpr right_hand_side = BIG_M * (y_val[{i, j, k}] - 1) +
                                       d_.bufferTime + departure_pert.at(j);

          Row row;
          row.constr =
              sub.addConstr(left_hand_side >= right_hand_side,
                            "SubDepartSuccessionConstr_" + to_string(i) + "_" +
                                to_string(j) + "_" + to_string(k));

          row.constant = d_.bufferTime - static_cast<double>(BIG_M) +
                         static_cast<double>(departure_pert.at(j));

          row.y_coeff = static_cast<double>(BIG_M);

          row.y_var = {i, j, k};

          rows.push_back(row);
        }
      }
    }

    // 3-23
    for (int j : d_.departFlights) {
      Row row;
      row.constr =
          sub.addConstr(departure_time[j] >= d_.flightMap.at(j).scheduled_time,
                        "SubScheduledTimeConstr_" + to_string(j));
      row.constant = static_cast<double>(d_.flightMap.at(j).scheduled_time);

      rows.push_back(row);
    }

    // 3-24
    for (int j : arrival_flights) {
      Row row;
      row.constr =
          sub.addConstr(arrival_time[j] >= data.flightMap.at(j).scheduled_time,
                        "SubArrivalScheduledTimeConstr_" + to_string(j));
      row.constant = static_cast<double>(d_.flightMap.at(j).scheduled_time);

      rows.push_back(row);
    }

    // 3-25
    for (int j : arrival_flights) {
      Row row;
      row.constr = sub.addConstr(arrival_delay[j] >=
                                     arrival_time[j] -
                                         data.flightMap.at(j).scheduled_time,
                                 "SubArrivalDelayConstr_" + to_string(j));

      row.constant = -static_cast<double>(d_.flightMap.at(j).scheduled_time);

      rows.push_back(row);
    }

    GRBLinExpr objective = 0.0;

    for (int i : d_.departFlights) {
      objective += d_.delayPenaltyCost[i] *
                   (departure_time[i] - d_.flightMap.at(i).scheduled_time);
    }

    for (int i : arrival_flights) {
      objective += d_.delayPenaltyCost[i] * arrival_delay[i];
    }

    sub.setObjective(objective);

    sub.update();

    // return {std::make_unique<GRBModel>(std::move(sub)), rows};
    return {std::move(sub_ptr), rows};
  }

  std::pair<int, GRBLinExpr> get_cut(GRBModel &sub, const vector<Row> &rows) {

    sub.set(GRB_IntParam_InfUnbdInfo, 1);
    sub.set(GRB_IntParam_DualReductions, 0);
    sub.optimize(); // 求解子问题

    int status = sub.get(GRB_IntAttr_Status);

    if (status == GRB_INFEASIBLE) {
      return {status, get_feasibility_cut(rows)};
    } else if (status == GRB_OPTIMAL) {
      return {status, get_optimality_cut(rows)};
    } else {
      std::cerr << "Subproblem optimization ended with status " << status
                << std::endl;
      return {status, GRBLinExpr(0.0)};
    }
  }

  GRBLinExpr get_feasibility_cut(const vector<Row> &rows) {
    GRBLinExpr lhs = 0.0;
    for (const auto &row : rows) {
      GRBLinExpr expr = 0.0;
      double lambda_i = -row.constr.get(GRB_DoubleAttr_FarkasDual);
      if (fabs(lambda_i) < 1e-6)
        continue;
      expr += row.constant;
      if (row.x_coeff != 0.0) {
        expr += row.x_coeff * x_[row.x_var.first][row.x_var.second];
      }
      if (row.y_coeff != 0.0) {
        expr += row.y_coeff * y_[row.y_var];
      }
      if (row.z_coeff != 0.0) {
        expr += row.z_coeff * z_[row.z_var];
      }
      expr *= lambda_i;
      lhs += expr;
    }
    return lhs;
  }

  GRBLinExpr get_optimality_cut(const vector<Row> &rows) {
    GRBLinExpr rhs = 0.0;

    // === 补上丢失的目标函数常数偏移量 ===
    double obj_constant = 0.0;
    for (int i : d_.departFlights) {
      obj_constant -=
          d_.delayPenaltyCost[i] * d_.flightMap.at(i).scheduled_time;
    }
    rhs += obj_constant; // 加到 RHS 里

    for (const auto &row : rows) {
      GRBLinExpr expr = 0.0;
      double pi_i = row.constr.get(GRB_DoubleAttr_Pi);
      if (fabs(pi_i) < 1e-6)
        continue;
      expr += row.constant;
      if (row.x_coeff != 0.0) {
        expr += row.x_coeff * x_[row.x_var.first][row.x_var.second];
      }
      if (row.y_coeff != 0.0) {
        expr += row.y_coeff * y_[row.y_var];
      }
      if (row.z_coeff != 0.0) {
        expr += row.z_coeff * z_[row.z_var];
      }
      expr *= pi_i;
      rhs += expr;
    }
    // rhs *= weight;
    return rhs;
  }

  void solve_with_callback(vector<caseData> &senarios) {
    master_.set(GRB_IntParam_LazyConstraints, 1);
    BendersCallback cb(*this, senarios);
    master_.setCallback(&cb);
    master_.optimize();
    master_.setCallback(nullptr); // 取消回调
  }

  void final_solution_feasibility_check(caseData &data) {
    // 取最终解
    unordered_map<yKey, double, yKeyHash> y_val;
    unordered_map<zKey, double, zKeyHash> z_val;

    for (auto &[key, var] : y_) {
      y_val[key] = var.get(GRB_DoubleAttr_X);
    }
    for (auto &[key, var] : z_) {
      z_val[key] = var.get(GRB_DoubleAttr_X);
    }

    //  构建子问题
    auto [sub_ptr, rows] = build_single_senario_subproblem(data, y_val, z_val);
    auto &sub = *sub_ptr; // 获取引用以便后续使用

    // 关闭dual reduction（保持一致）
    sub.set(GRB_IntParam_InfUnbdInfo, 1);
    sub.set(GRB_IntParam_DualReductions, 0);

    //  求解
    sub.optimize();

    // 判断结果
    int status = sub.get(GRB_IntAttr_Status);

    if (status == GRB_OPTIMAL) {
      std::cout << GREEN << "Final solution: subproblem FEASIBLE " << RESET
                << std::endl;
      print_solution();
    } else if (status == GRB_INFEASIBLE) {
      std::cout << GREEN << "Final solution: subproblem INFEASIBLE " << RESET
                << std::endl;
      sub.computeIIS(); // 计算不可行子系统
      std::string iis_path = "D:/PYPJ/qbota/infeasible_subproblem.ilp";
      sub.write(iis_path);
    } else {
      std::cout << GREEN << "Final solution: subproblem status = " << status
                << RESET << std::endl;
    }
  }

  void print_solution() {
    std::cout << GREEN << "=== Final Solution ===" << RESET << std::endl;
    for (int i = 0; i < d_.flightNumber; ++i) {
      for (int k = 0; k <= d_.gateNumber; ++k) {
        double x_val = x_[i][k].get(GRB_DoubleAttr_X);
        if (x_val > 0.5) {
          std::cout << GREEN << "Flight " << i << " assigned to gate " << k
                    << " with x[" << i << "][" << k << "] = " << x_val << RESET
                    << std::endl;
        }
      }
    }

    for (auto &[key, var] : y_) {
      double y_val = var.get(GRB_DoubleAttr_X);
      if (y_val > 0.5) {
        std::cout << GREEN << "y[{" << key.i << "," << key.j << "," << key.k
                  << "}] = " << y_val << RESET << std::endl;
      }
    }

    for (auto &[key, var] : z_) {
      double z_val = var.get(GRB_DoubleAttr_X);
      if (z_val > 0.5) {
        if (key.u == key.v)
          continue;
        std::cout << GREEN << "z[{" << key.i << "," << key.delta_i << ","
                  << key.u << "," << key.v << "}] = " << z_val << RESET
                  << std::endl;
      }
    }

    for (int s = 0; s < eta_.size(); ++s) {
      double eta_val = eta_[s].get(GRB_DoubleAttr_X);
      std::cout << GREEN << "eta[" << s << "] = " << eta_val << RESET
                << std::endl;
    }
  }

  double get_objective_value() { return master_.get(GRB_DoubleAttr_ObjVal); }

private:
  class BendersCallback : public GRBCallback {
  private:
    Solver &solver_;
    vector<caseData> senarios_;

  protected:
    void callback() override {
      if (where == GRB_CB_MIPSOL) {
        try {
          int senario_num = senarios_.size();
          double weight =
              1.0 / static_cast<double>(senario_num); // 平均分配每个场景的权重
          GRBLinExpr RHS = 0.0;

          unordered_map<yKey, double, yKeyHash>
              y_val; // 存储当前整数解中 y 变量的取值
          unordered_map<zKey, double, zKeyHash>
              z_val; // 存储当前整数解中 z 变量的取值

          for (auto &[key, var] : solver_.y_) {
            y_val[key] = getSolution(var);
          }
          for (auto &[key, var] : solver_.z_) {
            z_val[key] = getSolution(var);
          }

          for (int i = 0; i < solver_.d_.flightNumber; ++i) {
            for (int k = 0; k <= solver_.d_.gateNumber; ++k) {
              double x_val = getSolution(solver_.x_[i][k]);
              if (x_val > 0.5) {
                std::cout << GREEN << "x_[" << i << "][" << k << "] = " << x_val
                          << RESET << std::endl;
              }
            }
          }

          for (auto &[key, val] : y_val) {
            if (val > 0.5) {
              std::cout << GREEN << "y_val[{" << key.i << "," << key.j << ","
                        << key.k << "}] = " << val << RESET << std::endl;
            }
          }

          for (size_t s = 0; s < senarios_.size(); ++s) {
            caseData &senario = senarios_[s];

            auto [sub_ptr, rows] =
                solver_.build_single_senario_subproblem(senario, y_val, z_val);

            auto [status, cut] = solver_.get_cut(*sub_ptr, rows);

            if (status == GRB_INFEASIBLE) {
              addLazy(cut <= 0);
            } else if (status == GRB_OPTIMAL) {
              // RHS += cut;
              //  最优性割：获取主问题对当前场景 s 的估算值
              double master_eta_s_val = getSolution(solver_.eta_[s]);
              // 获取子问题算出的场景 s 的真实延误值
              double sub_obj_s_val = sub_ptr->get(GRB_DoubleAttr_ObjVal);

              if (sub_obj_s_val > master_eta_s_val + 1e-5) {
                addLazy(solver_.eta_[s] >= cut);
              }
            }
            // addLazy(solver_.eta_ >= RHS);
          }

        } catch (const GRBException &e) {
          std::cerr << "Gurobi error in callback: " << e.getMessage()
                    << std::endl;
        } catch (const std::exception &e) {
          std::cerr << "Exception in callback: " << e.what() << std::endl;
        } catch (...) {
          std::cerr << "Unknown exception in callback" << std::endl;
        } // === 添加 catch ===
      }
    }

  public:
    BendersCallback(Solver &solver, const vector<caseData> &senarios)
        : solver_(solver), senarios_(senarios) {}
  };

  static GRBEnv createEnv(const std::string &logFile) {
    GRBEnv env(true);
    env.set("LogFile", logFile);
    env.set(GRB_IntParam_OutputFlag, 0);
    env.start();
    return env;
  }

  caseData d_;
  int BIG_M = 100000;
  GRBEnv env_;
  GRBEnv sub_env_;
  GRBModel master_;
  vector<GRBVar> eta_;
  vector<vector<GRBVar>> x_;
  vector<GRBVar> gate_used_;
  unordered_map<yKey, GRBVar, yKeyHash> y_;
  unordered_map<zKey, GRBVar, zKeyHash> z_;
};

double calculate_distance(std::pair<double, double> coord1,
                          std::pair<double, double> coord2) {
  double dx = coord1.first - coord2.first;
  double dy = coord1.second - coord2.second;
  return std::sqrt(dx * dx + dy * dy);
}

caseData read_case_data(const std::string &filePath) {
  std::ifstream f(filePath);

  if (!f.is_open()) {
    std::cerr << "Failed to open file: " << filePath << std::endl;
    exit(1);
  }

  int flight_number, gate_number, buffer_time, apron_penalty, delay_penalty;
  double tow_speed;
  f >> flight_number >> gate_number >> buffer_time >> apron_penalty >>
      delay_penalty >> tow_speed;

  double penalty_per_meter = 30.0 / 2000.0;

  auto apron_penalty_cost = vector<double>(flight_number, apron_penalty);
  auto delay_penalty_cost = vector<double>(flight_number, delay_penalty);
  unordered_map<int, std::pair<double, double>> gate_coords;
  vector<int> medium_gates;
  vector<int> large_flights;
  vector<int> departFlights;
  vector<int> noDepartArr;
  vector<int> haveDepartArr;
  map<int, int> delta;

  for (int i = 0; i <= gate_number; ++i) {
    int id;
    double x, y;
    std::string size;
    f >> id >> x >> y >> size;
    gate_coords[id] = {x, y};
    if (size == "medium")
      medium_gates.push_back(id);
  }

  vector<vector<double>> tow_time(gate_number + 1,
                                  vector<double>(gate_number + 1));
  vector<vector<double>> tow_cost(gate_number + 1,
                                  vector<double>(gate_number + 1));

  for (int i = 0; i <= gate_number; ++i) {
    for (int j = 0; j <= gate_number; ++j) {
      double distance = calculate_distance(gate_coords[i], gate_coords[j]);
      tow_time[i][j] = distance / tow_speed;
      tow_cost[i][j] = distance * penalty_per_meter;
    }
  }

  vector<Flight> flights(flight_number);
  unordered_map<int, int> departure_pert;

  for (int i = 0; i < flight_number; ++i) {
    int id, scheduled_time;
    std::string type, size;
    int departure_id, departure_perturbation;
    f >> id >> scheduled_time >> type >> size >> departure_id >>
        departure_perturbation;
    flights[i] = Flight{id, type == "departure" ? DEPARTURE : ARRIVAL,
                        size == "large" ? LARGE : MEDIUM, scheduled_time};
    if (type == "arrival" and departure_id != -1) {
      delta[id] = departure_id;
      haveDepartArr.push_back(id);
    }

    if (type == "arrival" and departure_id == -1) {
      noDepartArr.push_back(id);
    }

    if (type == "departure") {
      departFlights.push_back(id);
    }

    if (size == "large") {
      large_flights.push_back(id);
    }

    departure_pert[id] = departure_perturbation;
  }

  return makeCaseData(
      flight_number, gate_number, buffer_time, apron_penalty_cost,
      delay_penalty_cost, flights, noDepartArr, haveDepartArr, departFlights,
      delta, medium_gates, large_flights, tow_cost, tow_time, departure_pert);
}

int main() {

  caseData base_case = read_case_data("D:/PYPJ/qbota/toy_data3.txt");
  vector<caseData> senarios = {base_case};

  std::string senario_path = "D:/PYPJ/qbota/toy_senario3";
  for (const auto &entry : fs::directory_iterator(senario_path)) {
    if (entry.path().extension() == ".txt") {
      senarios.push_back(read_case_data(entry.path().string()));
    }
  }

  // senarios.push_back(read_case_data("D:/PYPJ/qbota/toy_senario1.txt"));
  // senarios.push_back(read_case_data("D:/PYPJ/qbota/toy_senario2.txt"));

  Solver solver(base_case);
  solver.build_master_problem(senarios.size());

  auto start_time = std::chrono::high_resolution_clock::now();

  solver.solve_with_callback(senarios);

  std::cout << GREEN << "Optimization complete." << RESET << std::endl;
  std::cout << GREEN
            << "Optimal objective value: " << solver.get_objective_value()
            << RESET << std::endl;

  solver.final_solution_feasibility_check(senarios[1]);

  auto end_time = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed = end_time - start_time;
  std::cout << GREEN << "Total elapsed time: " << elapsed.count() << " seconds"
            << RESET << std::endl;

  return 0;
}