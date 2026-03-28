#include "gurobi_c++.h"

#include "case_data.hpp"
#include "gurobi_c.h"
#include "modelDef.hpp"

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using std::map;
using std::to_string;
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

  std::unordered_map<int, int> departure_pert;

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
  Solver(const caseData &data)
      : d_(data), env_(createEnv("gate_assignment.log")), master_(env_) {}

  void build_master_problem() {
    auto flight_number = d_.flightNumber;
    auto gate_number = d_.gateNumber;
    const auto &no_depart_arr = d_.noDepartArr;

    x_.resize(flight_number, vector<GRBVar>(gate_number + 1));
    // 决策变量 x[i][j]，表示航班 i 是否分配到登机口 j , x[i][d.apronIndex]
    // 表示航班 i 分配停机坪
    for (int i = 0; i < flight_number; ++i) {
      for (int k = 0; k <= gate_number; ++k) {
        auto varName = "x_" + to_string(i) + "_" + to_string(k);
        if (k < gate_number) { // 如果分配到登机口
          if (std::find(no_depart_arr.begin(), no_depart_arr.end(), i) ==
              no_depart_arr.end()) {
            x_[i][k] = master_.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
          } else // 如果是到达后不离开的，增加拖行到停机坪的成本
            x_[i][k] = master_.addVar(0.0, 1.0, d_.towCost[k][d_.apronIndex],
                                      GRB_BINARY, varName);
        } else {
          // 分配到停机坪的惩罚成本
          x_[i][k] = master_.addVar(0.0, 1.0, d_.apronPenaltyCost[i],
                                    GRB_BINARY, varName);
        }
        // 航班的停机坪分配成本
      }
    }
    //---------------------------
    // 决策变量  y[i][j][k]，表示航班 i 和 j 在登机口 k 连续执行
    int virtual_start_flight = flight_number;   // 虚拟起始航班索引
    int virtual_end_flight = flight_number + 1; // 虚拟结束航班索引

    for (int i = 0; i <= flight_number + 1; ++i) {
      for (int j = 0; j <= flight_number + 1; ++j) {
        for (int k = 0; k <= gate_number; ++k) {
          auto varName =
              "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k);
          y_[{i, j, k}] = master_.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
        }
      }
    }
    //---------------------------- z_i_j_u_v
    // 决策变量 z[i][j][u][v]，表示关联航班 i 和 j 从 登机口 u 拖行到 v

    for (int i = 0; i < flight_number; ++i) {
      auto delta_i = d_.delta.at(i); // 到港航班 i 对应的离开航班ID
      for (int j = 0; j < flight_number; ++j) {
        for (int u = 0; u <= gate_number; ++u) {
          for (int v = 0; v <= gate_number; ++v) {
            auto varName = "z_" + to_string(i) + "_" + to_string(j) + "_" +
                           to_string(u) + "_" + to_string(v);
            if (j == delta_i)
              z_[{i, j, u, v}] = master_.addVar(0.0, 1.0, d_.towCost[u][v],
                                                GRB_BINARY, varName);
            else
              z_[{i, j, u, v}] =
                  master_.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
          }
        }
      }
    }

    // 非线性部分
    eta_ = master_.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "eta");

    // 主问题约束------------------------
    // 每个航班分配一个登机口或停机坪
    for (int i = 0; i < flight_number; ++i) {
      GRBLinExpr expr = 0;
      for (int j = 0; j <= gate_number; ++j) {
        expr += x_[i][j];
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

    // 每个航班都有后续航班或者到虚拟结束航班
    for (int i = 0; i < flight_number; ++i) {
      for (int k = 0; k < gate_number; ++k) {
        GRBLinExpr right_hand_side = 0;
        for (int j = 0; j < flight_number; ++j) {
          if (j != i) {
            right_hand_side += y_[{i, j, k}];
          }
          right_hand_side += y_[{i, virtual_end_flight,
                                 k}]; // i 后面没有航班了，直接到虚拟结束航班
        }
        master_.addConstr(x_[i][k] == right_hand_side, "GateSuccessionConstr_" +
                                                           to_string(i) + "_" +
                                                           to_string(k));
      }
    }

    // 虚拟起始航班有一个后续
    for (int k = 0; k < gate_number; ++k) {
      GRBLinExpr left_hand_side = 0;
      for (int j = 0; j < flight_number; ++j) {
        left_hand_side += y_[{virtual_start_flight, j, k}];
      }
      left_hand_side += y_[{virtual_start_flight, virtual_end_flight, k}];
      master_.addConstr(left_hand_side == 1,
                        "VirtualStartConstr_" + to_string(k));
    }

    // 虚拟终止航班有一个前驱
    for (int k = 0; k < gate_number; ++k) {
      GRBLinExpr left_hand_side = 0;
      for (int j = 0; j < flight_number; ++j) {
        left_hand_side += y_[{j, virtual_end_flight, k}];
      }
      left_hand_side += y_[{virtual_start_flight, virtual_end_flight, k}];
      master_.addConstr(left_hand_side == 1,
                        "VirtualEndConstr_" + to_string(k));
    }

    // 航班分配约束
    for (int i = 0; i < flight_number; ++i) {
      for (int k = 0; k < gate_number; ++k) {
        GRBLinExpr left_hand_side = 0;
        GRBLinExpr right_hand_side = 0;
        for (int j = 0; j < flight_number; ++j) {
          if (j != i) {
            left_hand_side += y_[{j, i, k}];
            right_hand_side += y_[{i, j, k}];
          }
        }
        left_hand_side += y_[{virtual_start_flight, i, k}];
        right_hand_side += y_[{i, virtual_end_flight, k}];

        master_.addConstr(left_hand_side == right_hand_side,
                          "FlowConstr_" + to_string(i) + "_" + to_string(k));
      }
    }

    // 拖行约束
    for (int i : d_.haveDepartArr) {
      int delta_i = d_.delta.at(i);
      for (int u = 0; u < gate_number; ++u) {
        for (int v = 0; v < gate_number; ++v) {
          if (v != u) {
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

        int k = u; // 同一登机口的情况
        master_.addConstr(x_[i][k] + x_[delta_i][k] - y_[{i, delta_i, k}] <=
                              z_[{i, delta_i, k, k}] + 1,
                          "TowConstrSameGate_" + to_string(i) + "_" +
                              to_string(delta_i) + "_" + to_string(k));
      }
    }

    // 影子约束
    for (int i : d_.largeFlights) {
      for (int j : d_.largeFlights) {
        if (j != i) {
          for (int k = 0; k < gate_number; ++k) {
            if (std::find(d_.mediumGates.begin(), d_.mediumGates.end(), k) !=
                d_.mediumGates.end())
              continue;
            master_.addConstr(x_[i][k] + x_[j][(k + 1) % gate_number] <= 1,
                              "ShadowConstr_" + to_string(i) + "_" +
                                  to_string(j) + "_" + to_string(k));
          }
        }
      }
    }
  }

  std::pair<GRBModel, vector<Row>>
  build_single_senario_subproblem(caseData &data) {

    GRBModel sub(env_);

    int flight_number = d_.flightNumber;
    auto departure_pert = data.departure_pert;

    std::unordered_map<int, GRBVar> departure_time;
    std::unordered_map<int, GRBVar> arrival_time;
    std::unordered_map<int, GRBVar> arrival_delay;

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
              d_.towTime[u][v] * z_[{i, delta_i, u, v}];
          Row row;
          row.constr = sub.addConstr(left_hand_side >= right_hand_side,
                                     "SubTowConstr_" + to_string(i) + "_" +
                                         to_string(delta_i) + "_" +
                                         to_string(u) + "_" + to_string(v));

          row.constant = static_cast<double>(departure_pert.at(delta_i));
          row.x_coeff = 0.0;
          row.y_coeff = 0.0;
          row.z_coeff = d_.towTime[u][v];
          row.x_var = {0, 0};
          row.y_var = {0, 0, 0};
          row.z_var = {i, delta_i, u, v};
          rows.push_back(row);
        }
      }
    }

    // 3-19
    for (int i : d_.noDepartArr) {
      int delta_i = d_.delta.at(i);
      for (int j : d_.departFlights) {
        if (j == delta_i)
          continue;
        for (int k = 0; k < d_.gateNumber; ++k) {
          GRBLinExpr left_hand_side = departure_time[j] - arrival_time[i];
          GRBLinExpr right_hand_side = BIG_M * (y_[{i, j, k}] - 1) +
                                       d_.bufferTime + departure_pert.at(j);
          Row row;
          row.constr =
              sub.addConstr(left_hand_side >= right_hand_side,
                            "SubSuccessionConstr_" + to_string(i) + "_" +
                                to_string(j) + "_" + to_string(k));
          row.constant = d_.bufferTime +
                         static_cast<double>(departure_pert.at(j)) -
                         static_cast<double>(BIG_M);
          row.x_coeff = 0.0;
          row.y_coeff = static_cast<double>(BIG_M);
          row.z_coeff = 0.0;
          row.x_var = {0, 0};
          row.y_var = {i, j, k};
          row.z_var = {0, 0, 0, 0};
          rows.push_back(row);
        }
      }
    }

    // 3-19
    for (int i : d_.haveDepartArr) {
      int delta_i = d_.delta.at(i);
      for (int j : d_.departFlights) {
        if (j == delta_i)
          continue;
        for (int k = 0; k < d_.gateNumber; ++k) {
          GRBLinExpr left_hand_side = departure_time[j] - arrival_time[i];
          GRBLinExpr right_hand_side = BIG_M * (y_[{i, j, k}] - 1) +
                                       d_.bufferTime + departure_pert.at(j);
          Row row;
          row.constr =
              sub.addConstr(left_hand_side >= right_hand_side,
                            "SubSuccessionConstr_" + to_string(i) + "_" +
                                to_string(j) + "_" + to_string(k));
          row.constant = d_.bufferTime +
                         static_cast<double>(departure_pert.at(j)) -
                         static_cast<double>(BIG_M);
          row.x_coeff = 0.0;
          row.y_coeff = static_cast<double>(BIG_M);
          row.z_coeff = 0.0;
          row.x_var = {0, 0};
          row.y_var = {i, j, k};
          row.z_var = {0, 0, 0, 0};
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
              BIG_M * (y_[{i, j, k}] - 1) + d_.bufferTime;

          Row row;
          row.constr =
              sub.addConstr(left_hand_side >= right_hand_side,
                            "SubArrivalSuccessionConstr_" + to_string(i) + "_" +
                                to_string(j) + "_" + to_string(k));

          row.constant = d_.bufferTime - static_cast<double>(BIG_M);
          row.x_coeff = 0.0;
          row.y_coeff = static_cast<double>(BIG_M);
          row.z_coeff = 0.0;
          row.x_var = {0, 0};
          row.y_var = {i, j, k};
          row.z_var = {0, 0, 0, 0};
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
              BIG_M * (y_[{i, j, k}] - 1) + d_.bufferTime;

          Row row;
          row.constr =
              sub.addConstr(left_hand_side >= right_hand_side,
                            "SubDepartArrivalSuccessionConstr_" + to_string(i) +
                                "_" + to_string(j) + "_" + to_string(k));

          row.constant = d_.bufferTime - static_cast<double>(BIG_M);
          row.x_coeff = 0.0;
          row.y_coeff = static_cast<double>(BIG_M);
          row.z_coeff = 0.0;
          row.x_var = {0, 0};
          row.y_var = {i, j, k};
          row.z_var = {0, 0, 0, 0};

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
          GRBLinExpr right_hand_side = BIG_M * (y_[{i, j, k}] - 1) +
                                       d_.bufferTime + departure_pert.at(j);

          Row row;
          row.constr =
              sub.addConstr(left_hand_side >= right_hand_side,
                            "SubDepartSuccessionConstr_" + to_string(i) + "_" +
                                to_string(j) + "_" + to_string(k));

          row.constant = d_.bufferTime - static_cast<double>(BIG_M) +
                         static_cast<double>(departure_pert.at(j));
          row.x_coeff = 0.0;
          row.y_coeff = static_cast<double>(BIG_M);
          row.z_coeff = 0.0;
          row.x_var = {0, 0};
          row.y_var = {i, j, k};
          row.z_var = {0, 0, 0, 0};
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
      row.x_coeff = 0.0;
      row.y_coeff = 0.0;
      row.z_coeff = 0.0;
      row.x_var = {0, 0};
      row.y_var = {0, 0, 0};
      row.z_var = {0, 0, 0, 0};
      rows.push_back(row);
    }

    // 3-24
    for (int j : arrival_flights) {
      Row row;
      row.constr =
          sub.addConstr(arrival_time[j] >= data.flightMap.at(j).scheduled_time,
                        "SubArrivalScheduledTimeConstr_" + to_string(j));
      row.constant = static_cast<double>(d_.flightMap.at(j).scheduled_time);
      row.x_coeff = 0.0;
      row.y_coeff = 0.0;
      row.z_coeff = 0.0;
      row.x_var = {0, 0};
      row.y_var = {0, 0, 0};
      row.z_var = {0, 0, 0, 0};
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
      row.x_coeff = 0.0;
      row.y_coeff = 0.0;
      row.z_coeff = 0.0;
      row.x_var = {0, 0};
      row.y_var = {0, 0, 0};
      row.z_var = {0, 0, 0, 0};
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

    return {sub, rows};
  }

  std::pair<int, GRBLinExpr> get_cut(GRBModel &sub, const vector<Row> &rows,
                                     const double weight) {
    sub.optimize(); // 求解子问题

    int status = sub.get(GRB_IntAttr_Status);

    if (status == GRB_INFEASIBLE) {
      return {status, get_feasibility_cut(rows)};
    } else if (status == GRB_OPTIMAL) {
      return {status, get_optimality_cut(rows, weight)};
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
      double lambda_i = row.constr.get(GRB_DoubleAttr_FarkasDual);
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

  GRBLinExpr get_optimality_cut(const vector<Row> &rows, const double weight) {
    GRBLinExpr rhs = 0.0;
    for (const auto &row : rows) {
      GRBLinExpr expr = 0.0;
      double pi_i = row.constr.get(GRB_DoubleAttr_Pi);
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
    rhs *= weight;
    return rhs;
  }

  void solve_with_callback(vector<caseData> &senarios) {
    master_.set(GRB_IntParam_LazyConstraints, 1);
    BendersCallback cb(this, senarios);
    master_.setCallback(&cb);
    master_.optimize();
    master_.setCallback(nullptr); // 取消回调，准备添加割
  }

private:
  class BendersCallback : public GRBCallback {
  private:
    Solver *solver_;
    vector<caseData> senarios_;

  protected:
    void callback() override {
      if (where == GRB_CB_MIPSOL) {
        int senario_num = senarios_.size();
        double weight = 1.0 / static_cast<double>(senario_num); // 平均分配权重
        GRBLinExpr RHS = 0.0;
        for (caseData &senario : senarios_) {
          auto [sub, rows] = solver_->build_single_senario_subproblem(senario);
          auto [status, cut] = solver_->get_cut(sub, rows, weight);
          if (status == GRB_INFEASIBLE) {
            addLazy(cut <= 0);
          } else if (status == GRB_OPTIMAL) {
            RHS += cut;
          }
        }
        addLazy(solver_->eta_ >= RHS);
      }
    }

  public:
    BendersCallback(Solver *solver, const vector<caseData> &senarios)
        : solver_(solver), senarios_(senarios) {}
  };

  static GRBEnv createEnv(const std::string &logFile) {
    GRBEnv env(true);
    env.set("LogFile", logFile);
    env.start();
    return env;
  }

  caseData d_;
  int BIG_M = 1450;
  GRBEnv env_;
  GRBModel master_;
  GRBVar eta_;
  vector<vector<GRBVar>> x_;
  std::unordered_map<yKey, GRBVar, yKeyHash> y_;
  std::unordered_map<zKey, GRBVar, zKeyHash> z_;
};

int main() {
  auto env = GRBEnv(true);
  env.set("LogFile", "gate_assignment.log");
  env.start();

  auto m = GRBModel(env);

  auto d = makeSampleCaseData();

  vector<vector<GRBVar>> x(d.flightNumber, vector<GRBVar>(d.gateNumber + 1));
  // 决策变量 x[i][j]，表示航班 i 是否分配到登机口 j , x[i][d.apronIndex]
  // 表示航班 i 分配停机坪

  for (int i = 0; i < d.flightNumber; ++i) {
    for (int j = 0; j <= d.gateNumber; ++j) {
      auto varName = "x_" + to_string(i) + "_" + to_string(j);
      if (j < d.gateNumber)
        x[i][j] = m.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
      else
        x[i][j] =
            m.addVar(0.0, 1.0, d.apronPenaltyCost[i], GRB_BINARY, varName);
      // 航班的停机坪分配成本
    }
  }

  for (auto i : d.noDepartArr) {
    for (int k = 0; k < d.gateNumber; ++k) {
      x[i][k].set(GRB_DoubleAttr_Obj, d.towCost[k][d.apronIndex]);
    }
  } // 对于到达后不离开的航班，设置拖行到停机坪的成本

  auto eta = m.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "eta");
  // =延误成本 + 拖行成本

  m.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // 最小化

  for (auto i = 0; i < d.flightNumber; ++i) {
    GRBLinExpr expr = 0;
    for (int j = 0; j <= d.gateNumber; ++j) {
      expr += x[i][j];
    }
    m.addConstr(expr == 1, "AssignConstr_" + to_string(i));
  } // 每个航班分配一个登机口或停机坪

  for (auto i : d.largeFlights) {
    for (auto k : d.mediumGates) {
      m.addConstr(x[i][k] == 0,
                  "Flight-GateConstr_" + to_string(i) + "_" + to_string(k));
    }
  } // 大型飞机不能分配到中等大小登机口

  // 写一个benders分解， 主问题算x， 子问题算 x的延误成本和拖行成本
  double LB = 0.0, UB = 1e6, eps = 10.0;

  while (UB - LB > eps) {
    m.optimize();
    auto Y = determine_Y(x, d.flights);
    sparseZ Z;
    double tow_cost = calculateTowCost(env, d, x, Y, Z);
    double delay_cost = calculateDelayCost(env, d, x, Y, Z);
    double total_cost = tow_cost + delay_cost;

    // cTx + eta - eta + total_cost
    UB = std::min(UB, m.get(GRB_DoubleAttr_ObjVal) - eta.get(GRB_DoubleAttr_X) +
                          total_cost);

    // 添加割
    /*  待补充的内容 */

    LB = std::max(LB, m.get(GRB_DoubleAttr_ObjVal));

    std::cout << "Current LB: " << LB << ", UB: " << UB << std::endl;
  }

  return 0;
}