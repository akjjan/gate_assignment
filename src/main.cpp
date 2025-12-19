#include "gurobi_c++.h"

#include "modelDef.hpp"
#include "case_data.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <unordered_set>

using std::map;
using std::set;
using std::to_string;
using std::unordered_set;
using std::vector;

caseData makeSampleCaseData()
{
    int flightNumber = 500;                                                               // 全局变量，航班数量
    int gateNumber = 50;                                                                  // 全局变量，登机口数量
    int apronIndex = gateNumber;                                                          // 明确停机坪索引
    int bufferTime = 15;                                                                  // 缓冲时间
    vector<double> apronPenaltyCost(flightNumber, 100.0);                                 // 每个航班的停机坪惩罚成本
    vector<Flight> flights(flightNumber);                                                 // 航班信息列表
    vector<int> noDepartArr;                                                              // \underline{F_a}  不离开的到达航班索引
    vector<int> haveDepartArr;                                                            // \overline{F_a}   有离开的到达航班索引
    vector<int> departFlights;                                                            // 离开航班索引  F_d
    map<int, int> delta;                                                                  //  \delta 映射 ： 到达航班ID -> 离开航班ID
    vector<int> mediumGates;                                                              // 中等大小登机口集合
    vector<int> largeFlights;                                                             // 大型飞机航班集合
    vector<vector<double>> towCost(gateNumber + 1, vector<double>(gateNumber + 1, 50.0)); // 拖行成本
    vector<vector<double>> towTime(gateNumber + 1, vector<double>(gateNumber + 1, 30.0)); // 拖行时间

    return makeCaseData(flightNumber, gateNumber, bufferTime,
                        apronPenaltyCost,
                        flights,
                        noDepartArr,
                        haveDepartArr,
                        departFlights,
                        delta,
                        mediumGates,
                        largeFlights,
                        towCost,
                        towTime);
}

map<int, vector<int>> get_flights_in_gate_map(const vector<vector<GRBVar>> &x)
{
    map<int, vector<int>> flights_in_gate;
    int FlightNumber = x.size();
    int GateNumber = x[0].size() - 1;
    for (int i = 0; i < FlightNumber; ++i)
    {
        for (int j = 0; j <= GateNumber; ++j)
        {
            if (x[i][j].get(GRB_DoubleAttr_X) > 0.5)
            {
                flights_in_gate[j].push_back(i);
            }
        }
    }

    return flights_in_gate;
}

using sparseSuccession = unordered_set<successionKey, successionKeyHash>;
// y_i_j_k，连续执行关系的类型别名，稀疏存储

sparseSuccession determine_Y(const vector<vector<GRBVar>> &x,
                             const vector<Flight> &flights)
{
    // 根据 x 计算 Y
    sparseSuccession Y;
    // {i，j，k} in Y 表示 航班 i 和 j 在登机口 k 连续执行

    auto flights_in_gate = get_flights_in_gate_map(x);

    for (auto &[gate, indices] : flights_in_gate)
    {
        // 按计划时间排序
        auto cmp = [&](int a, int b)
        {
            return flights[a].scheduled_time < flights[b].scheduled_time;
        };
        std::sort(indices.begin(), indices.end(), cmp);
        for (size_t idx = 0; idx + 1 < indices.size(); ++idx)
        {
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

double calculateTowCost(const GRBEnv &env, const caseData &d, const vector<vector<GRBVar>> &x, const sparseSuccession &Y, sparseZ &Z)
{

    auto &F_a_up = d.haveDepartArr; // 有离开的到达航班索引
                                    // 起个别名方便书写

    double tow_cost = 0.0;

    int GateNumber = d.gateNumber;

    for (auto i : F_a_up)
    {
        auto delta_i = d.delta.at(i); // 到港航班 i 对应的离开航班ID
        for (int u = 0; u <= GateNumber; ++u)
        {
            for (int v = 0; v <= GateNumber; ++v)
            {
                if (u == v)
                {
                    if (x[i][u].get(GRB_DoubleAttr_X) > 0.5 and x[delta_i][u].get(GRB_DoubleAttr_X) > 0.5 and Y.count({i, delta_i, u}) == 0)
                    {
                        tow_cost += d.towCost[u][u]; // 同一登机口拖行成本
                        Z.insert({i, delta_i, u, v});
                    }
                }
                else
                {
                    if (x[i][u].get(GRB_DoubleAttr_X) > 0.5 and x[delta_i][v].get(GRB_DoubleAttr_X) > 0.5)
                    {
                        tow_cost += d.towCost[u][v]; // 不同登机口拖行成本
                        Z.insert({i, delta_i, u, v});
                    }
                }
            }
        }
    }

    return tow_cost;
}

double calculateDelayCost(const GRBEnv &env, const caseData &d, const vector<vector<GRBVar>> &x, const sparseSuccession &Y, const sparseZ &Z)
{
    // 计算延误成本
    auto sub_m = GRBModel(env);
    sub_m.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // 最小化

    int flightNumber = x.size();

    vector<GRBVar> decisionTime(flightNumber);
    vector<GRBVar> arrivalDelay(flightNumber);

    double delay_cost = 0.0;

    for (int i = 0; i < flightNumber; ++i)
    {
        if (d.flightMap.at(i).flight_type == DEPARTURE)
        {
            decisionTime[i] = sub_m.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS);
            arrivalDelay[i] = sub_m.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            sub_m.addConstr(decisionTime[i] >= d.flightMap.at(i).scheduled_time);
            delay_cost -= d.flightMap.at(i).scheduled_time;
        }
        else
        {
            decisionTime[i] = sub_m.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            arrivalDelay[i] = sub_m.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS);
            sub_m.addConstr(decisionTime[i] >= d.flightMap.at(i).scheduled_time);
            sub_m.addConstr(arrivalDelay[i] >= decisionTime[i] - d.flightMap.at(i).scheduled_time);
        }
    }

    for (auto &[i, delta_i, u, v] : Z)
    {
        sub_m.addConstr(decisionTime[delta_i] >= decisionTime[i] + d.bufferTime + d.towTime[u][v]);
    }

    for (auto &[i, j, k] : Y)
    {
        sub_m.addConstr(decisionTime[j] >= decisionTime[i] + d.bufferTime);
    }

    sub_m.optimize();

    delay_cost += sub_m.get(GRB_DoubleAttr_ObjVal);

    return delay_cost;
}

int main()
{
    auto env = GRBEnv(true);
    env.set("LogFile", "gate_assignment.log");
    env.start();

    auto m = GRBModel(env);

    auto d = makeSampleCaseData();

    vector<vector<GRBVar>> x(d.flightNumber, vector<GRBVar>(d.gateNumber + 1));
    // 决策变量 x[i][j]，表示航班 i 是否分配到登机口 j , x[i][d.apronIndex] 表示航班 i 分配停机坪

    for (int i = 0; i < d.flightNumber; ++i)
    {
        for (int j = 0; j <= d.gateNumber; ++j)
        {
            auto varName = "x_" + to_string(i) + "_" + to_string(j);
            if (j < d.gateNumber)
                x[i][j] = m.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
            else
                x[i][j] = m.addVar(0.0, 1.0, d.apronPenaltyCost[i], GRB_BINARY, varName);
            // 航班的停机坪分配成本
        }
    }

    for (auto i : d.noDepartArr)
    {
        for (int k = 0; k < d.gateNumber; ++k)
        {
            x[i][k].set(GRB_DoubleAttr_Obj, d.towCost[k][d.apronIndex]);
        }
    } // 对于到达后不离开的航班，设置拖行到停机坪的成本

    auto eta = m.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "eta");
    // =延误成本 + 拖行成本

    m.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // 最小化

    for (auto i = 0; i < d.flightNumber; ++i)
    {
        GRBLinExpr expr = 0;
        for (int j = 0; j <= d.gateNumber; ++j)
        {
            expr += x[i][j];
        }
        m.addConstr(expr == 1, "AssignConstr_" + to_string(i));
    } // 每个航班分配一个登机口或停机坪

    for (auto i : d.largeFlights)
    {
        for (auto k : d.mediumGates)
        {
            m.addConstr(x[i][k] == 0, "Flight-GateConstr_" + to_string(i) + "_" + to_string(k));
        }
    } // 大型飞机不能分配到中等大小登机口

    // 写一个benders分解， 主问题算x， 子问题算 x的延误成本和拖行成本
    double LB = 0.0, UB = 1e6, eps = 10.0;

    while (UB - LB > eps)
    {
        m.optimize();
        auto Y = determine_Y(x, d.flights);
        sparseZ Z;
        double tow_cost = calculateTowCost(env, d, x, Y, Z);
        double delay_cost = calculateDelayCost(env, d, x, Y, Z);
        double total_cost = tow_cost + delay_cost;

        // cTx + eta - eta + total_cost
        UB = std::min(UB, m.get(GRB_DoubleAttr_ObjVal) - eta.get(GRB_DoubleAttr_X) + total_cost);

        // 添加割
        /*  待补充 */

        LB = std::max(LB, m.get(GRB_DoubleAttr_ObjVal));

        std::cout << "Current LB: " << LB << ", UB: " << UB << std::endl;
    }

    return 0;
}