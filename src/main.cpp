#include "gurobi_c++.h"

#include "modelDef.hpp"
#include "case_data.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>

using std::map;
using std::set;
using std::to_string;
using std::vector;

caseData makeSampleCaseData()
{
    int FlightNumber = 400;                                                               // 全局变量，航班数量
    int GateNumber = 50;                                                                  // 全局变量，登机口数量
    int ApronIndex = GateNumber;                                                          // 明确停机坪索引
    vector<double> ApronPenaltyCost(FlightNumber, 10.0);                                  // 每个航班的停机坪惩罚成本
    vector<Flight> flights(FlightNumber);                                                 // 航班信息列表
    vector<int> No_departue_arrival_flights_indices;                                      // \underline{F_a}  不离开的到达航班索引
    vector<int> Have_departue_arrival_flights_indices;                                    // \overline{F_a}   有离开的到达航班索引
    vector<int> Departure_flights_indices;                                                // 离开航班索引  F_d
    map<int, int> arrival_to_departure_map;                                               //  \delta 映射 ： 到达航班ID -> 离开航班ID
    vector<int> MediumGates;                                                              // 中等大小登机口集合
    vector<int> LargeFlights;                                                             // 大型飞机航班集合
    vector<vector<double>> TowCost(GateNumber + 1, vector<double>(GateNumber + 1, 50.0)); // 拖行成本

    return makeCaseData(FlightNumber, GateNumber,
                        ApronPenaltyCost,
                        flights,
                        No_departue_arrival_flights_indices,
                        Have_departue_arrival_flights_indices,
                        Departure_flights_indices,
                        arrival_to_departure_map,
                        MediumGates,
                        LargeFlights,
                        TowCost);
}

using Succession = vector<vector<vector<uint8_t>>>; // y_i_j_k的类型，连续执行关系的类型别名

Succession determine_Y(const vector<vector<GRBVar>> &x,
                       const vector<Flight> &flights,
                       int FlightNumber,
                       int GateNumber)
{
    // 根据 x 计算 Y

    Succession Y(FlightNumber, vector<vector<uint8_t>>(FlightNumber, vector<uint8_t>(GateNumber, 0)));
    // Y[i][j][k] = 1 表示 航班 i 和 j 在登机口 k 连续执行

    map<int, vector<Flight>> flights_in_gate;

    for (int i = 0; i < FlightNumber; ++i)
    {
        for (int j = 0; j < GateNumber; ++j)
        {
            if (x[i][j].get(GRB_DoubleAttr_X) > 0.5)
            {
                flights_in_gate[j].push_back(flights[i]);
            }
        }
    }

    for (auto &[gate, flight_list] : flights_in_gate)
    {
        // 按计划时间排序
        std::sort(flight_list.begin(), flight_list.end());

        for (size_t idx = 0; idx + 1 < flight_list.size(); ++idx)
        {
            int i = flight_list[idx].flight_id;
            int j = flight_list[idx + 1].flight_id;
            Y[i][j][gate] = 1;
        }
    }

    return Y;
}

auto solve_towcost_subproblem(GRBEnv const &env, const vector<vector<GRBVar>> &x, const caseData &d)
{

    auto &F_a_up = d.haveDepartArr; // 有离开的到达航班索引
    // 起个别名方便书写

    auto sub_m = GRBModel(env);

    sub_m.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // 最小化

    // z_i_u_v 变量表示航班 i 是否从登机口 u 拖行到登机口 v
    vector<vector<vector<GRBVar>>> z(F_a_up.size(), vector<vector<GRBVar>>(d.gateNumber + 1, vector<GRBVar>(d.gateNumber + 1)));

    for (auto i : F_a_up)
    {
        int idx = -1;
        for (size_t k = 0; k < F_a_up.size(); ++k)
        {
            if (F_a_up[k] == i)
            {
                idx = k;
                break;
            }
        }
        auto delta_i = d.delta.at(i);
        for (int u = 0; u <= d.gateNumber; ++u)
        {
            for (int v = 0; v <= d.gateNumber; ++v)
            {
                auto varName = "z_" + to_string(i) + "_" + to_string(delta_i) + "_" + to_string(u) + "_" + to_string(v);
                z[idx][u][v] = sub_m.addVar(0.0, 1.0, d.towCost[u][v], GRB_BINARY, varName);
            }
        }
    }

    auto Y = determine_Y(x, d.flights, d.flightNumber, d.gateNumber);

    for (auto i : F_a_up)
    {
        for (int u = 0; u <= d.gateNumber; ++u)
        {
            for (int v = 0; v <= d.gateNumber; ++v)
            {
                int idx = -1;
                for (size_t k = 0; k < F_a_up.size(); ++k)
                {
                    if (F_a_up[k] == i)
                    {
                        idx = k;
                        break;
                    }
                }
                auto delta_i = d.delta.at(i);
                if (u == v)
                {
                    sub_m.addConstr(x[i][u] + x[delta_i][v] - Y[i][delta_i][u] <= z[idx][u][u] + 1);
                    // 约束3-12
                }
                else
                {
                    sub_m.addConstr(z[idx][u][v] <= x[i][u]);
                    sub_m.addConstr(z[idx][u][v] <= x[delta_i][v]);
                    sub_m.addConstr(x[i][u] + x[delta_i][v] <= z[idx][u][v] + 1);
                    // 约束3-9  3-10  3-11
                }
            }
        }
    }

    sub_m.optimize();

    return sub_m.get(GRB_DoubleAttr_ObjVal);
}

auto solve_delaycost_subproblem(GRBEnv const &env, const vector<vector<GRBVar>> &x)
{
    // 计算期望延误成本 E(x)

    auto sub_m = GRBModel(env);

    sub_m.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // 最小化
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
    // =E(x) ， 期望延误成本
    auto tow_cost = m.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "tow_cost");
    // C_tow(x)   ， 机位拖行成本

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

    return 0;
}