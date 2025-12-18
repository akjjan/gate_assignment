#include "gurobi_c++.h"
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

enum FlightType
{
    ARRIVAL,
    DEPARTURE
}; // 枚举类型表示航班类型

enum AircraftSize
{
    MEDIUM,
    LARGE
}; // 枚举类型表示飞机大小

struct Flight
{
    int flight_id;              // 航班ID
    FlightType flight_type;     // 类型
    AircraftSize aircraft_size; // 飞机大小
    int scheduled_time;         // 计划时间
};

const int FlightNumber = 500; // 全局变量，航班数量

const int GateNumber = 50; // 全局变量，登机口数量

vector<double> ApronPenaltyCost(FlightNumber, 10.0); // 每个航班的停机坪惩罚成本

vector<Flight> flights(FlightNumber); // 航班信息列表

set<int> No_departue_arrival_flights_indices;   // \underline{F_a}
set<int> Have_departue_arrival_flights_indices; // \overline{F_a}
// 分别存储到达后不离开和有离开的  到达航班索引

set<int> Departure_flights_indices;
// 离开航班索引  F_d

map<int, int> arrival_to_departure_map;
//  \delta 映射 ： 到达航班ID -> 离开航班ID

set<int> MediumGates; // 中等大小登机口集合

set<int> LargeFlights; // 大型飞机航班集合

vector<vector<double>> TowCost(GateNumber + 1, vector<double>(GateNumber + 1, 50.0));
// 拖行成本

bool operator<(const Flight &a, const Flight &b)
{
    return a.scheduled_time < b.scheduled_time;
}

auto &determine_Y(const vector<vector<GRBVar>> &x)
{
    // 根据 x 计算 Y

    vector<vector<vector<uint8_t>>> Y(FlightNumber, vector<vector<uint8_t>>(FlightNumber, vector<uint8_t>(GateNumber, 0)));
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

auto solve_towcost_subproblem(GRBEnv const &env, const vector<vector<GRBVar>> &x)
{

    auto &F_a_up = Have_departue_arrival_flights_indices;
    // 起个别名方便书写

    auto sub_m = GRBModel(env);

    sub_m.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // 最小化

    // z_i_u_v 变量表示航班 i 是否从登机口 u 拖行到登机口 v
    vector<vector<vector<GRBVar>>> z(F_a_up.size(), vector<vector<GRBVar>>(GateNumber + 1, vector<GRBVar>(GateNumber + 1)));

    for (auto i : F_a_up)
    {
        int idx = std::distance(F_a_up.begin(),
                                F_a_up.find(i));
        auto delta_i = arrival_to_departure_map[i];
        for (int u = 0; u <= GateNumber; ++u)
        {
            for (int v = 0; v <= GateNumber; ++v)
            {
                auto varName = "z_" + to_string(i) + "_" + to_string(delta_i) + "_" + to_string(u) + "_" + to_string(v);
                z[idx][u][v] = sub_m.addVar(0.0, 1.0, TowCost[u][v], GRB_BINARY, varName);
            }
        }
    }

    auto Y = determine_Y(x);

    for (auto i : F_a_up)
    {
        for (int u = 0; u <= GateNumber; ++u)
        {
            for (int v = 0; v <= GateNumber; ++v)
            {
                int idx = std::distance(F_a_up.begin(),
                                        F_a_up.find(i));
                auto delta_i = arrival_to_departure_map[i];
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

    vector<vector<GRBVar>> x(FlightNumber, vector<GRBVar>(GateNumber + 1));
    // 创建决策变量 x[i][j]，表示航班 i 是否分配到登机口 j , x[i][GateNumber] 表示航班 i 分配停机坪

    for (int i = 0; i < FlightNumber; ++i)
    {
        for (int j = 0; j <= GateNumber; ++j)
        {
            auto varName = "x_" + to_string(i) + "_" + to_string(j);
            if (j < GateNumber)
                x[i][j] = m.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
            else
                x[i][j] = m.addVar(0.0, 1.0, ApronPenaltyCost[i], GRB_BINARY, varName);
            // 航班的停机坪分配成本
        }
    }

    for (auto i : No_departue_arrival_flights_indices)
    {
        for (int k = 0; k < GateNumber; ++k)
        {
            x[i][k].set(GRB_DoubleAttr_Obj, TowCost[k][GateNumber]);
        }
    } // 对于到达后不离开的航班，设置拖行到停机坪的成本

    auto eta = m.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "eta");
    // =E(x) ， 期望延误成本
    auto tow_cost = m.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "tow_cost");
    // C_tow(x)   ， 机位拖行成本

    m.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // 最小化

    for (auto i = 0; i < FlightNumber; ++i)
    {
        GRBLinExpr expr = 0;
        for (int j = 0; j <= GateNumber; ++j)
        {
            expr += x[i][j];
        }
        m.addConstr(expr == 1, "AssignConstr_" + to_string(i));
    } // 每个航班分配一个登机口或停机坪

    for (auto i : LargeFlights)
    {
        for (auto k : MediumGates)
        {
            m.addConstr(x[i][k] == 0, "Flight-GateConstr_" + to_string(i) + "_" + to_string(k));
        }
    } // 大型飞机不能分配到中等大小登机口

    return 0;
}