#include "gurobi_c++.h"
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>

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
//  \delta 映射 ： 到达航班ID到离开航班ID的映射

set<int> MediumGates; // 中等大小登机口集合

set<int> LargeFlights; // 大型飞机航班集合

vector<vector<double>> TowCost(GateNumber + 1, vector<double>(GateNumber + 1, 50.0));
// 拖行成本

auto solve_subproblem(GRBEnv const &env, const vector<vector<GRBVar>> &x)
{

    auto sub_m = GRBModel(env);

    // z_i_u_v 变量表示航班 i 是否从登机口 u 拖行到登机口 v

    vector<vector<vector<GRBVar>>> z(Have_departue_arrival_flights_indices.size(), vector<vector<GRBVar>>(GateNumber + 1, vector<GRBVar>(GateNumber + 1)));

    for (auto i : Have_departue_arrival_flights_indices)
    {
        int idx = std::distance(Have_departue_arrival_flights_indices.begin(),
                                Have_departue_arrival_flights_indices.find(i));
        for (int u = 0; u <= GateNumber; ++u)
        {
            for (int v = 0; v <= GateNumber; ++v)
            {
                auto varName = "z_" + to_string(i) + "_" + to_string(arrival_to_departure_map[i]) + "_" + to_string(u) + "_" + to_string(v);
                z[idx][u][v] = sub_m.addVar(0.0, 1.0, TowCost[u][v], GRB_BINARY, varName);
            }
        }
    }
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
            m.addConstr(x[i][k] == 0, "LargeFlightMediumGateConstr_" + to_string(i) + "_" + to_string(k));
        }
    } // 大型飞机不能分配到中等大小登机口

    return 0;
}