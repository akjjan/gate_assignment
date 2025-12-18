#include "gurobi_c++.h"
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>

using std::vector;

enum FlightType
{
    ARRIVAL,
    DEPARTURE
}; // 枚举类型表示航班类型

struct Flight
{
    int flight_id;          // 航班ID
    FlightType flight_type; // 类型
    int scheduled_time;     // 计划时间
};

const int FlightNumber = 500; // 全局变量，航班数量

const int GateNumber = 50; // 全局变量，登机口数量

std::vector<double> ApronPenaltyCost(FlightNumber, 100.0); // 每个航班的停机坪惩罚成本

std::vector<Flight> flights(FlightNumber); // 航班信息列表

std::set<int> No_departue_arrival_flights_indices;   // \underline{F_a}
std::set<int> Have_departue_arrival_flights_indices; // \overline{F_a}
// 分别存储到达后不离开和有离开的  到达航班索引

std::set<int> Departure_flights_indices;
// 离开航班索引  F_d

std::map<int, int> arrival_to_departure_map;
//  \delta 映射 ： 到达航班ID到离开航班ID的映射

std::vector<std::vector<double>> TowCost(GateNumber + 1, std::vector<double>(GateNumber + 1, 50.0));
// 拖行成本

int main()
{
    auto env = GRBEnv(true);
    env.set("LogFile", "gate_assignment.log");
    env.start();

    auto m = GRBModel(env);

    std::vector<std::vector<GRBVar>> x(FlightNumber, std::vector<GRBVar>(GateNumber + 1));
    // 创建决策变量 x[i][j]，表示航班 i 是否分配到登机口 j , x[i][GateNumber] 表示航班 i 未分配登机口，而是停机坪

    for (int i = 0; i < FlightNumber; ++i)
    {
        for (int j = 0; j <= GateNumber; ++j)
        {
            std::string varName = "x_" + std::to_string(i) + "_" + std::to_string(j);
            if (j < GateNumber)
                x[i][j] = m.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
            else
                x[i][j] = m.addVar(0.0, 1.0, ApronPenaltyCost[i], GRB_BINARY, varName); // 航班的停机坪惩罚成本
        }
    }

    for (auto i : No_departue_arrival_flights_indices)
    {
        for (int k = 0; k < GateNumber; ++k)
        {
            x[i][k].set(GRB_DoubleAttr_Obj, TowCost[k][GateNumber]);
        }
    } // 对于到达后不离开的航班，设置拖行成本

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
                auto varName = "z_" + std::to_string(i) + "_" + std::to_string(arrival_to_departure_map[i]) + "-" + std::to_string(u) + "_" + std::to_string(v);
                z[idx][u][v] = m.addVar(0.0, 1.0, TowCost[u][v], GRB_BINARY, varName);
            }
        }
    }

    return 0;
}