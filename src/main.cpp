#include "gurobi_c++.h"
#include <iostream>
#include <string>
#include <vector>

enum FlightType
{
    ARRIVAL,
    DEPARTURE
}; // 枚举类型表示航班类型

struct flight
{
    int flight_id;
    FlightType flight_type;
    int scheduled_time;
}; // 航班结构体, 包含航班ID、类型和计划时间

const int FlightNumber = 500; // 全局变量，航班数量
const int GateNumber = 50;    // 全局变量，登机口数量

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
            x[i][j] = m.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName);
        }
    }

    return 0;
}