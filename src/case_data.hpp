#pragma once

#include "modelDef.hpp"
#include <vector>
#include <map>

using std::map;
using std::vector;

struct caseData
{
    int flightNumber; // 航班数量
    int gateNumber;   // 登机口数量
    int apronIndex;   // 停机坪索引, = gateNumber

    vector<double> apronPenaltyCost; // 每个航班的停机坪惩罚成本
    vector<Flight> flights;          // 航班信息列表

    vector<int> noDepartArr;   //  \underline{F_a}  不离开的到达航班索引
    vector<int> haveDepartArr; // \overline{F_a}   有离开的到达航班索引
    vector<int> departFlights; // 离开航班索引  F_d

    map<int, int> delta; //  \delta 映射 ： 到达航班ID -> 离开航班ID

    vector<int> mediumGates;  // 中等大小登机口集合
    vector<int> largeFlights; // 大型飞机航班集合

    vector<vector<double>> towCost; // 拖行成本
};

inline caseData makeCaseData(int flightNum, int gateNum,
                             const vector<double> &apronPenalty,
                             const vector<Flight> &flightList,
                             const vector<int> &noDepartArr,
                             const vector<int> &haveDepartArr,
                             const vector<int> &departFlights,
                             const map<int, int> &deltaMap,
                             const vector<int> &mediumGates,
                             const vector<int> &largeFlights,
                             const vector<vector<double>> &towCostMatrix)
{
    caseData data;
    data.flightNumber = flightNum;
    data.gateNumber = gateNum;
    data.apronIndex = gateNum;
    data.apronPenaltyCost = apronPenalty;
    data.flights = flightList;
    data.noDepartArr = noDepartArr;
    data.haveDepartArr = haveDepartArr;
    data.departFlights = departFlights;
    data.delta = deltaMap;
    data.mediumGates = mediumGates;
    data.largeFlights = largeFlights;
    data.towCost = towCostMatrix;
    return data;
}