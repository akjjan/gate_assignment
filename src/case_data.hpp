#pragma once

#include "modelDef.hpp"
#include <map>
#include <vector>

using std::map;
using std::vector;

struct caseData {
  int flightNumber; // 航班数量
  int gateNumber;   // 登机口数量
  int apronIndex;   // 停机坪索引, = gateNumber
  int bufferTime;   // 缓冲时间

  vector<double> apronPenaltyCost; // 每个航班的停机坪惩罚成本
  vector<Flight> flights;          // 航班信息列表

  vector<int> noDepartArr;   //  \underline{F_a}  不离开的到达航班索引
  vector<int> haveDepartArr; // \overline{F_a}   有离开的到达航班索引
  vector<int> departFlights; // 离开航班索引  F_d

  map<int, int> delta; //  \delta 映射 ： 到达航班ID -> 离开航班ID

  vector<int> mediumGates;  // 中等大小登机口集合
  vector<int> largeFlights; // 大型飞机航班集合

  vector<vector<double>> towCost; // 拖行成本
  vector<vector<double>> towTime; // 拖行时间

  map<int, Flight> flightMap; // 航班ID 到 航班信息 的映射
};

inline caseData
makeCaseData(int flightNumber, int gateNumber, int bufferTime,
             const vector<double> &apronPenaltyCost,
             const vector<Flight> &flights, const vector<int> &noDepartArr,
             const vector<int> &haveDepartArr, const vector<int> &departFlights,
             const map<int, int> &delta, const vector<int> &mediumGates,
             const vector<int> &largeFlights,
             const vector<vector<double>> &towCost,
             const vector<vector<double>> &towTime) {
  caseData data;

  data.flightNumber = flightNumber;
  data.gateNumber = gateNumber;
  data.apronIndex = gateNumber; // 停机坪索引等于登机口数量
  data.bufferTime = bufferTime;
  data.apronPenaltyCost = apronPenaltyCost;
  data.flights = flights;
  data.noDepartArr = noDepartArr;
  data.haveDepartArr = haveDepartArr;
  data.departFlights = departFlights;
  data.delta = delta;
  data.mediumGates = mediumGates;
  data.largeFlights = largeFlights;
  data.towCost = towCost;
  data.towTime = towTime;

  for (const auto &flight : flights) {
    data.flightMap[flight.flight_id] = flight;
  }

  return data;
}