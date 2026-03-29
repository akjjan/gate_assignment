#pragma once

#include "modelDef.hpp"
#include <map>
#include <unordered_map>
#include <vector>

struct caseData {
  int flightNumber; // 航班数量
  int gateNumber;   // 登机口数量
  int apronIndex;   // 停机坪索引, = gateNumber
  int bufferTime;   // 缓冲时间

  std::vector<double> apronPenaltyCost; // 每个航班的停机坪惩罚成本
  std::vector<double> delayPenaltyCost; // 每个航班的延误惩罚成本
  std::vector<Flight> flights;          // 航班信息列表

  std::vector<int> noDepartArr;    //  \underline{F_a}  不离开的到达航班索引
  std::vector<int> haveDepartArr;  // \overline{F_a}   有离开的到达航班索引
  std::vector<int> arrivalFlights; // 到达航班索引 F_a
  std::vector<int> departFlights;  // 离开航班索引  F_d

  std::map<int, int> delta; //  \delta 映射 ： 到达航班ID -> 离开航班ID
  std::unordered_map<int, int> departure_pert; // 离开航班的扰动时间映射

  std::vector<int> mediumGates;  // 中等大小登机口集合
  std::vector<int> largeFlights; // 大型飞机航班集合

  std::vector<std::vector<double>> towCost; // 拖行成本
  std::vector<std::vector<double>> towTime; // 拖行时间

  std::map<int, Flight> flightMap; // 航班ID 到 航班信息 的映射
};

inline caseData makeCaseData(
    int flightNumber, int gateNumber, int bufferTime,
    const std::vector<double> &apronPenaltyCost,
    const std::vector<double> &delayPenaltyCost,
    const std::vector<Flight> &flights, const std::vector<int> &noDepartArr,
    const std::vector<int> &haveDepartArr,
    const std::vector<int> &departFlights, const std::map<int, int> &delta,
    const std::vector<int> &mediumGates, const std::vector<int> &largeFlights,
    const std::vector<std::vector<double>> &towCost,
    const std::vector<std::vector<double>> &towTime,
    const std::unordered_map<int, int> &departure_pert) {
  caseData data;

  data.flightNumber = flightNumber;
  data.gateNumber = gateNumber;
  data.apronIndex = gateNumber; // 停机坪索引等于登机口数量
  data.bufferTime = bufferTime;
  data.apronPenaltyCost = apronPenaltyCost;
  data.delayPenaltyCost = delayPenaltyCost;
  data.flights = flights;
  data.noDepartArr = noDepartArr;
  data.haveDepartArr = haveDepartArr;
  data.departFlights = departFlights;
  data.delta = delta;
  data.mediumGates = mediumGates;
  data.largeFlights = largeFlights;
  data.towCost = towCost;
  data.towTime = towTime;
  data.departure_pert = departure_pert;

  std::vector<int> arrivalFlights = noDepartArr;
  arrivalFlights.insert(arrivalFlights.end(), haveDepartArr.begin(),
                        haveDepartArr.end());
  data.arrivalFlights = arrivalFlights;

  for (const Flight &flight : flights) {
    data.flightMap[flight.flight_id] = flight;
  }

  return data;
}