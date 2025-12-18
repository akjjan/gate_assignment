#pragma once
#include <cstddef>
#include <functional>

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
    FlightType flight_type;     // 到达/离开类型
    AircraftSize aircraft_size; // 飞机大小
    int scheduled_time;         // 计划时间
};

bool operator<(const Flight &a, const Flight &b)
{
    return a.scheduled_time < b.scheduled_time;
}

struct successionKey
{
    int i, j, k;
    bool operator==(const successionKey &other) const
    {
        return i == other.i && j == other.j && k == other.k;
    }
};

struct successionKeyHash
{
    std::size_t operator()(const successionKey &key) const
    {
        return std::hash<int>()(key.i) ^ std::hash<int>()(key.j) ^ std::hash<int>()(key.k);
    }
};