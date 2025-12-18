#pragma once

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