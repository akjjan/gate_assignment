/*
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "hello world!" << std::endl;
    return 0;
}
*/
/* Copyright 2025, Gurobi Optimization, LLC */

/* This example formulates and solves the following simple MIP model:

     maximize    x +   y + 2 z
     subject to  x + 2 y + 3 z <= 4
                 x +   y       >= 1
                 x, y, z binary
*/
//  这是一个用于测试 Gurobi C++ 接口的简单 MIP 模型示例。111
#include "gurobi_c++.h"

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
    try
    {

        // 创建一个环境，如果参数为true，则表示启用日志文件
        auto env = GRBEnv(true);
        env.set("LogFile", "mip1.log");
        env.start();

        // 创建一个空模型
        auto model = GRBModel(env);

        // 创建变量
        auto x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
        auto y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
        auto z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");

        // 设置目标函数：最大化 x + y + 2 z
        model.setObjective(x + y + 2 * z, GRB_MAXIMIZE);

        // 约束: x + 2 y + 3 z <= 4
        model.addConstr(x + 2 * y + 3 * z <= 4, "c0");

        // 约束: x + y >= 1
        model.addConstr(x + y >= 1, "c1");

        // 优化模型
        model.optimize();

        cout << "-------------------------------" << endl;

        cout << x.get(GRB_StringAttr_VarName) << " "
             << x.get(GRB_DoubleAttr_X) << endl;
        cout << y.get(GRB_StringAttr_VarName) << " "
             << y.get(GRB_DoubleAttr_X) << endl;
        cout << z.get(GRB_StringAttr_VarName) << " "
             << z.get(GRB_DoubleAttr_X) << endl;

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
    }
    catch (GRBException e)
    {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    catch (...)
    {
        cout << "Exception during optimization" << endl;
    }

    return 0;
}