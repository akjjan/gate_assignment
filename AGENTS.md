# Agent Guidelines for gate_assignment

This is a C++20 project using xmake build system and Gurobi optimizer for solving airport gate assignment problems with Benders decomposition.

## Build Commands

```bash
# Build the project
xmake

# Build with specific configuration
xmake f -p windows -a x64 -m release
xmake build

# Run the executable
xmake run

# Clean build artifacts
xmake clean
```

Note: This project does not have unit tests. The only runnable target is `gate_assignment`.

## Code Style

### Formatting
- Use tabs for indentation (2-4 spaces equivalent)
- Braces on same line as control statements
- One line between logical code sections

### Naming Conventions
- **Types/Structs**: PascalCase (e.g., `Flight`, `Solver`, `caseData`)
- **Functions/Variables**: snake_case (e.g., `calculateTowCost`, `flight_number`)
- **Class Members**: trailing underscore with snake_case (e.g., `env_`, `d_`, `x_`)
- **Constants**: SCREAMING_SNAKE_CASE (e.g., `BIG_M`)
- **Enums**: PascalCase with PascalCase values

### Imports
- Use `using std::` for frequently used STL types (map, vector, string, etc.)
- Group STL includes: `<algorithm>`, `<iostream>`, `<map>`, `<string>`, `<unordered_map>`, `<unordered_set>`, `<vector>`
- Include Gurobi headers: `"gurobi_c++.h"`, `"gurobi_c.h"`
- Include project headers: `"case_data.hpp"`, `"modelDef.hpp"`

### Types
- Prefer `int` for indices and counts
- Use `double` for floating-point values
- Use explicit types rather than `auto` except for Gurobi variable returns
- Use `std::vector` for dynamic arrays
- Use `std::map` for key-value mappings
- Use `std::unordered_set` with custom hash for sparse data structures

### Error Handling
- Wrap Gurobi operations in try-catch blocks catching `GRBException`
- Use `std::cerr` for error output
- Check optimization status with `GRB_IntAttr_Status` after `optimize()`

### Architecture Patterns
- Solver class encapsulates Gurobi model and variables
- Decision variables stored as `vector<vector<GRBVar>>` or `std::unordered_map`
- Use custom hash structs for sparse decision variables (see `yKey`, `zKey`)
- Prefer member variables over globals

### Code Practices
- Add constraint names using `to_string(i)` for debugging
- Use meaningful variable names (avoid single letters except for indices)
- Comment constraint equations with their mathematical form
- Initialize Gurobi env with log file: `env.set("LogFile", "filename.log")`
- Set model sense: `model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE)` or `GRB_MAXIMIZE`

### Gurobi Specific
- Binary variables: `GRB_BINARY`
- Continuous variables: `GRB_CONTINUOUS`
- Access variable values: `var.get(GRB_DoubleAttr_X)`
- Access objective: `model.get(GRB_DoubleAttr_ObjVal)`
- Access status: `model.get(GRB_IntAttr_Status)` - check for `GRB_OPTIMAL`, `GRB_INFEASIBLE`