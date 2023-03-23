# OR-Tools VRP Wrappers

Pythonic wrappers around Google OR-Tools' VRP solver covering the variants we use as baselines:

- `solve_cvrp` — capacitated VRP
- `solve_cvrptw` — CVRP with hard time windows
- `solve_hf_vrptw` — heterogeneous-fleet VRPTW with per-vehicle cost-per-km and fixed costs

Used as the classical baseline in EFECTIW-ROTER, SED2AM, and Edge-DIRECT. Returns dataclass results with the routes, total cost, and (where relevant) arrival times.

## Use

```python
import numpy as np
from ortools_vrp import solve_cvrp

dist = np.array([[0, 1, 2], [1, 0, 3], [2, 3, 0]])
demand = np.array([0.3, 0.4])
sol = solve_cvrp(dist, demand, capacity=1.0)
print(sol.routes, sol.total_distance)
```
