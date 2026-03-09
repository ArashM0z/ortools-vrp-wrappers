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

<!-- 2023-05 -->

<!-- 2023-08 -->

<!-- 2023-10 -->

<!-- maint 2025-01-22 -->

<!-- maint 2025-03-01 -->

<!-- maint 2025-04-10 -->

<!-- maint 2025-05-18 -->

<!-- maint 2025-06-26 -->

<!-- maint 2025-08-04 -->

<!-- maint 2025-09-12 -->

<!-- maint 2025-10-22 -->

<!-- maint 2025-11-29 -->

<!-- maint 2024-01-28 -->

<!-- maint 2024-03-21 -->

<!-- maint 2024-05-12 -->

<!-- maint 2024-07-02 -->

<!-- maint 2024-08-23 -->

<!-- maint 2024-10-15 -->

<!-- maint 2024-12-06 -->

<!-- maint 2023-02-08 -->

<!-- maint 2023-04-14 -->

<!-- iter 2023-06-12-09 -->

<!-- iter 2023-06-12-11 -->

<!-- iter 2023-06-12-13 -->

<!-- iter 2023-06-12-15 -->

<!-- iter 2023-06-12-17 -->

<!-- iter 2023-06-12-19 -->

<!-- iter 2023-06-12-21 -->

<!-- iter 2023-11-06-09 -->

<!-- iter 2023-11-06-11 -->

<!-- iter 2023-11-06-13 -->

<!-- iter 2023-11-06-15 -->

<!-- iter 2023-11-06-17 -->

<!-- iter 2023-11-06-19 -->

<!-- iter 2024-03-04-09 -->

<!-- iter 2024-03-04-11 -->

<!-- iter 2024-03-04-13 -->

<!-- iter 2024-03-04-15 -->

<!-- iter 2024-03-04-17 -->

<!-- iter 2024-03-04-19 -->

<!-- iter 2024-09-16-09 -->

<!-- iter 2024-09-16-11 -->

<!-- iter 2024-09-16-13 -->

<!-- iter 2024-09-16-15 -->

<!-- iter 2024-09-16-17 -->

<!-- iter 2024-09-16-19 -->

<!-- iter 2024-09-16-21 -->

<!-- iter 2026-03-09-09 -->

<!-- iter 2026-03-09-11 -->

<!-- iter 2026-03-09-13 -->
