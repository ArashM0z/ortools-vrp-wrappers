import numpy as np
from ortools_vrp.cvrp import solve_cvrp


def test_solve_small_cvrp() -> None:
    rng = np.random.default_rng(0)
    coords = rng.uniform(0, 1, size=(11, 2))
    coords[0] = 0.5
    dist = np.linalg.norm(coords[:, None] - coords[None], axis=-1)
    demand = rng.uniform(0.1, 0.3, size=10)
    sol = solve_cvrp(dist, demand, capacity=1.0, time_limit_s=3)
    assert sol.total_distance > 0
    assert len(sol.routes) >= 1
    for r in sol.routes:
        assert r[0] == 0 and r[-1] == 0
