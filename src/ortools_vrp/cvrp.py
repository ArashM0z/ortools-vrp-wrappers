"""Capacitated VRP solver."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


@dataclass(frozen=True)
class CVRPSolution:
    total_distance: float
    routes: list[list[int]]
    solve_time_s: float


def solve_cvrp(
    distance_matrix: np.ndarray,
    demand: np.ndarray,
    capacity: float,
    n_vehicles: int = 5,
    depot: int = 0,
    time_limit_s: int = 10,
    scale: int = 1000,
) -> CVRPSolution:
    import time
    n = distance_matrix.shape[0]
    scaled_dist = (distance_matrix * scale).astype(int)
    scaled_demand = np.concatenate([[0], (demand * scale).astype(int)])[: n]

    manager = pywrapcp.RoutingIndexManager(n, n_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)

    transit_cb = routing.RegisterTransitCallback(
        lambda i, j: int(scaled_dist[manager.IndexToNode(i), manager.IndexToNode(j)])
    )
    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb)

    demand_cb = routing.RegisterUnaryTransitCallback(
        lambda i: int(scaled_demand[manager.IndexToNode(i)])
    )
    routing.AddDimensionWithVehicleCapacity(
        demand_cb, 0, [int(capacity * scale)] * n_vehicles, True, "Capacity"
    )

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.seconds = time_limit_s

    start = time.monotonic()
    solution = routing.SolveWithParameters(params)
    elapsed = time.monotonic() - start
    if solution is None:
        return CVRPSolution(total_distance=float("inf"), routes=[], solve_time_s=elapsed)

    routes: list[list[int]] = []
    for v in range(n_vehicles):
        idx = routing.Start(v)
        r = [manager.IndexToNode(idx)]
        while not routing.IsEnd(idx):
            idx = solution.Value(routing.NextVar(idx))
            r.append(manager.IndexToNode(idx))
        if len(r) > 2:
            routes.append(r)
    return CVRPSolution(
        total_distance=solution.ObjectiveValue() / scale,
        routes=routes,
        solve_time_s=elapsed,
    )
