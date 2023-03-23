"""CVRP with hard time windows."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


@dataclass(frozen=True)
class CVRPTWSolution:
    total_distance: float
    routes: list[list[int]]
    arrival_times: list[list[float]]


def solve_cvrptw(
    distance_matrix: np.ndarray,
    time_matrix: np.ndarray,
    demand: np.ndarray,
    tw_start: np.ndarray,
    tw_end: np.ndarray,
    service_time: np.ndarray,
    capacity: float,
    n_vehicles: int = 5,
    depot: int = 0,
    time_limit_s: int = 30,
) -> CVRPTWSolution:
    n = distance_matrix.shape[0]
    scale = 1000

    manager = pywrapcp.RoutingIndexManager(n, n_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)

    dist_cb = routing.RegisterTransitCallback(
        lambda i, j: int(distance_matrix[manager.IndexToNode(i), manager.IndexToNode(j)] * scale)
    )
    routing.SetArcCostEvaluatorOfAllVehicles(dist_cb)

    time_cb = routing.RegisterTransitCallback(
        lambda i, j: int(time_matrix[manager.IndexToNode(i), manager.IndexToNode(j)]
                          + service_time[manager.IndexToNode(j)])
    )
    routing.AddDimension(time_cb, 30, int(tw_end.max()) + 1, False, "Time")
    time_dim = routing.GetDimensionOrDie("Time")
    for node in range(n):
        idx = manager.NodeToIndex(node)
        time_dim.CumulVar(idx).SetRange(int(tw_start[node]), int(tw_end[node]))

    demand_cb = routing.RegisterUnaryTransitCallback(
        lambda i: int(demand[manager.IndexToNode(i)] * scale)
    )
    routing.AddDimensionWithVehicleCapacity(
        demand_cb, 0, [int(capacity * scale)] * n_vehicles, True, "Capacity"
    )

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.seconds = time_limit_s

    solution = routing.SolveWithParameters(params)
    if solution is None:
        return CVRPTWSolution(total_distance=float("inf"), routes=[], arrival_times=[])

    routes: list[list[int]] = []
    arrivals: list[list[float]] = []
    for v in range(n_vehicles):
        idx = routing.Start(v)
        r = [manager.IndexToNode(idx)]
        arr = [solution.Min(time_dim.CumulVar(idx))]
        while not routing.IsEnd(idx):
            idx = solution.Value(routing.NextVar(idx))
            r.append(manager.IndexToNode(idx))
            arr.append(solution.Min(time_dim.CumulVar(idx)))
        if len(r) > 2:
            routes.append(r)
            arrivals.append([float(a) for a in arr])
    return CVRPTWSolution(
        total_distance=solution.ObjectiveValue() / scale,
        routes=routes,
        arrival_times=arrivals,
    )
