"""Heterogeneous-fleet VRPTW solver."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


@dataclass(frozen=True)
class HFVRPTWSolution:
    total_cost: float
    routes: list[list[int]]
    vehicle_types: list[int]


def solve_hf_vrptw(
    distance_matrix: np.ndarray,
    demand: np.ndarray,
    tw_start: np.ndarray,
    tw_end: np.ndarray,
    service_time: np.ndarray,
    vehicle_capacities: list[float],
    vehicle_cost_per_km: list[float],
    vehicle_fixed_costs: list[float],
    time_limit_s: int = 60,
) -> HFVRPTWSolution:
    n = distance_matrix.shape[0]
    n_vehicles = len(vehicle_capacities)
    scale = 1000

    manager = pywrapcp.RoutingIndexManager(n, n_vehicles, 0)
    routing = pywrapcp.RoutingModel(manager)

    for v in range(n_vehicles):
        cb = routing.RegisterTransitCallback(
            lambda i, j, vv=v: int(
                distance_matrix[manager.IndexToNode(i), manager.IndexToNode(j)]
                * vehicle_cost_per_km[vv] * scale
            )
        )
        routing.SetArcCostEvaluatorOfVehicle(cb, v)
        routing.SetFixedCostOfVehicle(int(vehicle_fixed_costs[v] * scale), v)

    time_cb = routing.RegisterTransitCallback(
        lambda i, j: int(distance_matrix[manager.IndexToNode(i), manager.IndexToNode(j)]
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
        demand_cb, 0, [int(c * scale) for c in vehicle_capacities], True, "Capacity"
    )

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.seconds = time_limit_s

    solution = routing.SolveWithParameters(params)
    if solution is None:
        return HFVRPTWSolution(total_cost=float("inf"), routes=[], vehicle_types=[])

    routes: list[list[int]] = []
    types: list[int] = []
    for v in range(n_vehicles):
        idx = routing.Start(v)
        r = [manager.IndexToNode(idx)]
        while not routing.IsEnd(idx):
            idx = solution.Value(routing.NextVar(idx))
            r.append(manager.IndexToNode(idx))
        if len(r) > 2:
            routes.append(r)
            types.append(v)
    return HFVRPTWSolution(
        total_cost=solution.ObjectiveValue() / scale,
        routes=routes,
        vehicle_types=types,
    )
