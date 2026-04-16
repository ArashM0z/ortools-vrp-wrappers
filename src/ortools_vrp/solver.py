"""Problem-aware OR-Tools VRP dispatcher.

Single entrypoint that picks the right per-problem solver based on which
fields the ``Instance`` dict carries. Each per-problem solver
(``cvrp``, ``cvrptw``, ``hf_vrptw``) keeps its strict positional signature;
this layer normalises everything into a uniform :class:`SolveResult`.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

import numpy as np

from ortools_vrp.cvrp import solve_cvrp
from ortools_vrp.cvrptw import solve_cvrptw
from ortools_vrp.hf_vrptw import solve_hf_vrptw


ProblemKind = Literal["cvrp", "cvrptw", "hf_vrptw"]


@dataclass(frozen=True)
class SolveResult:
    problem: ProblemKind
    objective: float
    routes: list[list[int]]
    used_vehicles: int


_REQUIRED_BASE = ("distance_matrix", "demand")


def detect_kind(instance: dict[str, Any]) -> ProblemKind:
    """Pick the smallest OR-Tools assembly that fits the instance."""
    has_tw = "time_windows" in instance and instance["time_windows"] is not None
    caps = instance.get("vehicle_capacities")
    heterogeneous = bool(caps and len(set(caps)) > 1)
    if has_tw and heterogeneous:
        return "hf_vrptw"
    if has_tw:
        return "cvrptw"
    return "cvrp"


def _validate(instance: dict[str, Any]) -> None:
    missing = [k for k in _REQUIRED_BASE if k not in instance]
    if missing:
        raise KeyError(f"instance is missing required keys: {missing}")


def _unpack_tw(instance: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    tw = np.asarray(instance["time_windows"], dtype=float)
    if tw.ndim != 2 or tw.shape[1] != 2:
        raise ValueError("time_windows must have shape (N, 2): [tw_start, tw_end]")
    return tw[:, 0], tw[:, 1]


def solve(instance: dict[str, Any], *,
          time_limit_seconds: int = 30,
          n_vehicles: int | None = None,
          depot: int = 0) -> SolveResult:
    """Dispatch to the appropriate per-problem solver.

    Required keys (all variants):
        distance_matrix : (N, N) array
        demand          : (N,) array

    CVRP:
        capacity        : float
    CVRPTW (adds):
        time_matrix     : (N, N) array
        time_windows    : (N, 2) array of [tw_start, tw_end]
        service_time    : (N,) array
        capacity        : float
    HF-VRPTW (CVRPTW + heterogeneous fleet, no ``capacity``, instead):
        vehicle_capacities       : list[float]
        vehicle_cost_per_km      : list[float]
        vehicle_fixed_costs      : list[float]
    """
    _validate(instance)
    kind = detect_kind(instance)
    dm = np.asarray(instance["distance_matrix"], dtype=float)
    demand = np.asarray(instance["demand"], dtype=float)

    if kind == "cvrp":
        out = solve_cvrp(
            dm, demand, capacity=float(instance["capacity"]),
            n_vehicles=n_vehicles or 5, depot=depot,
            time_limit_s=time_limit_seconds,
        )
        return SolveResult(
            problem=kind, objective=float(out.total_distance),
            routes=out.routes, used_vehicles=len(out.routes),
        )

    tw_start, tw_end = _unpack_tw(instance)
    service_time = np.asarray(instance.get("service_time", np.zeros_like(demand)),
                              dtype=float)

    if kind == "cvrptw":
        tm = np.asarray(instance.get("time_matrix", dm), dtype=float)
        out = solve_cvrptw(
            dm, tm, demand, tw_start, tw_end, service_time,
            capacity=float(instance["capacity"]),
            n_vehicles=n_vehicles or 5, depot=depot,
            time_limit_s=time_limit_seconds,
        )
        return SolveResult(
            problem=kind, objective=float(out.total_distance),
            routes=out.routes, used_vehicles=len(out.routes),
        )

    # hf_vrptw
    caps = list(instance["vehicle_capacities"])
    cpkm = list(instance["vehicle_cost_per_km"])
    fixed = list(instance["vehicle_fixed_costs"])
    out = solve_hf_vrptw(
        dm, demand, tw_start, tw_end, service_time,
        vehicle_capacities=caps,
        vehicle_cost_per_km=cpkm,
        vehicle_fixed_costs=fixed,
        time_limit_s=time_limit_seconds,
    )
    return SolveResult(
        problem=kind, objective=float(out.total_cost),
        routes=out.routes, used_vehicles=len(out.routes),
    )


__all__ = ["ProblemKind", "SolveResult", "detect_kind", "solve"]
