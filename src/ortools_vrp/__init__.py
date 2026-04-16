"""Pythonic wrappers around OR-Tools VRP solvers."""
from ortools_vrp.cvrp import solve_cvrp
from ortools_vrp.cvrptw import solve_cvrptw
from ortools_vrp.hf_vrptw import solve_hf_vrptw
from ortools_vrp.solver import ProblemKind, SolveResult, detect_kind, solve

__all__ = [
    "solve_cvrp", "solve_cvrptw", "solve_hf_vrptw",
    "solve", "detect_kind", "ProblemKind", "SolveResult",
]
