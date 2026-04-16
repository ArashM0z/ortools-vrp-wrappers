"""Tests for the problem-aware dispatcher (no OR-Tools call required)."""
import numpy as np
import pytest

from ortools_vrp.solver import detect_kind


def test_detect_cvrp_without_time_windows():
    inst = {"distance_matrix": np.eye(3), "demand": np.ones(3), "capacity": 10.0}
    assert detect_kind(inst) == "cvrp"


def test_detect_cvrptw_with_time_windows_homogeneous_fleet():
    inst = {
        "distance_matrix": np.eye(3), "demand": np.ones(3),
        "time_windows": [[0, 100], [10, 50], [20, 80]],
        "capacity": 10.0,
    }
    assert detect_kind(inst) == "cvrptw"


def test_detect_hf_vrptw_when_capacities_differ():
    inst = {
        "distance_matrix": np.eye(3), "demand": np.ones(3),
        "time_windows": [[0, 100], [10, 50], [20, 80]],
        "vehicle_capacities": [5.0, 10.0, 20.0],
    }
    assert detect_kind(inst) == "hf_vrptw"


def test_detect_cvrptw_when_capacities_are_homogeneous():
    inst = {
        "distance_matrix": np.eye(3), "demand": np.ones(3),
        "time_windows": [[0, 100], [10, 50], [20, 80]],
        "vehicle_capacities": [10.0, 10.0, 10.0],
        "capacity": 10.0,
    }
    assert detect_kind(inst) == "cvrptw"


def test_solve_validates_required_keys():
    from ortools_vrp.solver import solve
    with pytest.raises(KeyError):
        solve({"distance_matrix": np.eye(3)})  # missing demand
