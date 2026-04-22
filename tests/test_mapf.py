# tests/test_mapf.py
import pytest
from solver.grid import Grid
from solver.mapf import Drone, MAPFSolver

def _no_vertex_conflicts(paths):
    drone_ids = list(paths.keys())
    max_t = max(len(p) for p in paths.values())
    for t in range(max_t):
        seen = {}
        for did in drone_ids:
            path = paths[did]
            pos = path[min(t, len(path) - 1)]
            assert pos not in seen, f"Vertex conflict at t={t}, pos={pos}"
            seen[pos] = did

def _no_edge_conflicts(paths):
    drone_ids = list(paths.keys())
    max_t = max(len(p) for p in paths.values())
    for t in range(max_t - 1):
        for i, a in enumerate(drone_ids):
            for b in drone_ids[i+1:]:
                pa_t  = paths[a][min(t,   len(paths[a])-1)]
                pa_t1 = paths[a][min(t+1, len(paths[a])-1)]
                pb_t  = paths[b][min(t,   len(paths[b])-1)]
                pb_t1 = paths[b][min(t+1, len(paths[b])-1)]
                assert not (pa_t == pb_t1 and pb_t == pa_t1), \
                    f"Edge conflict between drone {a} and {b} at t={t}"

def test_single_drone_reaches_goal():
    g = Grid(rows=4, cols=4)
    drones = [Drone(id=0, start=(0, 0), goal=(3, 3))]
    sol = MAPFSolver(g, drones).solve()
    assert sol.status in ("optimal", "feasible")
    assert sol.paths[0][-1] == (3, 3)

def test_two_drones_no_conflict():
    g = Grid(rows=4, cols=4)
    drones = [
        Drone(id=0, start=(0, 0), goal=(3, 3)),
        Drone(id=1, start=(3, 3), goal=(0, 0)),
    ]
    sol = MAPFSolver(g, drones).solve()
    assert sol.status in ("optimal", "feasible")
    _no_vertex_conflicts(sol.paths)
    _no_edge_conflicts(sol.paths)

def test_three_drones_no_conflict():
    g = Grid(rows=4, cols=4)
    drones = [
        Drone(id=0, start=(0, 0), goal=(0, 3)),
        Drone(id=1, start=(0, 3), goal=(3, 3)),
        Drone(id=2, start=(3, 3), goal=(0, 0)),
    ]
    sol = MAPFSolver(g, drones).solve()
    assert sol.status in ("optimal", "feasible")
    _no_vertex_conflicts(sol.paths)
    _no_edge_conflicts(sol.paths)

def test_makespan_is_optimal_single():
    g = Grid(rows=4, cols=4)
    drones = [Drone(id=0, start=(0, 0), goal=(0, 3))]
    sol = MAPFSolver(g, drones).solve()
    assert sol.makespan == 3

def test_nofly_zone_respected():
    g = Grid(rows=4, cols=4)
    g.add_nofly_box((0, 1), (0, 1))
    drones = [Drone(id=0, start=(0, 0), goal=(0, 2))]
    sol = MAPFSolver(g, drones).solve()
    assert sol.status in ("optimal", "feasible")
    for pos in sol.paths[0]:
        assert pos != (0, 1)

def test_all_drones_reach_goal():
    g = Grid(rows=5, cols=5)
    drones = [
        Drone(id=0, start=(0, 0), goal=(4, 4)),
        Drone(id=1, start=(0, 4), goal=(4, 0)),
        Drone(id=2, start=(4, 0), goal=(0, 4)),
    ]
    sol = MAPFSolver(g, drones).solve()
    assert sol.status in ("optimal", "feasible")
    for d in drones:
        assert sol.paths[d.id][-1] == d.goal
