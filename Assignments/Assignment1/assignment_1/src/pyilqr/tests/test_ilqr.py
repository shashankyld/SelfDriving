import numpy as np

from ..pyilqr.costs import QuadraticCost
from ..pyilqr.example_costs import SetpointTrackingCost
from ..pyilqr.example_dynamics import UnicycleDynamics, BicycleDynamics
from ..pyilqr.ocp import OptimalControlProblem
from ..pyilqr.ilqr import ILQRSolver
from ..pyilqr.strategies import FunctionStrategy


def test_ilqr_unicycle():
    dynamics = UnicycleDynamics(0.1)
    horizon = 100
    x0 = np.array([0, 0, -0.3, 0.1])
    state_cost = SetpointTrackingCost(np.eye(4), x_target=np.array([1, 1, 0, 0]))
    input_cost = QuadraticCost(np.eye(2), np.zeros(2))
    ocp = OptimalControlProblem(dynamics, state_cost, input_cost, horizon)
    solver = ILQRSolver(ocp)

    initial_strategy = FunctionStrategy(lambda x, t: np.array([0, 0]))
    initial_xs, initial_us, _ = dynamics.rollout(x0, initial_strategy, horizon)
    initial_cost = state_cost.trajectory_cost(initial_xs) + input_cost.trajectory_cost(
        initial_us
    )

    xs, us, has_converged = solver.solve(x0, initial_strategy)

    assert has_converged
    assert (
        state_cost.trajectory_cost(xs) + input_cost.trajectory_cost(us) < initial_cost
    )


def test_ilqr_bicycle():
    dynamics = BicycleDynamics(0.1)
    horizon = 100
    x0 = np.array([0, 0, -0.3, 0.1, 0])
    state_cost = SetpointTrackingCost(np.eye(5), x_target=np.array([1, 1, 0, 0, 0]))
    input_cost = QuadraticCost(np.eye(2), np.zeros(2))
    ocp = OptimalControlProblem(dynamics, state_cost, input_cost, horizon)
    solver = ILQRSolver(ocp)

    initial_strategy = FunctionStrategy(lambda x, t: np.array([0, 0]))
    initial_xs, initial_us, _ = dynamics.rollout(x0, initial_strategy, horizon)
    initial_cost = state_cost.trajectory_cost(initial_xs) + input_cost.trajectory_cost(
        initial_us
    )

    xs, us, has_converged = solver.solve(x0, initial_strategy)

    assert has_converged
    assert (
        state_cost.trajectory_cost(xs) + input_cost.trajectory_cost(us) < initial_cost
    )
