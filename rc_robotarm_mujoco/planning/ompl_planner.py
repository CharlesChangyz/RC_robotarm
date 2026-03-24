import numpy as np
from ompl import base as ob
from ompl import geometric as og


# Joint limits confirmed from robot XML
JOINT_LIMITS = np.array([
    [-np.pi,  np.pi],   # j1 — base yaw
    [0.0,     np.pi],   # j2 — shoulder
    [0.0,     np.pi],   # j3 — elbow
    [-np.pi,  np.pi],   # j4 — wrist yaw
])


class OMPLPlanner:
    """OMPL-based path planner operating in 4D joint space.

    Uses RRTConnect by default — bidirectional search is well-suited for
    manipulation planning in low-dimensional joint spaces and reliably
    produces feasible paths faster than unidirectional planners.

    The geometric path is simplified then densely interpolated before
    returning, making it suitable for TOPP-RA time parameterization.
    """

    def __init__(
        self,
        collision_checker,
        joint_limits: np.ndarray = JOINT_LIMITS,
        planner_name: str = "RRTConnect",
        range: float = 0.1,
        solve_time: float = 5.0,
        simplify: bool = True,
        interpolate_n: int = 150,
    ) -> None:
        """
        Parameters
        ----------
        collision_checker : CollisionChecker
        joint_limits : np.ndarray, shape (4, 2)
        planner_name : str
            Any OMPL geometric planner class name, e.g. "RRTConnect", "RRT",
            "KPIECE1", "BITstar".
        range : float
            Maximum extension step in radians.
        solve_time : float
            Planning time budget in seconds.
        simplify : bool
            Run path simplification after finding a solution.
        interpolate_n : int
            Number of states after dense interpolation (for TOPP-RA input).
        """
        self._checker = collision_checker
        self._limits = joint_limits
        self._planner_name = planner_name
        self._range = range
        self._solve_time = solve_time
        self._simplify = simplify
        self._interpolate_n = interpolate_n

        # Build reusable state space
        self._space = ob.RealVectorStateSpace(4)
        bounds = ob.RealVectorBounds(4)
        for i, (lo, hi) in enumerate(joint_limits):
            bounds.setLow(i, float(lo))
            bounds.setHigh(i, float(hi))
        self._space.setBounds(bounds)

    def plan(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
    ) -> tuple:
        """Plan a collision-free path from q_start to q_goal.

        Parameters
        ----------
        q_start : np.ndarray, shape (4,)
        q_goal  : np.ndarray, shape (4,)

        Returns
        -------
        (waypoints, success) : (list of np.ndarray each shape (4,), bool)
            waypoints is empty on failure.
        """
        # Quick validity checks before handing off to OMPL
        if not self._checker.is_valid(q_start):
            print("[OMPLPlanner] Start configuration is in collision.")
            return [], False
        if not self._checker.is_valid(q_goal):
            print("[OMPLPlanner] Goal configuration is in collision.")
            return [], False

        checker = self._checker  # capture for closure

        def is_state_valid(state):
            q = np.array([state[i] for i in range(4)])
            return checker.is_valid(q)

        ss = og.SimpleSetup(self._space)
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))

        # Start state
        start = ob.State(self._space)
        for i, v in enumerate(q_start):
            start()[i] = float(v)

        # Goal state
        goal = ob.State(self._space)
        for i, v in enumerate(q_goal):
            goal()[i] = float(v)

        ss.setStartAndGoalStates(start, goal)

        # Planner selection
        planner_cls = getattr(og, self._planner_name, None)
        if planner_cls is None:
            raise ValueError(f"Unknown OMPL planner: {self._planner_name}")
        planner = planner_cls(ss.getSpaceInformation())
        if hasattr(planner, "setRange"):
            planner.setRange(self._range)
        ss.setPlanner(planner)

        # Solve
        solved = ss.solve(self._solve_time)
        if not solved:
            print("[OMPLPlanner] Planning failed: no solution found within time limit.")
            return [], False

        if self._simplify:
            ss.simplifySolution()

        path = ss.getSolutionPath()
        path.interpolate(self._interpolate_n)

        waypoints = []
        for i in range(path.getStateCount()):
            s = path.getState(i)
            waypoints.append(np.array([s[j] for j in range(4)]))

        print(f"[OMPLPlanner] Found path with {len(waypoints)} waypoints.")
        return waypoints, True
