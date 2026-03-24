import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo


# Default joint limits for rc_arm_2 (conservative for competition use)
DEFAULT_VEL_LIMITS = np.array([3.0, 2.0, 2.0, 4.0])    # rad/s per joint
DEFAULT_ACC_LIMITS = np.array([6.0, 4.0, 4.0, 8.0])    # rad/s² per joint


class TrajectoryResult:
    """Time-parameterized joint trajectory produced by TOPPRAParameterizer."""

    def __init__(self, jnt_traj, duration: float) -> None:
        self._traj = jnt_traj
        self.duration = duration

    def query(self, t: float) -> tuple:
        """Query trajectory at time t.

        Parameters
        ----------
        t : float
            Time in seconds (clamped to [0, duration]).

        Returns
        -------
        (q, dq) : (np.ndarray shape (4,), np.ndarray shape (4,))
        """
        t = float(np.clip(t, 0.0, self.duration))
        q = np.asarray(self._traj(t)).flatten()
        dq = np.asarray(self._traj(t, 1)).flatten()
        return q, dq

    def is_finished(self, t: float) -> bool:
        return float(t) >= self.duration


class TOPPRAParameterizer:
    """Time-optimal path parameterization via TOPP-RA.

    Converts a geometric joint-space path (list of waypoints from OMPL) into
    a time-parameterized trajectory satisfying joint velocity and acceleration
    constraints.  Falls back to a simple linear-speed trajectory if TOPP-RA
    fails.
    """

    def __init__(
        self,
        vel_limits: np.ndarray = DEFAULT_VEL_LIMITS,
        acc_limits: np.ndarray = DEFAULT_ACC_LIMITS,
    ) -> None:
        """
        Parameters
        ----------
        vel_limits : np.ndarray, shape (4,)  — max |velocity| per joint (rad/s)
        acc_limits : np.ndarray, shape (4,)  — max |acceleration| per joint (rad/s²)
        """
        self._vlim = np.stack([-vel_limits, vel_limits], axis=1)   # (4, 2)
        self._alim = np.stack([-acc_limits, acc_limits], axis=1)   # (4, 2)

    def parameterize(self, waypoints: list) -> TrajectoryResult:
        """Parameterize a geometric path.

        Parameters
        ----------
        waypoints : list of np.ndarray each shape (4,)
            At least 2 waypoints required.

        Returns
        -------
        TrajectoryResult
        """
        Q = self._deduplicate(np.array(waypoints))
        if len(Q) < 2:
            # Path is trivially short — stay at the single configuration
            single = Q[0] if len(Q) == 1 else waypoints[0]
            return self._constant_trajectory(single)

        times = np.linspace(0.0, 1.0, len(Q))
        path = ta.SplineInterpolator(times, Q)

        pc_vel = constraint.JointVelocityConstraint(self._vlim)
        pc_acc = constraint.JointAccelerationConstraint(self._alim)

        try:
            instance = algo.TOPPRA([pc_vel, pc_acc], path)
            jnt_traj = instance.compute_trajectory()
            if jnt_traj is None:
                raise RuntimeError("TOPP-RA returned None")
            print(f"[TOPPRAParameterizer] Trajectory duration: {jnt_traj.duration:.3f} s")
            return TrajectoryResult(jnt_traj, jnt_traj.duration)
        except Exception as exc:
            print(f"[TOPPRAParameterizer] TOPP-RA failed ({exc}), using linear fallback.")
            return self._linear_fallback(Q)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _deduplicate(Q: np.ndarray, tol: float = 1e-6) -> np.ndarray:
        """Remove consecutive duplicate waypoints."""
        keep = [Q[0]]
        for i in range(1, len(Q)):
            if np.linalg.norm(Q[i] - keep[-1]) > tol:
                keep.append(Q[i])
        return np.array(keep)

    def _linear_fallback(self, Q: np.ndarray) -> TrajectoryResult:
        """Simple linear interpolation respecting velocity limits."""
        total_dist = sum(np.linalg.norm(Q[i + 1] - Q[i]) for i in range(len(Q) - 1))
        if total_dist < 1e-9:
            return self._constant_trajectory(Q[0])
        # duration from slowest-joint velocity limit
        vmax = float(np.min(self._vlim[:, 1]))
        duration = max(total_dist / vmax, 0.1)
        times = np.linspace(0.0, 1.0, len(Q))
        path = ta.SplineInterpolator(times, Q)
        # Build a uniform-speed trajectory wrapper
        return _LinearTrajectoryResult(path, duration)

    @staticmethod
    def _constant_trajectory(q: np.ndarray) -> "TrajectoryResult":
        """Trivial trajectory: stay at q for 0.1 s."""
        Q = np.stack([q, q])
        times = np.array([0.0, 1.0])
        path = ta.SplineInterpolator(times, Q)
        # Wrap as a constant trajectory
        return _LinearTrajectoryResult(path, 0.1)


class _LinearTrajectoryResult:
    """Uniform-speed fallback trajectory that wraps a toppra SplineInterpolator."""

    def __init__(self, path, duration: float) -> None:
        self._path = path
        self.duration = duration

    def query(self, t: float) -> tuple:
        t = float(np.clip(t, 0.0, self.duration))
        s = t / self.duration          # normalised in [0, 1]
        q = np.asarray(self._path(s)).flatten()
        dq = np.asarray(self._path(s, 1)).flatten() / self.duration
        return q, dq

    def is_finished(self, t: float) -> bool:
        return float(t) >= self.duration
