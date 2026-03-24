import numpy as np


class TrajectoryExecutor:
    """Step-by-step trajectory execution interface.

    Converts time queries into EEF poses by:
      1. Querying q(t) from a TrajectoryResult
      2. Computing FK → 7D Cartesian pose via FKSolver

    The caller (main loop / demo) provides wall-clock time relative to when
    the trajectory started.  The executor clamps t to [0, duration] so the
    arm holds at the final pose after completion.
    """

    def __init__(self, fk_solver) -> None:
        """
        Parameters
        ----------
        fk_solver : FKSolver
        """
        self._fk = fk_solver
        self._traj = None        # current TrajectoryResult
        self._q_final = None     # last joint config, held after completion

    def load(self, traj_result) -> None:
        """Load a new trajectory for execution.

        Parameters
        ----------
        traj_result : TrajectoryResult  (from TOPPRAParameterizer)
        """
        self._traj = traj_result
        # Pre-compute final pose so we can hold it after trajectory ends
        q_end, _ = traj_result.query(traj_result.duration)
        self._q_final = q_end.copy()

    def query_eef_pose(self, t: float) -> np.ndarray:
        """Return the EEF pose at trajectory time t via FK.

        Parameters
        ----------
        t : float  — seconds since trajectory start

        Returns
        -------
        pose : np.ndarray, shape (7,)  [x, y, z, qx, qy, qz, qw]
        """
        if self._traj is None:
            raise RuntimeError("No trajectory loaded. Call load() first.")

        if self._traj.is_finished(t):
            q = self._q_final
        else:
            q, _ = self._traj.query(t)

        return self._fk.compute_pose(q)

    def is_finished(self, t: float) -> bool:
        """Return True when the trajectory has been fully executed."""
        if self._traj is None:
            return True
        return self._traj.is_finished(t)

    @property
    def duration(self) -> float:
        """Total trajectory duration in seconds."""
        if self._traj is None:
            return 0.0
        return self._traj.duration

    def reset(self) -> None:
        """Clear the loaded trajectory."""
        self._traj = None
        self._q_final = None
