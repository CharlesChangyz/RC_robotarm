import numpy as np

from rc_robotarm_mujoco.planning.fk_solver import FKSolver
from rc_robotarm_mujoco.planning.ik_solver import IKSolver
from rc_robotarm_mujoco.planning.collision_checker import CollisionChecker
from rc_robotarm_mujoco.planning.ompl_planner import OMPLPlanner
from rc_robotarm_mujoco.planning.toppra_parameterizer import (
    TOPPRAParameterizer,
    DEFAULT_VEL_LIMITS,
    DEFAULT_ACC_LIMITS,
)
from rc_robotarm_mujoco.planning.trajectory_executor import TrajectoryExecutor


class MotionPlanner:
    """Top-level coordinator: IK → OMPL → TOPP-RA → trajectory execution.

    Usage
    -----
    Create once after the gymnasium environment has been reset::

        mp = MotionPlanner(env.unwrapped._arena.mjcf_model, env.unwrapped._arm)

    Request a new target in the ROS2 callback (or whenever a new goal arrives)::

        success = mp.request(target_pose_7d, q_current)

    Each simulation step::

        eef_pose = mp.query_eef_pose(t)   # t = seconds since trajectory start
        env.unwrapped._target.set_mocap_pose(physics, eef_pose[:3], eef_pose[3:])

    Once mp.is_finished(t) is True the arm holds its final pose.
    """

    def __init__(
        self,
        arena_mjcf_model,
        arm,
        vel_limits: np.ndarray = DEFAULT_VEL_LIMITS,
        acc_limits: np.ndarray = DEFAULT_ACC_LIMITS,
        ompl_planner_name: str = "RRTConnect",
        ompl_range: float = 0.1,
        ompl_solve_time: float = 5.0,
        ik_n_restarts: int = 12,
        ik_position_tol: float = 5e-3,
    ) -> None:
        """
        Parameters
        ----------
        arena_mjcf_model : mjcf.RootElement
            Fully assembled arena model with the arm already attached.
        arm : Arm
            The arm instance (joints, eef_site, etc.).
        vel_limits : np.ndarray, shape (4,)
            Max joint velocity for each joint [rad/s].
        acc_limits : np.ndarray, shape (4,)
            Max joint acceleration for each joint [rad/s²].
        ompl_planner_name : str
            OMPL planner class name (default "RRTConnect").
        ompl_range : float
            OMPL planner step size [rad].
        ompl_solve_time : float
            Planning time budget [s].
        ik_n_restarts : int
            Number of IK random restarts.
        ik_position_tol : float
            IK position tolerance [m].
        """
        self._arm = arm

        self.fk = FKSolver(arena_mjcf_model, arm)
        self.checker = CollisionChecker(arena_mjcf_model, arm)
        self.ik = IKSolver(
            self.fk,
            n_restarts=ik_n_restarts,
            position_tol=ik_position_tol,
        )
        self.planner = OMPLPlanner(
            self.checker,
            planner_name=ompl_planner_name,
            range=ompl_range,
            solve_time=ompl_solve_time,
        )
        self.parameterizer = TOPPRAParameterizer(vel_limits, acc_limits)
        self.executor = TrajectoryExecutor(self.fk)

        self._active = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def request(
        self,
        target_pose: np.ndarray,
        q_current: np.ndarray,
    ) -> bool:
        """Plan and load a trajectory to the given Cartesian target.

        Executes the full pipeline synchronously (blocks until done):
          1. IK  → q_goal
          2. OMPL → collision-free waypoints
          3. TOPP-RA → time-parameterized trajectory
          4. TrajectoryExecutor.load()

        Parameters
        ----------
        target_pose : np.ndarray, shape (7,)  [x,y,z,qx,qy,qz,qw]
        q_current   : np.ndarray, shape (4,)

        Returns
        -------
        bool : True if planning succeeded and trajectory is ready.
        """
        self._active = False

        # Step 1: IK
        print("[MotionPlanner] Solving IK...")
        q_goal, ik_ok = self.ik.solve(target_pose, q_init=q_current)
        if not ik_ok:
            print("[MotionPlanner] IK failed. Aborting.")
            return False

        # Step 2: if already at goal, skip planning
        if np.linalg.norm(q_goal - q_current) < 1e-3:
            print("[MotionPlanner] Already at goal configuration.")
            traj = self.parameterizer.parameterize([q_current, q_goal])
            self.executor.load(traj)
            self._active = True
            return True

        # Step 3: OMPL path planning
        print("[MotionPlanner] Running OMPL path planning...")
        waypoints, plan_ok = self.planner.plan(q_current, q_goal)
        if not plan_ok:
            print("[MotionPlanner] OMPL planning failed. Aborting.")
            return False

        # Step 4: TOPP-RA time parameterization
        print("[MotionPlanner] Running TOPP-RA parameterization...")
        traj = self.parameterizer.parameterize(waypoints)

        # Step 5: Load trajectory
        self.executor.load(traj)
        self._active = True
        print(f"[MotionPlanner] Trajectory ready. Duration: {traj.duration:.3f} s")
        return True

    def query_eef_pose(self, t: float) -> np.ndarray:
        """Return EEF pose at trajectory time t.

        Parameters
        ----------
        t : float  — seconds since the last successful request()

        Returns
        -------
        pose : np.ndarray, shape (7,)  [x,y,z,qx,qy,qz,qw]
        """
        return self.executor.query_eef_pose(t)

    def is_finished(self, t: float) -> bool:
        """Return True when the trajectory has completed."""
        return self.executor.is_finished(t)

    @property
    def duration(self) -> float:
        """Total duration of the current trajectory [s]."""
        return self.executor.duration

    @property
    def is_active(self) -> bool:
        """True if a trajectory has been successfully planned and loaded."""
        return self._active
