import numpy as np
from scipy.optimize import minimize

from rc_robotarm_mujoco.utils.transform_utils import quat2mat


# Joint limits: j1(-π,π), j2(0,π), j3(0,π), j4(-π,π) — confirmed from XML
JOINT_LIMITS = np.array([
    [-np.pi,  np.pi],
    [0.0,     np.pi],
    [0.0,     np.pi],
    [-np.pi,  np.pi],
])


class IKSolver:
    """Numerical IK via scipy SLSQP with multiple random restarts.

    Because this arm has 4 DOF, full 6-DOF pose is underdetermined.
    The orientation cost only matches the tool-Z axis direction, consistent
    with the OSC's orientation_axis=[0,0,1] usage in the demo.
    """

    def __init__(
        self,
        fk_solver,
        joint_limits: np.ndarray = JOINT_LIMITS,
        n_restarts: int = 12,
        position_tol: float = 5e-3,
        max_iter: int = 500,
        orientation_weight: float = 0.1,
        regularization_weight: float = 0.01,
    ) -> None:
        self._fk = fk_solver
        self._limits = joint_limits
        self._n_restarts = n_restarts
        self._pos_tol = position_tol
        self._max_iter = max_iter
        self._ori_w = orientation_weight
        self._reg_w = regularization_weight
        self._bounds = [(float(lo), float(hi)) for lo, hi in joint_limits]
        self._q_preferred = np.array([0.0, np.pi / 2, np.pi / 2, 0.0])

    def solve(
        self,
        target_pose: np.ndarray,
        q_init: np.ndarray = None,
        q_preferred: np.ndarray = None,
    ) -> tuple:
        """Solve IK for a Cartesian target pose.

        Parameters
        ----------
        target_pose : np.ndarray, shape (7,)  [x,y,z,qx,qy,qz,qw]
        q_init : np.ndarray, shape (4,), optional warm-start
        q_preferred : np.ndarray, shape (4,), optional null-space target

        Returns
        -------
        (q_solution, success) : (np.ndarray shape (4,), bool)
        """
        target_pos = target_pose[:3]
        target_quat = target_pose[3:]
        target_z = quat2mat(target_quat)[:, 2]

        if q_preferred is None:
            q_preferred = self._q_preferred

        starts = []
        if q_init is not None:
            starts.append(q_init.copy())
        rng = np.random.default_rng()
        for _ in range(self._n_restarts):
            starts.append(rng.uniform(self._limits[:, 0], self._limits[:, 1]))

        best_q = q_preferred.copy()
        best_cost = np.inf

        for q0 in starts:
            result = minimize(
                self._cost,
                q0,
                args=(target_pos, target_z, q_preferred),
                method="SLSQP",
                bounds=self._bounds,
                options={"maxiter": self._max_iter, "ftol": 1e-9},
            )
            if result.fun < best_cost:
                best_cost = result.fun
                best_q = result.x.copy()

        pose = self._fk.compute_pose(best_q)
        pos_err = np.linalg.norm(pose[:3] - target_pos)
        success = pos_err < self._pos_tol

        if not success:
            print(f"[IKSolver] IK failed: position error = {pos_err:.4f} m (tol={self._pos_tol} m)")

        return best_q, success

    def _cost(
        self,
        q: np.ndarray,
        target_pos: np.ndarray,
        target_z: np.ndarray,
        q_preferred: np.ndarray,
    ) -> float:
        pose = self._fk.compute_pose(q)
        pos_err = float(np.sum((pose[:3] - target_pos) ** 2))
        current_z = quat2mat(pose[3:])[:, 2]
        z_err = 1.0 - float(np.dot(current_z, target_z)) ** 2
        reg = float(np.sum((q - q_preferred) ** 2))
        return pos_err + self._ori_w * z_err + self._reg_w * reg
