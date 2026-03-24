import numpy as np
import mujoco
from dm_control import mjcf

from rc_robotarm_mujoco.utils.transform_utils import mat2quat


class FKSolver:
    """Forward kinematics using an isolated shadow physics model.

    The shadow model is compiled once from the same arena MJCF model and is
    never stepped, keeping it fully independent from the live simulation.
    """

    def __init__(self, arena_mjcf_model, arm) -> None:
        """
        Parameters
        ----------
        arena_mjcf_model : mjcf.RootElement
            Fully assembled arena model (arena + arm already attached).
        arm : Arm
            Arm instance for joint / site binding.
        """
        self._shadow = mjcf.Physics.from_mjcf_model(arena_mjcf_model)
        self._arm = arm

    def compute_pose(self, q: np.ndarray) -> np.ndarray:
        """Compute world-frame EEF pose for joint configuration q.

        Parameters
        ----------
        q : np.ndarray, shape (4,)

        Returns
        -------
        pose : np.ndarray, shape (7,) — [x, y, z, qx, qy, qz, qw]
        """
        self._shadow.bind(self._arm.joints).qpos = q
        mujoco.mj_kinematics(self._shadow.model.ptr, self._shadow.data.ptr)
        pos = self._shadow.bind(self._arm.eef_site).xpos.copy()
        mat = self._shadow.bind(self._arm.eef_site).xmat.reshape(3, 3).copy()
        quat = mat2quat(mat)
        return np.concatenate([pos, quat])

    def compute_pose_batch(self, Q: np.ndarray) -> np.ndarray:
        """Compute FK for a batch of configurations.

        Parameters
        ----------
        Q : np.ndarray, shape (N, 4)

        Returns
        -------
        poses : np.ndarray, shape (N, 7)
        """
        poses = np.zeros((len(Q), 7))
        for i, q in enumerate(Q):
            poses[i] = self.compute_pose(q)
        return poses
