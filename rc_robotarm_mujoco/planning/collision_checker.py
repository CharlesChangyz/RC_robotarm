import numpy as np
import mujoco
from dm_control import mjcf


class CollisionChecker:
    """MuJoCo-based collision checker using an isolated shadow physics model.

    Notes
    -----
    - MuJoCo automatically excludes parent-child body pairs from narrow-phase
      collision, so adjacent arm links never produce false positives.
    - The arena's `base_link` geom has contype=0 / conaffinity=0 and is
      already excluded from collision detection by MuJoCo.
    - The four obstacle pillars on base_link have contype=1 and correctly
      produce contacts when arm links penetrate them.
    - Field mesh (scene_collision body) and kfs marker bodies correctly
      produce contacts with arm links.

    Therefore `data.ncon > 0` after mj_kinematics + mj_collision is a
    reliable collision signal with no extra filtering required.
    """

    def __init__(self, arena_mjcf_model, arm) -> None:
        """
        Parameters
        ----------
        arena_mjcf_model : mjcf.RootElement
            Fully assembled arena model (arena + arm already attached).
        arm : Arm
            Arm instance for joint binding.
        """
        self._shadow = mjcf.Physics.from_mjcf_model(arena_mjcf_model)
        self._arm = arm

    def is_valid(self, q: np.ndarray) -> bool:
        """Return True if configuration q is collision-free.

        Sets joint positions in the shadow model, updates kinematics, runs
        collision detection, and returns True when no contacts are found.

        Parameters
        ----------
        q : np.ndarray, shape (4,)

        Returns
        -------
        bool
        """
        self._shadow.bind(self._arm.joints).qpos = q
        mujoco.mj_kinematics(self._shadow.model.ptr, self._shadow.data.ptr)
        mujoco.mj_collision(self._shadow.model.ptr, self._shadow.data.ptr)
        return int(self._shadow.data.ncon) == 0

    def contact_details(self, q: np.ndarray) -> list:
        """Return list of (body_name_1, body_name_2) contact pairs.

        Useful for debugging — call when is_valid() returns False.
        """
        self._shadow.bind(self._arm.joints).qpos = q
        mujoco.mj_kinematics(self._shadow.model.ptr, self._shadow.data.ptr)
        mujoco.mj_collision(self._shadow.model.ptr, self._shadow.data.ptr)
        model = self._shadow.model.ptr
        data = self._shadow.data.ptr
        contacts = []
        for i in range(int(data.ncon)):
            g1 = int(data.contact[i].geom1)
            g2 = int(data.contact[i].geom2)
            b1 = int(model.geom_bodyid[g1])
            b2 = int(model.geom_bodyid[g2])
            n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b1) or str(b1)
            n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b2) or str(b2)
            contacts.append((n1, n2))
        return contacts
