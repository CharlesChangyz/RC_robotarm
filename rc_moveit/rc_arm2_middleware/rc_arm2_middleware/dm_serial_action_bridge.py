"""Helpers for mapping DM-serial raw CAN frames to middleware action sets.

Default protocol:
- command `0x4xx` maps to action set `0xXX`
- completion returns fixed id `0x500`
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import List, Optional, Set


@dataclass(frozen=True)
class RawCanFrame:
    can_id: int
    dlc: int
    data: List[int] = field(default_factory=list)
    is_extended: bool = False
    is_remote: bool = False
    is_fd: bool = True


class DmSerialActionBridge:
    def __init__(self, action_set_ids: Iterable[int], command_base_id: int, complete_id: int) -> None:
        self._action_set_ids: Set[int] = {int(action_id) for action_id in action_set_ids}
        self._command_base_id = int(command_base_id)
        self._complete_id = int(complete_id)

    def action_set_id_from_frame(self, frame: RawCanFrame) -> Optional[int]:
        if frame.is_extended:
            return None

        action_id = int(frame.can_id) - self._command_base_id
        if action_id < 1 or action_id not in self._action_set_ids:
            return None
        return action_id

    def completion_frame(self) -> RawCanFrame:
        return RawCanFrame(can_id=self._complete_id, dlc=0, data=[])


def resolve_allowed_action_set_ids(
    action_set_ids: Iterable[int],
    configured_ids: object,
) -> Set[int]:
    available_ids = {int(action_id) for action_id in action_set_ids}
    if configured_ids is None:
        return available_ids

    if isinstance(configured_ids, str):
        configured_ids = configured_ids.strip()
        if not configured_ids:
            return available_ids
        requested_ids = {
            int(part.strip(), 0)
            for part in configured_ids.split(",")
            if part.strip()
        }
    elif isinstance(configured_ids, Iterable):
        requested_ids = {int(action_id) for action_id in configured_ids}
    else:
        requested_ids = {int(configured_ids)}

    return available_ids.intersection(requested_ids)
