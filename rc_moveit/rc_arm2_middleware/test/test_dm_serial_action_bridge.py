from rc_arm2_middleware import dm_serial_action_bridge
from rc_arm2_middleware.dm_serial_action_bridge import (
    DmSerialActionBridge,
    RawCanFrame,
)


def test_decodes_standard_command_frame_to_action_set_id():
    bridge = DmSerialActionBridge(action_set_ids={1, 2}, command_base_id=0x30, complete_id=0x40)

    action_id = bridge.action_set_id_from_frame(RawCanFrame(can_id=0x31, dlc=0, data=[]))

    assert action_id == 1


def test_ignores_extended_and_unknown_command_frames():
    bridge = DmSerialActionBridge(action_set_ids={1, 2}, command_base_id=0x30, complete_id=0x40)

    assert bridge.action_set_id_from_frame(
        RawCanFrame(can_id=0x31, dlc=0, data=[], is_extended=True)
    ) is None
    assert bridge.action_set_id_from_frame(RawCanFrame(can_id=0x35, dlc=0, data=[])) is None


def test_allows_remote_flag_because_dm_usb_reports_data_frames_as_remote():
    bridge = DmSerialActionBridge(action_set_ids={1, 2}, command_base_id=0x30, complete_id=0x40)

    assert bridge.action_set_id_from_frame(
        RawCanFrame(can_id=0x32, dlc=0, data=[], is_remote=True)
    ) == 2


def test_builds_completion_frame():
    bridge = DmSerialActionBridge(action_set_ids={1}, command_base_id=0x30, complete_id=0x40)

    frame = bridge.completion_frame()

    assert frame.can_id == 0x40
    assert frame.dlc == 0
    assert frame.data == []
    assert not frame.is_extended
    assert not frame.is_remote


def test_resolves_configured_allowed_action_set_ids():
    assert hasattr(dm_serial_action_bridge, "resolve_allowed_action_set_ids")
    allowed = dm_serial_action_bridge.resolve_allowed_action_set_ids({1, 2, 3, 4, 5}, "1,2")

    assert allowed == {1, 2}


def test_empty_allowed_action_set_ids_allows_all_action_sets():
    allowed = dm_serial_action_bridge.resolve_allowed_action_set_ids({1, 2, 3, 4, 5}, "")
    bridge = DmSerialActionBridge(action_set_ids=allowed, command_base_id=0x30, complete_id=0x40)

    assert bridge.action_set_id_from_frame(RawCanFrame(can_id=0x35, dlc=0, data=[])) == 5


def test_configured_allowed_ids_prevent_other_bus_ids_from_triggering():
    allowed = dm_serial_action_bridge.resolve_allowed_action_set_ids({1, 2, 3, 4, 5}, "1,2")
    bridge = DmSerialActionBridge(action_set_ids=allowed, command_base_id=0x30, complete_id=0x40)

    assert bridge.action_set_id_from_frame(RawCanFrame(can_id=0x32, dlc=0, data=[])) == 2
    assert bridge.action_set_id_from_frame(RawCanFrame(can_id=0x35, dlc=0, data=[])) is None
