from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
GUI_SOURCE = ROOT_DIR / "demo" / "tf_target_cli_publisher.py"


def _source() -> str:
    return GUI_SOURCE.read_text(encoding="utf-8")


def test_control_process_starts_in_own_session() -> None:
    source = _source()

    assert "setsid" in source


def test_control_process_stops_process_group_with_sigint_first() -> None:
    source = _source()

    sigint_pos = source.find("signal.SIGINT")
    sigterm_pos = source.find("signal.SIGTERM")
    sigkill_pos = source.find("signal.SIGKILL")

    assert "os.killpg" in source
    assert sigint_pos != -1
    assert sigterm_pos != -1
    assert sigkill_pos != -1
    assert sigint_pos < sigterm_pos < sigkill_pos
