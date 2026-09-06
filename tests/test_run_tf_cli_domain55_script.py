from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT = ROOT_DIR / "scripts" / "run_tf_cli_domain55.sh"


def test_tf_cli_domain55_script_launches_gui_in_domain_55() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert source.startswith("#!/usr/bin/env bash")
    assert "set -eo pipefail" in source
    assert 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"' in source
    assert "source /opt/ros/humble/setup.bash" in source
    assert 'source "${WORKSPACE_DIR}/install/setup.bash"' in source
    assert 'cd "${REPO_ROOT}/demo"' in source
    assert 'exec python3 tf_target_cli_publisher.py "$@"' in source
