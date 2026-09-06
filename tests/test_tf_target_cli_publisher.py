import ast
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT = ROOT_DIR / "demo" / "tf_target_cli_publisher.py"


def _source() -> str:
    return SCRIPT.read_text(encoding="utf-8")


def _tree() -> ast.Module:
    return ast.parse(_source())


def _class_def(tree: ast.Module, name: str) -> ast.ClassDef:
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == name:
            return node
    raise AssertionError(f"class {name} not found")


def _function_def(nodes: list[ast.stmt], name: str) -> ast.FunctionDef:
    for node in nodes:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"function {name} not found")


def test_gui_exposes_j5_topics_and_controls() -> None:
    source = _source()

    assert "Float64" in source
    assert "--j5-command-topic" in source
    assert "--j5-position-topic" in source
    assert "/rc_arm_2/j5/command_position" in source
    assert "/rc_arm_2/j5/actual_position" in source
    assert "J5 target (m)" in source
    assert "Send J5" in source
    assert "Use actual" in source
    assert "J5 actual (m)" in source
    assert "Last J5 command (m)" in source


def test_backend_wires_j5_float64_pub_sub_and_flush() -> None:
    tree = _tree()
    backend = _class_def(tree, "RosBackend")

    init_source = ast.get_source_segment(_source(), _function_def(backend.body, "__init__"))
    start_source = ast.get_source_segment(_source(), _function_def(backend.body, "start"))
    spin_source = ast.get_source_segment(_source(), _function_def(backend.body, "_spin_loop"))

    assert "_j5_command_pub" in init_source
    assert "_j5_position_sub" in init_source
    assert "_pending_j5_command" in init_source
    assert "create_publisher(Float64, self._args.j5_command_topic, 10)" in start_source
    assert "create_subscription(" in start_source
    assert "Float64" in start_source
    assert "self._args.j5_position_topic" in start_source
    assert "self._on_j5_position" in start_source
    assert "self._flush_j5_request()" in spin_source

    method_names = {
        node.name for node in backend.body if isinstance(node, ast.FunctionDef)
    }
    assert "queue_j5_command" in method_names
    assert "_flush_j5_request" in method_names
    assert "_on_j5_position" in method_names


def test_window_connects_j5_status_and_send_button() -> None:
    tree = _tree()
    window = _class_def(tree, "TargetPublisherWindow")
    init_source = ast.get_source_segment(_source(), _function_def(window.body, "__init__"))
    target_editor_source = ast.get_source_segment(_source(), _function_def(window.body, "_build_target_editor"))
    system_source = ast.get_source_segment(_source(), _function_def(window.body, "_build_system_panel"))
    status_source = ast.get_source_segment(_source(), _function_def(window.body, "_build_status_panel"))

    assert "j5_position_updated.connect(self._on_j5_position)" in init_source
    assert "last_j5_status.connect(self._set_j5_status)" in init_source
    assert "_j5_target_spin" in target_editor_source
    assert "_send_j5_btn.clicked.connect(self._send_j5_command)" in target_editor_source
    assert "_j5_use_actual_btn.clicked.connect(self._use_actual_j5)" in target_editor_source
    assert "_j5_target_spin" not in system_source
    assert "_j5_actual_label" in status_source
    assert "_j5_command_status_label" in status_source
