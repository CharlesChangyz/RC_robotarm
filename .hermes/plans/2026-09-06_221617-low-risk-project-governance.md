# RC Robot Arm Low-Risk Project Governance Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Make the existing project reproducibly buildable and testable without changing runtime behavior, motion algorithms, limits, topics, services, or hardware configuration values.

**Architecture:** Keep all production execution paths unchanged. Restore the ignored tests as versioned assets, align their assertions with the current source contract, register them with the owning ROS packages, declare already-used dependencies, and verify everything from isolated build directories under `/tmp` so stale workspace artifacts cannot affect results.

**Tech Stack:** ROS 2 Humble, colcon, ament_cmake, ament_python, pytest, GoogleTest, setuptools, CMake.

---

## Constraints and baseline

- Do not add product features, limit enforcement, watchdogs, new ROS interfaces, new parameters, or new runtime dependencies beyond libraries already imported by current code.
- Do not change control formulas, gains, launch defaults, action-set values, URDF/MuJoCo limits, motor mappings, vacuum behavior, or payload behavior.
- Do not start real-hardware processes; validation is build, unit/static test, and import based only.
- Baseline: all eight ROS packages build; isolated Python collection currently stops on the obsolete `target_point_fallback` test; excluding it yields 21 passes and 10 stale-test failures.
- Do not alter `TODO` license or maintainer identities without owner-supplied legal metadata.
- Do not commit automatically; use the task boundaries below as suggested commit boundaries if the user later requests commits.

## Task 1: Restore tests to version control

**Files:**
- Modify: `.gitignore`
- Modify: `rc_moveit/.gitignore`
- Move: `rc_moveit/rc_arm2_middleware/tests/test_motion_step_names.py` → `rc_moveit/rc_arm2_middleware/test/test_motion_step_names.py`
- Move: `rc_moveit/rc_arm2_middleware/tests/test_target_point_sampling.py` → `rc_moveit/rc_arm2_middleware/test/test_target_point_sampling.py`
- Remove after migration: `rc_moveit/rc_arm2_middleware/test/test_target_point_fallback.py`

**Steps:**

1. Remove only the broad `tests/`, package `test/`, and `rc_arm_moveit_config/test/` ignore rules; keep `__pycache__/`, `*.pyc`, build, install, log, virtualenv, IDE, temporary, and generated-URDF rules.
2. Standardize the middleware tests under the ROS-conventional singular `test/` directory.
3. Remove `test_target_point_fallback.py`: its production module no longer exists, and the current fallback behavior is already covered by `test_target_point_sampling.py`; do not recreate the deleted production feature.
4. Confirm `git check-ignore` reports no match for root, middleware, hardware, or MoveIt test sources, while cache files remain ignored.

**Verification:**

```bash
git check-ignore tests/test_tf_gui_shutdown.py \
  rc_moveit/rc_arm2_middleware/test/test_target_point_sampling.py \
  rc_moveit/rc_arm_hardware/test/test_payload_blend.cpp \
  rc_moveit/rc_arm_moveit_config/test/test_cartesian_motion_defaults.py
```

Expected: no output and exit status 1.

## Task 2: Update stale tests to the current contract

**Files:**
- Modify: `tests/test_run_tf_cli_domain55_script.py`
- Modify: `rc_moveit/rc_arm2_middleware/test/test_motion_step_names.py`
- Modify: `rc_moveit/rc_arm2_middleware/test/test_target_point_sampling.py`
- Modify: `rc_moveit/rc_arm_moveit_config/test/test_cartesian_motion_defaults.py`

**Steps:**

1. Change the domain script assertion to require `export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"`, preserving both the default domain and environment override behavior already implemented.
2. Replace the assertion that every `fallback_xyz` equals zero with assertions that every declared fallback is a three-element numeric finite vector; do not modify `action_sets.yaml` values.
3. Remove the obsolete `max_x` keyword from all sampling calls. Preserve the existing expected outcomes using the current `max_abs_y` and distance filters; the existing “too few samples” case remains valid because one sample violates `max_abs_y`.
4. Update the Cartesian middleware contract test to assert the current single Cartesian target topic and `_publish_motion_target(..., cartesian=True)` route instead of the removed Cartesian path topic/helper names.
5. Update `Arm2TargetPath.msg` expectations to require exactly two fields in order: `Arm2TargetPoint[] waypoints` and `float64 blend_radius`.
6. Run tests with source packages placed before `install/` on `PYTHONPATH`, bytecode and pytest cache disabled.

**Verification:**

```bash
source /opt/ros/humble/setup.bash
source rc_moveit/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/rc_moveit/rc_arm2_middleware:$PWD/rc_moveit/rc_arm_teleop:$PYTHONPATH" \
python3 -m pytest -p no:cacheprovider -q \
  tests \
  rc_moveit/rc_arm2_middleware/test \
  rc_moveit/rc_arm_moveit_config/test
```

Expected: collection succeeds and all current Python tests pass; no production source or action-set data changes are needed.

## Task 3: Register package-owned tests with colcon

**Files:**
- Modify: `rc_moveit/rc_arm2_middleware/setup.py`
- Modify: `rc_moveit/rc_arm_moveit_config/CMakeLists.txt`
- Modify: `rc_moveit/rc_arm_moveit_config/package.xml`
- Modify: `rc_moveit/rc_arm_hardware/CMakeLists.txt`

**Steps:**

1. Add `tests_require=["pytest"]` to the middleware setuptools metadata and retain the existing `python3-pytest` test dependency.
2. Under `if(BUILD_TESTING)` in MoveIt config, find `ament_cmake_pytest` and register each of the four existing Python test files as a separate `ament_add_pytest_test` target with a 60-second timeout.
3. Add `<test_depend>ament_cmake_pytest</test_depend>` to the MoveIt config manifest.
4. Under `if(BUILD_TESTING)` in hardware CMake, find `ament_cmake_gtest`, create `test_payload_blend` from the existing test source, and give it the package include directory. The header-only payload-blend implementation requires no production target linkage.
5. Do not add placeholder tests for teleop; reporting zero teleop tests is preferable to adding assertions without an independently specified behavior contract.

**Verification:**

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths rc_moveit --symlink-install \
  --packages-select rc_arm2_middleware rc_arm_hardware rc_arm_moveit_config \
  --cmake-args -DBUILD_TESTING=ON
colcon test --base-paths rc_moveit \
  --packages-select rc_arm2_middleware rc_arm_hardware rc_arm_moveit_config
colcon test-result --test-result-base rc_moveit/build --verbose
```

Expected: middleware Python tests, four MoveIt pytest targets, and five payload-blend GoogleTests are discovered and pass.

## Task 4: Align manifests with imports and installation contents

**Files:**
- Modify: `rc_moveit/rc_arm_teleop/package.xml`
- Modify: `rc_moveit/rc_arm_hardware/package.xml`
- Modify: `rc_moveit/rc_arm_moveit_config/package.xml`
- Modify: `rc_moveit/arm_msgs/package.xml`
- Modify: `rc_moveit/rc_arm_moveit_config/CMakeLists.txt`
- Modify: `setup.py`
- Modify: package descriptions in the affected `package.xml`/`setup.py` files

**Steps:**

1. Add existing teleop runtime dependencies: `builtin_interfaces`, `control_msgs`, `std_srvs`, `trajectory_msgs`, `python3-numpy`, and `python3-scipy`.
2. Add existing hardware runtime dependencies: `sensor_msgs`, `std_msgs`, and `std_srvs`.
3. Add existing MoveIt-config script dependencies: `sensor_msgs`, `tf2_msgs`, `python3-numpy`, `python3-scipy`, `launch`, and `launch_ros`.
4. Add `rosidl_default_runtime` as the runtime dependency for generated `arm_msgs` interfaces.
5. Keep Python package `install_requires` unchanged except set root `python_requires` from `>=3.8` to `>=3.10`, matching the existing `X | None` syntax. ROS/system dependencies remain in `package.xml` for rosdep rather than being duplicated as pip requirements.
6. Change stale “EL-A3 6-DOF” descriptions to neutral, factual “rc_arm_2 robot arm” wording. Do not modify maintainer identities, licenses, versions, or package names.
7. Add `PATTERN "__pycache__" EXCLUDE` and `PATTERN "*.pyc" EXCLUDE` to the MoveIt config directory install rule.

**Verification:**

```bash
source /opt/ros/humble/setup.bash
rosdep check --from-paths rc_moveit --ignore-src
python3 -m compileall -q -f rc_robotarm_mujoco demo rc_moveit
```

Expected: rosdep finds no undeclared/missing dependency on a fully provisioned Humble environment; Python compilation succeeds. If rosdep reports packages not installed locally, record them separately and do not weaken the manifests.

## Task 5: Document and prove a clean offline build

**Files:**
- Modify: `README.md`
- Modify only if statements are stale: `CODE_AUDIT_SUGGESTIONS.md`, `CODE_TRIM_SUGGESTIONS.md`

**Steps:**

1. Update both Chinese and English requirements from Python 3.8+ to Python 3.10+.
2. Add a concise “offline verification” section containing the isolated build procedure below and state explicitly that it does not start the robot or require connected hardware.
3. Document the two test layers: package tests through `colcon test`, and repository-level static/GUI-script tests through pytest.
4. Mark historical audit statements as dated where they conflict with the current tree; do not rewrite operational behavior documentation in this batch.
5. Run the final build with separate `/tmp` bases so existing `rc_moveit/build`, `install`, and `log` cannot influence the result. Do not delete the user’s existing build products.

**Final acceptance commands:**

```bash
VERIFY_ROOT="$(mktemp -d /tmp/rc_robotarm_verify.XXXXXX)"
source /opt/ros/humble/setup.bash
colcon --log-base "$VERIFY_ROOT/log" build \
  --base-paths rc_moveit \
  --build-base "$VERIFY_ROOT/build" \
  --install-base "$VERIFY_ROOT/install" \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=ON
source "$VERIFY_ROOT/install/setup.bash"
colcon --log-base "$VERIFY_ROOT/test-log" test \
  --base-paths rc_moveit \
  --build-base "$VERIFY_ROOT/build" \
  --install-base "$VERIFY_ROOT/install" \
  --packages-select rc_arm2_middleware rc_arm_hardware rc_arm_moveit_config
colcon test-result --test-result-base "$VERIFY_ROOT/build" --verbose
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/rc_moveit/rc_arm2_middleware:$PWD/rc_moveit/rc_arm_teleop:$PYTHONPATH" \
python3 -m pytest -p no:cacheprovider -q tests
```

Expected acceptance result:

- All eight packages build from isolated directories.
- All registered tests in the three selected packages pass.
- All repository-level tests pass.
- `git diff` contains no change to production Python/C++ logic, motion configuration, limit values, ROS names, or launch defaults.
- No real-hardware executable or launch file is started.

## Risks and handling

- The `arm_msgs` XML-schema lint can fail without network access because the manifest references the remote ROS package XSD. Treat that separately from functional test failure; do not remove the schema declaration solely to make an offline check green.
- `rosdep check` may report missing local packages even when declarations are correct. Manifest correctness is the acceptance criterion; dependency installation is not part of this batch.
- Moving ignored tests makes them newly tracked rather than modifying previously tracked history. Review the full staged file list before any future commit.
