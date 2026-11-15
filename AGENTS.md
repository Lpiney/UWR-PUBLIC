# Repository Guidelines

## Project Structure & Module Organization

This repository controls the CityUHK training ROV. Python on the laptop handles the controller, camera, and vision; the ESP32-S3 handles serial commands, thrusters, and Mission 1 networking.

- `host/console.py`: operator display and task switching.
- `host/pad_bridge.py`: controller input, serial bridge, and telemetry.
- `host/vision/`: AprilTag and colour recognition, with adjacent tests.
- `host/test_console.py`: console interaction tests.
- `host/test_pad_bridge.py`: controller, safety-key, protocol and port-picking tests.
- `firmware/ESP32/src/`: Arduino C++ entry point and headers for communications, configuration, parameters, thrusters, and networking.
- `docs/GUIDE.md`: operating procedure; `docs/xbox-controller/`: controller references and images.

## Build, Test, and Development Commands

Run from the repository root unless stated otherwise:

- `python3 -m pip install -r host/requirements.txt`: install host dependencies. Use `pygame-ce` without `pygame`; retain the pinned desktop OpenCV build.
- `python3 host/console.py`: start the operator console.
- `python3 host/pad_bridge.py --show-input`: inspect controller axes and buttons.
- From `host/`, run `python3 -m unittest discover -v`: execute console, bridge and vision tests.
- From `firmware/ESP32/`, run `pio run`: compile; `pio run -t upload`: flash. If unavailable on PATH, use `~/.platformio/penv/bin/pio`.

## Coding Style & Naming Conventions

Match surrounding code: Python uses four-space indentation, `snake_case` functions, `PascalCase` classes, and uppercase constants. Firmware uses two-space indentation and camelCase functions. No formatter or linter is configured. Keep comments focused on behaviour and hardware constraints.

## Testing Guidelines

Use Python's `unittest`, `test_*.py` files, and descriptive `test_*` methods. Every test must run with no camera, board, controller or window: the stand-ins for those live in the test files, so the shipped programs keep a single hardware path and grow no test modes. Add regression tests for changed behaviour and compile firmware changes. No coverage threshold is configured.

## Commit & Pull Request Guidelines

Follow Git history's short, imperative subjects, such as “Show the keyboard shortcuts on screen.” Keep commits focused. PR descriptions should explain the behaviour change, validation performed, hardware checks still needed, and linked issues where applicable. Include screenshots for display changes.

## Hardware & Configuration Safety

Remove propellers during bench tests. Preserve firmware safety gates, link timeout, startup neutral output, and throttle limiting. Keep the control loop responsive; networking runs on core 0. Keep the single 8 MB flash configuration: it is the one that boots on both boards. Keep snapshots and local calibration configuration out of commits. Update operating documentation when controls or protocol behaviour change.


## Execution Efficiency — Batch Editing and Validation

**Primary principle: Work in coherent batches, not tiny edit-verify loops. Optimize for task completion speed while preserving correctness.**

### 1. Analyze First, Edit in Batches

* Inspect the relevant code and understand dependencies before editing.
* Form a concise internal implementation plan.
* Group related changes into coherent batches, preferably covering the complete feature or fix.
* Avoid repeatedly editing the same file in tiny increments.
* Prefer applying multiple related edits in one operation when practical.
* Do not reread files whose contents are already known unless they have changed or new information is needed.

### 2. Avoid Excessive Verification

* Do not run tests, builds, linters, or type checks after every small edit.
* Complete a coherent batch of changes before running verification.
* Use the following validation strategy:

  * Small changes: targeted checks only, when appropriate.
  * Medium changes: targeted tests after implementation.
  * Large changes: validate at meaningful milestones and perform broader verification at the end.
* Do not repeatedly run the same successful check without relevant changes.
* Reserve full test suites and full builds for cases where their coverage is justified or explicitly required.

### 3. Fix Related Issues Together

* When validation fails, collect and analyze the relevant errors before editing.
* Fix multiple related errors in one batch rather than one error per iteration.
* Rerun only the affected checks when possible.
* Expand verification scope if failures suggest wider regressions.

### 4. Minimize Tool Calls and Overhead

* Prefer targeted file searches over reading the entire repository.
* Combine related read operations and independent shell commands where practical.
* Avoid redundant status checks, repetitive planning, and unnecessary documentation.
* Do not create extra scripts, reports, or temporary files unless they materially help complete the task.
* Keep progress updates concise and only at meaningful milestones.

### 5. Autonomous Execution

* For routine, reversible code changes, proceed without asking for approval at every step.
* Make reasonable implementation decisions independently.
* Do not stop after each small milestone to request confirmation.
* Ask for clarification only when a missing decision materially affects correctness, scope, or safety.
* Preserve existing security, approval, and deployment boundaries.

### 6. Completion Criteria

* Deliver the requested behavior, not merely a partial implementation.
* Perform the minimum sufficient verification based on change risk and repository requirements.
* Report what changed, what was verified, and any remaining risks.
* Never claim tests passed unless they actually ran successfully.

**Default workflow: Inspect → Plan → Batch Edit → Targeted Verify → Fix if Needed → Finish.**

Avoid: Inspect → Edit One Line → Test → Edit One Line → Test → Repeat.
