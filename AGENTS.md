# AGENTS.md

This repository is a real-hardware ROS 2 workspace for a Unitree GO2 assistive mobility stack. Do not generalize that to other repos. In this repo, ROS is central because the tree contains ROS packages, launch files, message definitions, `colcon` build surfaces, and ROS-oriented CI.

## Repo-Specific Working Norms

1. Inspect before editing.
   - Read the target package, its `package.xml` or `CMakeLists.txt` or `setup.py`, nearby tests, launch/config files, and relevant docs before making changes.
   - Trace the contract boundary first: topics, frames, message types, launch arguments, package entry points, and external dependencies.

2. No fake completion.
   - Do not claim something is fixed, working, or validated unless you ran the relevant check.
   - Always separate `verified`, `inferred`, and `not yet validated`.

3. Push back on weak ideas.
   - Reject shortcuts that hide breakage risk, weaken observability, or encode one machine's quirks as architecture.
   - If the user asks for a brittle workaround, explain the debt explicitly and offer the smallest safer alternative.

4. Preserve public interfaces unless intentionally changing them.
   - Treat `/goal_pose`, `/go2/confirmed_target`, `/go2/detected_humans`, `/go2/safety_alert`, launch args, message fields, package names, and console entry points as public contracts.
   - If an interface must change, call it out before the edit and in the final handoff.

5. Do not assume ROS in the abstract.
   - This repo is ROS-centric by evidence.
   - Future cross-repo work must still verify the stack before applying ROS-first reasoning.

6. Keep scope tight.
   - Avoid unrelated edits.
   - Do not touch `build/`, `install/`, or `log/` as if they were source.
   - Treat generated workspace state as disposable evidence, not canonical code.

## Validation Expectations

1. Use the smallest credible validation scope first.
   - Prefer targeted `pytest`, narrow static checks, and package-scoped inspection before broad workspace actions.
   - Escalate only when the change crosses package boundaries or narrow validation is not credible.

2. Use package-first validation.
   - `go2_msgs` is the interface boundary. Build or inspect it first when message contracts are involved.
   - For Python-only logic, prefer `./scripts/test.sh`, targeted `pytest`, `./scripts/lint.sh`, and `./scripts/validate.sh`.
   - For launch changes, validate installed asset paths, package resolution, and launch arguments before trying to launch the whole stack.

3. Match checks to the change surface.
   - Messages or interfaces: verify definitions, imports, downstream consumers, and build order.
   - Launch or config: verify package shares, external package assumptions, behavior tree paths, and parameter names.
   - Runtime logic: run the narrowest test or smoke check that can falsify the claim.

4. Report the real boundary of proof.
   - If hardware, DDS networking, TF, sensors, or external packages are unavailable, say so plainly and validate everything else offline.

## Repo-Specific Guardrails

- This repo is the hardware path. It is not the simulation workspace.
- `use_sim:=true` is intentionally fail-fast. Do not quietly reintroduce pretend simulation support here.
- Safety semantics are weaker than the domain might suggest. `go2_safety_monitor` publishes alerts; it does not by itself prove enforced motion interlocks.
- TF from camera frame to `map` is a first-order runtime dependency for intent grounding to navigation handoff.
- Camera topics and QoS assumptions are hard-coded in Python nodes. Treat QoS mismatches and topic drift as likely integration failures.
- Audio and ASR nodes are threaded and device-dependent. Do not trust shutdown, latency, or audio thresholds without evidence.
- `docker/nemo-ros2/` is a specialized NeMo path, not proof of a complete repo-wide dev environment.

## Definition Of Done

A significant task is not done unless:

1. The root cause or highest-confidence explanation is stated.
2. The exact files changed are listed.
3. The validation commands and outcomes are listed, or the blocker is stated precisely.
4. Public interface impact is called out explicitly.
5. Residual risks and unverified assumptions are named.

## Response Contract

For each significant task, report:

1. Root cause or diagnosis
2. Exact files changed and why
3. Validation performed and results
4. Residual risks, assumptions, and what remains unverified
