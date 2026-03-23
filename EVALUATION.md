# Evaluation Plan

This document defines the first publishable evaluation pass for the public thesis stack. It is intentionally modest: the goal is to produce credible measurements for the behaviors that already exist in the repo, not to claim a finished assistive-navigation benchmark.

## Metrics To Publish First

| Area | Metric | How to measure |
|---|---|---|
| Follow behavior | Mean absolute follow distance error | Compare commanded vs measured separation over each trial |
| Follow behavior | Tracking loss events per trial | Count target loss events lasting longer than 1 second |
| Reacquisition | Reacquisition success rate | Successful return to confirmed target within 5 seconds after occlusion |
| Voice interaction | Command-to-action latency | Timestamp from end of spoken command to first robot motion or explicit acknowledgement |
| Safety | Stop response behavior | Trigger source, stop confirmation, and whether motion resumed only after operator/user clearance |

## Minimum Trial Set

| Scenario | Suggested trials | Notes |
|---|---|---|
| Straight corridor follow | 10 | Baseline distance-keeping behavior |
| Turned hallway follow | 10 | Includes target heading changes |
| Temporary crowd occlusion | 10 | Measure reacquisition vs permanent loss |
| Ambiguous voice command | 10 | Verify intent grounding and safe refusal behavior |
| Safety stop trigger | 10 | Obstacle, unexpected bystander, or clearance failure |

## Trial Template

| Trial ID | Scenario | Outcome | Distance error | Lost target? | Reacquired? | Command latency | Safety event | Notes |
|---|---|---:|---:|---|---|---:|---|---|
| F-01 | corridor follow | pending | - | - | - | - | no | fill during data collection |
| F-02 | corridor follow | pending | - | - | - | - | no | fill during data collection |
| O-01 | occlusion | pending | - | yes/no | yes/no | - | no | fill during data collection |

## Failure Modes Worth Reporting

Report these explicitly even if the numbers are not flattering:
- incorrect user lock when multiple people are visible
- false follow after casual bystander speech
- loss of target during depth dropouts
- unnecessary stop triggers from conservative safety thresholds
- delayed resume after a valid stop / continue sequence

## Publication Rule

Do not collapse these into one vague success number. Publish follow performance, reacquisition performance, and safety behavior separately.
