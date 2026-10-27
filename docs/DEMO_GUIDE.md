# Demo walkthrough

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos-current.svg" width="132" height="44" alt="Demos (current page)"></a>
  <a href="README.md"><img src="media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="REPOSITORY_GUIDE.md"><img src="media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="PROJECT_STATUS.md"><img src="media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="PROJECT_PLAN.md"><img src="media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

Related: [Suite reference](DEMO_SUITE.md) · [Visual runbook](ONBOARD_VISUAL_DEMO.md).

This guide takes you from choosing a demonstration to inspecting its outputs.
Run commands from the repository root. Complete the
[installation steps](../README.md#quick-start) before launching simulation.

## 1. Choose the right mode

| Your goal | Entry point | What the run establishes |
| --- | --- | --- |
| Explore the robot and viewers | `./geoff demo plaza` | Unscored onboard visual session |
| Explore a particular environment | `./geoff demo cafe --people 4 --duration 300` | The same visual pipeline in a representative world |
| Measure onboard goal completion | `./scripts/run_demo_suite.sh --case onboard-waypoints --goals 3 --people 8 --seed 44 --headless` | A scored trial subject to strict acceptance |
| Inspect locomotion response | `./scripts/run_demo_suite.sh --case fixed-forward --headless` | Forward response, stability and stopping diagnostics |
| Check benchmark mechanics | `./scripts/run_demo_suite.sh --suite quick --headless` | Mock orchestration and scoring, with a gate that can fail |
| Collect a multi-world cohort | `python3 scripts/run_avoidance_matrix.py` | Serial scored runs with retained failures and provenance |
| Read existing results | `./geoff results` | Local browser inspection; no simulation required |
| Explore pickup and carry | See [payload training](PAYLOAD_POLICY_TRAINING.md#reproduce) | Experimental magnetic transport using an explicit local checkpoint |

Historical `social-waypoints` and `social-endurance` cases use oracle inputs.
They help diagnose the command chain, but their outcomes do not establish
onboard perception performance. See the [suite reference](DEMO_SUITE.md) for
their distinct prerequisites and acceptance rules.

## 2. Launch an onboard visual session

```bash
./geoff worlds
./geoff demo plaza --people 8 --duration 120 --seed 44
```

The supported host uses Docker Compose v2 and an X11/XWayland display with
`/dev/dri`. ROS 2 and MuJoCo run inside the container. The first launch can
include substantial build and asset-generation time. Run one simulator at a time.

Once a normal build has completed, reuse it:

```bash
./geoff demo courtyard --people 2 --duration 300 --fast-start
```

Omit `--fast-start` after changing ROS packages or simulator sources. Duration is
in seconds; build, startup and shutdown add overhead. Press **Ctrl+C** to stop
the session. `--headless` disables RViz, but the standalone MuJoCo viewer may
still need X11.

Friendly world names are `courtyard`, `cafe`, `station`, `clinic`, `concourse`
and `office`; `plaza` selects the original scene. Small rooms default to two
people, medium rooms to four, and large rooms/plaza to eight. Set `--people`
explicitly for comparisons. See [world definitions](AVOIDANCE_EXPERIMENTS.md#representative-world-definitions).

## 3. Read the two viewers

| View | Look for | Interpretation |
| --- | --- | --- |
| MuJoCo | Robot motion, walking actors and physical scene | The simulated physical behavior |
| MuJoCo goal ring | Green ring, arrow and goal-distance annotation | Viewer-only projection of the estimated navigation goal; arrival tolerance is 0.50 m |
| RViz map and LiDAR | Measured returns and an expanding occupancy map | Online observation and mapping rather than a supplied scene map |
| RViz person tracks | Estimated tracks and predicted trajectories | Predictions from observed motion, not actor destinations |
| RViz routes and commands | Planned route, candidate motion and final decision | How navigation responds to perceived free space and people |

The MuJoCo annotation has no collision or sensor geometry. RViz's camera
previews are visualization inputs for navigation; the experimental transport
pipeline has its own RGB-D perception. The [visual runbook](ONBOARD_VISUAL_DEMO.md)
explains display controls and the [media catalogue](MEDIA.md) identifies the
recorded examples.

Visual sessions sample observed-free goals continuously. Near arrival, the next
goal is prepared and rechecked before handoff. Waiting can reflect unavailable
free space, stale observations or a planner stop; it should be interpreted using
the logs. Contacts are recorded and do not end a visual episode. Falls or data
loss can trigger a fresh episode, so uninterrupted session time is not equivalent
to uninterrupted robot endurance.

## 4. Move from demonstration to measurement

List the suite cases and inspect commands before launching:

```bash
./scripts/run_demo_suite.sh --list
./scripts/run_demo_suite.sh --suite full --headless --dry-run
```

For one scored onboard trial after building:

```bash
./scripts/run_demo_suite.sh --case onboard-waypoints \
  --goals 3 --people 8 --seed 44 --headless --fast-start
```

A strict pass requires more than motion or zero reported contacts: the trial
must have valid data, completed goals and satisfy the configured validator.
The suite stops on the first failed case by default. `--continue-on-failure`
collects further diagnostics while retaining failed status and a nonzero exit.

For repeated multi-world work, use the [matrix runbook](AVOIDANCE_EXPERIMENTS.md#collect-a-cohort).
The default matrix requests six worlds × three seeds, four people and three goals
per trial. It has a different people count from the eight-person example above.

## 5. Find and review the artifacts

| Run type | Default location |
| --- | --- |
| Visual session | `ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite/visual_session_<timestamp>/` |
| Suite | `ros2_ws/src/dynamic_obstacle_benchmarks/results/demo_suite/suite_<UTC timestamp>/` |
| Avoidance matrix | `ros2_ws/src/dynamic_obstacle_benchmarks/results/avoidance_matrix/` |
| Transport | `ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_demo/` |
| Published portable evidence | `docs/results/` |

Visual sessions have a `session.json` and per-episode artifacts. Suite runs
record `manifest.json`, `status.json` and case-specific logs. Open the results
dashboard for published datasets:

```bash
./geoff results
# Visit http://127.0.0.1:8765
```

New raw suite logs are not automatically equivalent to a published dashboard
dataset. Use the collection workflow for the relevant experiment and retain its
manifest. Continue with [Reviewing results and evidence](RESULTS_REVIEW.md).

## Troubleshooting

| Symptom | Next step |
| --- | --- |
| Docker permission denied after installation | Log out and back in to activate Docker group membership |
| Viewer cannot open | Check the host X11/XWayland session and installation requirements; `--headless` only suppresses RViz |
| A changed package appears to have no effect | Re-run without `--fast-start` |
| Viewers close and reopen | Inspect the visual session's episode stop reason; automatic restart follows falls/data-health failures |
| Robot stops or a goal does not advance | Inspect data freshness, goal distance and `next_goal_ready` in the episode logs |
| Command exits nonzero with saved results | Read status and validator failures; collection can complete while acceptance fails |
| Transport policy path is missing | Train/export or supply the explicit checkpoint described in the payload guide; local training outputs are not bundled assets |

For reproducible issue reports, include the command, world, seed, policy identity,
manifest and relevant stop/failure reason. See [Contributing](../CONTRIBUTING.md).
