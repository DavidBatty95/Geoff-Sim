# Grove-G1 review and applications to GeoffSim

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
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

Reviewed 2026-10-05 at
[`e158588eb6b99264247d64e2d811d48d704e0b67`](https://github.com/Adyansh04/grove-g1/tree/e158588eb6b99264247d64e2d811d48d704e0b67).
The review inspected README guides and controller/approach implementation in a
temporary checkout. Grove was not built or benchmarked on this host. No Grove
code or policy artifact was imported into GeoffSim.

## Main recommendation

Adopt the architectural separation and measured execution checks. Evaluate
WBC-AGILE as an alternative locomotion backend in isolation before deciding
whether further tuning of the current 29-action policy is the best investment.
Keep GeoffSim's sensor-only crowd benchmark and BrainCo hardware target as the
acceptance criteria. Grove's demonstrations are not evidence that those gates
have been met.

## Findings and concrete work

| Grove design and source | Relevance to GeoffSim | Action / gate |
| --- | --- | --- |
| [Single hardware writer and joint ownership](https://github.com/Adyansh04/grove-g1/blob/e158588eb6b99264247d64e2d811d48d704e0b67/workspace/src/g1_hardware_interface/README.md): one `ros2_control` component, atomic arm controller exchange, explicit hold controllers | Our compositor arbitrates arm targets, but system-wide controller ownership remains weaker | S1/S7: formalize command and joint ownership; prototype a Humble-compatible hardware adapter separately; require no unclaimed/dual-owned joint during transitions |
| [AGILE policy contract](https://github.com/Adyansh04/grove-g1/blob/e158588eb6b99264247d64e2d811d48d704e0b67/workspace/src/g1_controllers/src/agile_policy.cpp): 12 inputs, 10 outputs; 29 observed joints but 14 controlled joints; histories returned by the graph; absolute positions and gains | Particularly relevant to current failures while raising arms and stopping loaded turns. GeoffSim's actor instead has 480 observations/29 actions with external arm composition | S5: separate adapter with named mappings, strict tensor validation, reset/history tests and gain compatibility. Compare uprightness, displacement, stopping, arm transitions and payloads against cycle-01. Do not feed AGILE into the existing 480/29 adapter |
| [Measured approach, settle, recheck](https://github.com/Adyansh04/grove-g1/blob/e158588eb6b99264247d64e2d811d48d704e0b67/workspace/src/g1_locomotion/src/base_approach/approach_action.cpp) | GeoffSim's original docking accepted a moving threshold crossing, then settled without verifying it remained in reach | Implemented an independent stop/settle/recheck loop in the demo, with unchanged tolerances, fresh odometry and the original deadline. Test it natively before promotion |
| [Measured deadband and nudge control](https://github.com/Adyansh04/grove-g1/blob/e158588eb6b99264247d64e2d811d48d704e0b67/workspace/src/g1_locomotion/README.md) | Our fixed 0.30 m/s docking command can overshoot; low commands previously stalled | S1/S5: calibrate this policy's forward/lateral pulse and coast response under payload, then implement bounded nudges through the existing social supervisor. Grove's numeric gains belong to its different policy |
| [Manipulation phases and re-aiming](https://github.com/Adyansh04/grove-g1/blob/e158588eb6b99264247d64e2d811d48d704e0b67/workspace/src/g1_manipulation/README.md): measured settle, refresh before descent, release/retreat/reobserve | Our native tests exposed base drift after arm motion and a box outside the near camera view after release | Continue bounded measured redocking; retract/retreat before post-release observation; replace endpoint-only IK with swept collision-aware planning |
| [Persistent mission/held-state semantics](https://github.com/Adyansh04/grove-g1/blob/e158588eb6b99264247d64e2d811d48d704e0b67/workspace/src/g1_orchestration/README.md): skill actions, bounded mission macros, unknown possession, StopAll and status | Matches our needed transition from a script to a recoverable executive | S4: consolidate onto typed skill actions, persistent possession and resumable task state. Preserve the new cancel-and-await checks and uncertain-grasp semantics |
| [Perception separation](https://github.com/Adyansh04/grove-g1/blob/e158588eb6b99264247d64e2d811d48d704e0b67/workspace/src/g1_perception/README.md): masks → measured geometry/support → stable IDs → skills, capture stamps preserved | Our colored fixture has one ID per color and fixed surface/size assumptions | S3/S4: separate detection from geometry, estimate support planes, retain identities and uncertainty. Optional open-vocabulary detector must have a measured-input acceptance mode |
| [Camera-aware semantic world model](https://github.com/Adyansh04/grove-g1/blob/e158588eb6b99264247d64e2d811d48d704e0b67/docs/guides/world-model.md): camera coverage, target resolution and reachable viewpoints | Rotation-only discovery and close-range invisibility limit GeoffSim's autonomy | S4: calibrate real camera mounting/FOV, then choose observation viewpoints and approach poses from observed free space. Consider a canopy adapter after interface/evidence review |
| [VLA action-chunk gate](https://github.com/Adyansh04/grove-g1/blob/e158588eb6b99264247d64e2d811d48d704e0b67/workspace/src/g1_vla/README.md): check proposed chunks against the planning scene | Useful future learned-manipulation architecture | Later S3/S5 extension after physical BrainCo demonstration data and collision validation; not a prerequisite for dependable delivery |

## Evidence boundaries that matter

- Grove `main` uses ROS 2 **Jazzy**, C++20 and **Dex3-1** hands. GeoffSim uses
  **Humble** and targets **BrainCo**. Its hand geometry, size window, grip-effort
  thresholds, topics and controller gains are not directly transferable.
- Grove documents contact-only grasps in pinned manipulation/tabletop fixtures.
  Its navigation scene adds a contact-triggered **palm weld** for carrying. This
  is not contact-only mobile grasp acceptance.
- `perception:=true` defaults to a **mock detector whose masks use simulator
  truth**. A measured-depth pipeline downstream does not remove that oracle input.
  GeoffSim must preserve a separate non-oracle detector mode and source audit.
- The navigation example uses a committed map/localization. Compare that fairly
  against GeoffSim's online mapping conditions, not as an equivalent benchmark.
- The GR00T pipeline explicitly **does not grasp with the base checkpoint** and
  needs robot-specific demonstrations. It is not an available grasping solution.
- Shared DDS interfaces facilitate hardware migration, but Grove documents
  untested real-robot mode switching and timing. Hardware deployment remains a
  separate measurement/validation task in both projects.
- The inspected material does not establish GeoffSim-style multi-person social
  navigation acceptance. Group behavior, uncertainty-aware passing and repeated
  crowd transport remain our own gates.

## Revised priority order

1. Finish settled docking and action/evaluator correctness; retain native failures.
2. Calibrate the actual camera extrinsics/FOV against the intended physical sensor.
3. Compare locomotion backends under arm transitions and payload **before** another
   broad round of crowd-cost tuning or expensive policy training.
4. Add swept arm/table/person validation and calibrated BrainCo grip evidence.
5. Consolidate persistent skill orchestration and semantic/viewpoint state.
6. Requalify integrated transport and social navigation on held-out conditions.

Licensing: the reviewed root license is BSD-3-Clause; the AGILE policy and some
controller designs retain separate NVIDIA Apache-2.0 attribution. Any future
reuse must retain the applicable notices and pinned upstream identity.
