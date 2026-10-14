# Demo media catalogue

<!-- geoff-nav:start -->
<p align="center">
  <a href="../README.md"><img src="media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../README.md#quick-start"><img src="media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="DEMO_GUIDE.md"><img src="media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="INDEX.md"><img src="media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="REPOSITORY_GUIDE.md"><img src="media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="PROJECT_STATUS.md"><img src="media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="PROJECT_PLAN.md"><img src="media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="RESULTS_REVIEW.md"><img src="media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="literature_review/README.md"><img src="media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="DEVELOPMENT.md"><img src="media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[README gallery](../README.md#recorded-demo-gallery) · [Demo walkthrough](DEMO_GUIDE.md)

The README now leads with three freshly captured locomanipulation GIFs under
`docs/media/locomanipulation/`. The five earlier navigation GIFs remain in a
collapsed historical gallery. All are actual window recordings, not generated
robot motion. Upstream training-library media belongs to its own documentation.

## Locomanipulation recordings — 14 October 2026

| Asset | Recorded content | Source outcome |
| --- | --- | --- |
| [Mission overview](media/locomanipulation/mission-overview.gif) | Survey, approach, pickup, lift, retreat and attempted delivery navigation | Incomplete: navigation succeeded outside fresh sensed staging tolerance |
| [Pickup and lift](media/locomanipulation/pickup-and-lift.gif) | Close-up reach, magnetic attachment, lift and retreat | Source run later fell during delivery navigation |
| [Loaded transport](media/locomanipulation/loaded-transport.gif) | Close-up retreat and loaded navigation from the overview run | Incomplete: navigation/localization check |

These zero-person clips show development behavior, **not a completed delivery**.
Fresh placement and restoration footage is still missing. Earlier complete-run
logs are available in the [performance report](PERFORMANCE_IMPLEMENTATION_20261012.md).
Attachment is the capture-radius weld fixture, not contact-only finger grasping.
Each animation ends with its source outcome. PNG stills with matching basenames
provide a nonanimated alternative.

The [recording manifest](media/locomanipulation/recordings.json) retains all seven
fresh capture attempts, including startup/settling failures and falls. It records
commands, policy/source manifests, result hashes, frame timestamps, crops, selected
frame hashes and output GIF hashes. Raw frames and logs remain in ignored
`ros2_ws/src/dynamic_obstacle_benchmarks/results/transport_recordings/` and
`transport_demo/`. Run IDs are identifiers; `recorded_at` records the capture date/time.

| Capture ID suffix | Configuration | Outcome |
| --- | --- | --- |
| `readme-dual` | Cycle-03, dual task, device clock | Stale-state braking failure during discovery |
| `readme-contact` | Cycle-05, red → red, device clock | Braking timeout during discovery |
| `readme-settling` | Cycle-05, red → red, bounded 5-second braking allowance | Picked up and retreated; fell during delivery navigation |
| `readme-delay` | Cycle-03, red → red, bounded 5-second braking allowance | Picked up and carried; navigation/localization failure |
| `readme-exploration` | Cycle-05, medium-building red → blue | Body instability before search |
| `readme-wall-clock` | Cycle-03, dual fixture, wall-clock policy cadence | Body instability during discovery |
| `readme-aisle` | Cycle-03, aisle fixture | Body instability during discovery |

Capture samples the live X11 MuJoCo window at 3 fps. Exports crop simulator UI,
resize, overlay phase/time labels and subsample into time-lapse GIFs; they do not
interpolate poses or generate missing task stages. Timestamps are capture wall
time and phase labels are the latest received mission state, not exact physics
alignment. The recording workload can affect scheduling, so these are neither a
controlled speed comparison nor a reliability cohort.

To capture another attempt, first ensure exclusive simulator access:

```bash
python3 scripts/record_transport_preview.py --run-id <unique-id> --fps 3 -- \
  --world transport_dual --task red:red --people 0 --seed 48 --duration 600 \
  --carry-posture payload_v1 --policy <repository-local-policy.onnx> --fast-start --no-rviz

python3 scripts/export_transport_gifs.py \
  --pickup-run 20261014T164500Z-readme-settling \
  --carry-run 20261014T164700Z-readme-delay
```

The exporter uses documented crops for the captured 960 × 600 window. Inspect
new frames and adjust crops before publishing a different window layout. Capture
has a 780-second outer bound and saves partial evidence during cleanup.

## Earlier navigation recordings

| Asset | View | Context |
| --- | --- | --- |
| [Plaza crowd](media/onboard-crowd.gif) | MuJoCo | Eight-person onboard visual demonstration |
| [Plaza decisions](media/onboard-decisions.gif) | RViz | Onboard sensing, tracking and planning visualization |
| [Courtyard crowd](media/courtyard/onboard-crowd.gif) | MuJoCo | Representative residential courtyard |
| [Courtyard decisions](media/courtyard/onboard-decisions.gif) | RViz | Courtyard perception and navigation visualization |
| [Station crowd](media/station/onboard-crowd.gif) | MuJoCo | Representative station entrance |

The plaza pair has a [recording manifest](media/recordings.json) containing the
source run path, source-result SHA-256, recording timestamp, frame counts and
frame rate. Each clip has 32 frames at 4 fps. The visual runbook identifies the
[courtyard session and handoff observation](ONBOARD_VISUAL_DEMO.md#results-and-verification).
The station preview is presented as a scene illustration; the plaza manifest
does not establish its provenance.

Short recordings do not establish endurance, collision-free behavior or the
behavior of later controller revisions. For numerical claims, cite a scored
dataset and its configuration using the [results-review guide](RESULTS_REVIEW.md).

## Dashboard and schematics

| Asset | Purpose |
| --- | --- |
| [Results dashboard](media/results-dashboard.png) | Screenshot of dataset filtering and recorded-trajectory replay |
| [Locomanipulation title banner](media/title-banner.svg) | Static, accessible title and observe → approach → pick up → carry → place schematic |
| [Overview banner](media/banner.svg) | Generated three-panel summary of the onboard demonstration |
| [Installation pipeline](media/install-pipeline.svg) | Setup stages and dependency flow |
| [Architecture](media/architecture.svg) | High-level system organization |
| [Command safety](media/command-safety.svg) | Command supervision concept |
| [Social navigation](media/social-navigation.svg) | Perception/planning concept |
| [Verification tiers](media/verification-tiers.svg) | Layers of implementation and integration checks |
| [Repository map](media/repository-map.svg) | Task-oriented folder overview; the [folder guide](REPOSITORY_GUIDE.md) provides the text equivalent |
| [Project progression](media/project-progression.svg) | Dated development milestones, with source and evidence in [project status](PROJECT_STATUS.md) |
| [Forward roadmap](media/project-roadmap.svg) | Eight acceptance-driven stages; detailed deliverables are in [the plan](PROJECT_PLAN.md) |
| [Research programme](media/research-programme.svg) | Planned P0–P7 studies aligned with G1–G5 and candidate papers; see [research delivery](RESEARCH_PROGRAMME.md) |

The SVGs are schematics, not sensor output. The dashboard screenshot is a static
example; launch `./geoff results` to inspect the checked-in datasets interactively.

The title banner was refreshed on 14 October for the locomanipulation workflow.
It is maintained directly in `title-banner.svg`, separately from the generated
overview diagrams. It is static and schematic; the GIF gallery supplies actual
recorded robot behavior.

The folder, progression and roadmap graphics use a shared two-column layout with
readable static content, descriptive SVG titles and reduced-motion support.
Their source text lives in `BOARDS` in the graphics generator. Status graphics
are dated snapshots: update their text alongside the status/plan pages rather
than treating animation or a colored label as a live progress indicator.

## Resource-library artwork

The [resource library](resources/README.md) uses a linked cover illustration and
eight original topic cards under [`media/resources/`](media/resources/). Each
picture links to an annotated collection: demos, navigation, manipulation,
experiments, research, upstream tools, development or history. Cards share the
README palette, remain readable without animation, and respect reduced-motion
preferences. Text links provide an alternative to image navigation.

`scripts/resource_library.py` defines their labels, descriptions and icons and
builds the complete research index from the existing verified bibliography.
The shared graphics generator regenerates these outputs; its `--check` mode
detects stale cards/catalogue content and missing or duplicated research keys.
Existing GIFs and screenshots are reused only with links back to their runbooks
or evidence guide. No third-party images are copied into the library.

## Shared navigation buttons

The README, maintained project guides and local package READMEs use the same
two-row menu: **Home · Quick start · Demos · Docs · Folders**, followed by
**Progress · Roadmap · Results · Research · Develop**. Labels, order, icons and
132 × 44 px image dimensions are consistent. Exact destination pages use a mint
border and an accessible current-page label; other links use cyan accents.

Each button is a local SVG image inside a normal HTML link. This makes the whole
image clickable in GitHub Markdown without JavaScript or embedded interactive
SVG links. Standard browser link focus and keyboard activation still apply.
Buttons wrap on narrow screens and remain readable with animation disabled.
The small border trace and indicator pulse respect `prefers-reduced-motion`.

Assets live in [`media/navigation/`](media/navigation/) and are generated from
[`scripts/readme_navigation.py`](../scripts/readme_navigation.py). The companion
GitHub profile is a separate site's landing page and retains its own navigation.
Historical archive contents and upstream READMEs retain their original context;
the archive index and local workspace guides use the shared menu.

To update all buttons and links, edit the shared `LINKS` definitions and run:

```bash
python3 scripts/generate_readme_graphics.py
python3 scripts/generate_readme_graphics.py --check
python3 scripts/check_docs.py
```

The generator only replaces content between a page's paired `geoff-nav` HTML
comments. Copy that block into a new maintained page and regenerate to calculate
the correct relative paths. Page content outside the markers is preserved.
The existing graphics check in CI also checks that menu blocks match their source.

## Updating the gallery

Keep asset paths relative so the README works on GitHub and in a local checkout.
Use descriptive alt text and captions explaining the scene and evidence category.
When replacing a recording, record the command, world, seed, source configuration,
source-result hash and whether the session was scored. Update the associated
manifest rather than letting old provenance describe new footage.

Keep clips short enough for a practical GitHub landing page and retain full raw
logs in the experiment artifact location. Review the rendered README when changing
image dimensions or table layout. Check maintained links and generated schematics:

```bash
python3 scripts/check_docs.py
python3 scripts/generate_readme_graphics.py --check
```
