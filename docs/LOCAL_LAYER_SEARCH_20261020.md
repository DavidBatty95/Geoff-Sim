# Local refinement after the overnight search — 20 October 2026

The overnight worker accepted proposal 250 after a complete no-fall evaluation:
**197/208 passes**, improving on the initial 195/208. Eleven failures remain.
The search then went more than 1,300 proposals without another improvement;
many larger perturbations were rejected after early falls. Partial screens
are not qualified runs and are retained separately from the accepted result.

- [Accepted incumbent and complete 208-case evidence](results/layer_search_incumbent_20261020.json)
- [Declared local-refinement plan](results/local_layer_refinement_plan_20261020.json)

## Change in search strategy

The old worker was explicitly stopped, with its status and all artifacts retained.
The new worker starts from proposal 250's actual coefficients:
`[0.927732, 0.92538, 0.92884, 0.959266]`. Resume now verifies these coefficients
and both endpoint identities against the actor's manifest, and checks the
exported ONNX hash. It no longer assumes every starting actor uses 0.95 in all
four layers.

Coordinate refinement starts at 0.01 and restarts immediately after an accepted
improvement. Non-improving sweeps halve the step; after fine coordinate search,
the worker cycles Gaussian scales 0.0005/0.001/0.002/0.005/0.01 rather than
repeatedly applying the former 0.02 scale. The random sequence is deterministic.

Every candidate retains the full intended cumulative suite. A fall or a failure
count that cannot beat the incumbent causes explicit incomplete-screen rejection.
Only a complete no-fall result with fewer failures replaces the incumbent.
Zero cumulative failures trigger 48 newly declared validation conditions, and
validation failures are retained. No runtime policy is promoted automatically.

## Live operation

The active container is `geoff-native-layer-search-local`:

```bash
docker logs --tail 20 -f geoff-native-layer-search-local
docker stop geoff-native-layer-search-local
```

Live state: `ros2_ws/src/dynamic_obstacle_benchmarks/results/layer_search_local_20261020/status.json`.
Candidate artifacts: `vendor/mjlab/logs/payload_training/layer_search_local_20261020/`.
The worker stops for source changes, setup errors, validation falls requiring
review, explicit interruption or less than 10 GiB free disk. Existing artifacts
are never silently deleted to make room. These are native-suite experiments,
not claims of flawless physical-grasp or asynchronous ROS mission performance.

Verification: 68 focused tests passed, including actual-coefficient resume,
endpoint mismatch rejection, bounded proposals, deterministic scale cycling,
cumulative evidence reuse, and fail-closed screening. Documentation and
whitespace checks passed before launch.
