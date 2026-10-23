#!/usr/bin/env python3
"""Generate README graphics, shared navigation and the research resource index."""
import argparse
from html import escape
from pathlib import Path
from textwrap import wrap

from check_docs import maintained_documents
from readme_navigation import graphics as navigation_graphics, update_navigation
from resource_library import graphics as resource_graphics, paper_catalogue

ROOT = Path(__file__).resolve().parents[1]
PANELS = {
    "banner.svg": ("Geoff-Sim", "Sensor-only G1 navigation and an eight-person visual crowd", [
        ("8 independent walkers", "16 walk poses + relaxed idle", "Distance-driven gait; no robot yielding"),
        ("Onboard perception", "LiDAR + IMU + joint encoders", "Online SLAM; observed people only"),
        ("30-minute sessions", "MuJoCo goal + RViz decisions", "Experimental: contacts still occur")]),
    "architecture.svg": ("Onboard runtime", "Navigation receives observations; simulator truth is evaluation-only", [
        ("Sense / estimate", "Local LiDAR, IMU, encoders", "odom -> base_footprint; SLAM -> map"),
        ("Track / plan", "People tracks + CV forecasts", "Nav2 events up to 5 Hz; local 45 ms budget"),
        ("Execute / evaluate", "Supervisor -> policy -> DDS", "Truth goes to scorer/logger, not planner")]),
    "command-safety.svg": ("Command checks", "Layered command filtering is not a collision-free guarantee", [
        ("Navigation intent", "/cmd_vel -> social planner", "Swept body checks; selected trajectory"),
        ("Final command", "/cmd_vel_safe -> walking policy", "Freshness, limits, slew and response"),
        ("Actuation feedback", "Joint targets -> rt/lowcmd", "LowState + estimated odometry return")]),
    "install-pipeline.svg": ("Reproducible source setup", "Pinned sources and reviewed patches precede every clean build", [
        ("Checkout", "Clone recursively; configure .env", "Read access to parent and source forks"),
        ("Provision / build", "Apply source patches; rosdep", "Generate crowd assets; build ROS + MuJoCo"),
        ("Verify / launch", "Checksum-pinned official policy", "run_onboard_navigation_demo.sh")]),
    "social-navigation.svg": ("Dynamic social waypoints", "Goals and people are interpreted in the observed map", [
        ("Observe the crowd", "Green: tracked / amber: lost", "Bounded display persistence; real timestamps"),
        ("Choose motion", "Green path; blue/red alternatives", "Cyan: final safety output; visible reasons"),
        ("Advance goals", "Prepare a connected free-space target", "Switch at <= 0.50 m; wait if none is valid")]),
    "verification-tiers.svg": ("Verification and evidence", "Keep unit contracts, visual sessions and scored acceptance separate", [
        ("Fast / unit checks", "Syntax, assets, source patches", "Python + C++ regressions; CI artifacts"),
        ("Visual integration", "Eight walkers, sensors and HUD", "Unscored; contacts logged; episode restarts"),
        ("Scored acceptance", "Goal completion + collision gates", "Current eight-person avoidance still fails")]),
}

BOARDS = {
    "research-programme.svg": (
        "From research gaps to evidence and papers",
        "P0-P7 research delivery | Complements the S0-S7 engineering roadmap",
        [
            ("P0 / Protocol and novelty", "All gaps | Define the intended difference", "Audit nearest prior work; freeze study conditions, comparators and the analysis plan.", "PLANNED", "#43d2ec"),
            ("P1 / Measured response", "G1 | Identification and uncertainty", "Test policy response and occupancy coverage on held-out motions and operating conditions.", "PLANNED", "#43d2ec"),
            ("P2 / Causal diagnosis", "G2 | Oracle-to-onboard interventions", "Separate sensing, age and execution effects with matched encounters and failure traces.", "PLANNED", "#7de39a"),
            ("P3 / Calibrated planning", "G1 + G2 | First-paper priority", "Specify the method; compare collision-completion tradeoffs with strong matched baselines.", "PLANNED", "#7de39a"),
            ("P4 / Safety and liveness", "G3 | Distinct coordination mechanism", "Test veto, yield and resume decisions without shifting risk into contacts or falls.", "PLANNED", "#d3a5ff"),
            ("P5 / Articulated risk", "G4 | Body, arm and payload effects", "Establish when base-only models fail and whether added fidelity improves decisions.", "PLANNED", "#d3a5ff"),
            ("P6 / Human evaluation", "G5 | Context and perceived behavior", "Control speed, clearance and order; measure participant judgments and task outcomes.", "PLANNED", "#f4c978"),
            ("P7 / Replication and release", "Every paper | Claims linked to artifacts", "Repeat the evidence, audit analysis and publish methods, failures and limitations together.", "PLANNED", "#f4c978"),
        ],
        "P7 applies to each candidate paper. These are planned studies, not completed findings. See docs/RESEARCH_PROGRAMME.md.",
    ),
    "repository-map.svg": (
        "Find your way around Geoff-Sim",
        "A task-oriented map of the repository | Start with README.md and ./geoff",
        [
            ("01 / Run and automate", "scripts/ + geoff", "Demo launchers, world generators, training helpers and repeatable experiment commands.", "RUN A DEMO", "#43d2ec"),
            ("02 / Understand and review", "docs/", "Runbooks, architecture, project status, forward plan, literature review and published results.", "READ THE EVIDENCE", "#7de39a"),
            ("03 / Change robot behavior", "ros2_ws/src/", "ROS 2 perception, planning, locomotion, manipulation, messages and benchmark packages.", "DEVELOP THE STACK", "#43d2ec"),
            ("04 / Reproduce upstream changes", "patches/", "Reviewed simulator and MJLab source changes, pinned bases and checksum manifests.", "VERIFY THE SOURCE", "#d3a5ff"),
            ("05 / Build and verify", "docker/ + .github/", "Container dependencies, runtime setup, CI checks, issue forms and contribution workflow.", "CHECK THE BUILD", "#7de39a"),
            ("06 / Explore external systems", "vendor/ + integrations/", "Pinned training sources and the optional Grove-G1 profile with its own runtime boundary.", "FOLLOW THE BOUNDARIES", "#d3a5ff"),
            ("07 / Inspect runs", "tools/ + docs/results/", "Browser dashboard and portable datasets. Raw simulation logs stay in ignored result folders.", "COMPARE OUTCOMES", "#43d2ec"),
            ("08 / Explore the context", "archive/ + profile/", "Historical research notes and the companion GitHub profile. Current guidance lives in docs/.", "FOLLOW THE STORY", "#7de39a"),
        ],
        "Folder guide: docs/REPOSITORY_GUIDE.md | Source is versioned; builds, policies and raw logs are local artifacts.",
    ),
    "project-progression.svg": (
        "From simulation integration to delivery research",
        "Publication milestones and dated experiments | Snapshot reviewed 5 October 2026",
        [
            ("01 / Simulation foundation", "July 2026", "G1 integration, ROS interfaces and reproducible verification formed the initial platform.", "INTEGRATED", "#43d2ec"),
            ("02 / Demonstration workflows", "August 2026", "Locomotion, oracle social demos, manipulation interfaces and visual documentation expanded.", "DEMONSTRATED", "#43d2ec"),
            ("03 / Onboard experiments", "22 September 2026", "Six worlds and a results dashboard made onboard failures visible: v12 passed 4 of 18 trials.", "MEASURED", "#7de39a"),
            ("04 / Reliability investigation", "September development records", "Iteration datasets and alignment audits retained regressions. v84 passed 10 of 18 trials.", "ACCEPTANCE OPEN", "#f4c978"),
            ("05 / Payload transport", "25 September experiment report", "Two consecutive isolated fixture passes with a trained payload policy and magnetic attachment.", "ISOLATED EVIDENCE", "#d3a5ff"),
            ("06 / Published development baseline", "5 October 2026 | 3a7ceac", "Source, reviewed patches and evidence published together; regression checks documented.", "PUBLISHED", "#7de39a"),
        ],
        "These are different experiment conditions, not a single performance curve. See docs/PROJECT_STATUS.md.",
    ),
    "project-roadmap.svg": (
        "The route to people-aware indoor delivery",
        "Acceptance-driven stages | No promised dates or implied completion percentages",
        [
            ("S0 / Reproducible baseline", "Freeze source and runtime identity", "Repeat a clean build and native baseline with complete configuration and retained attempts.", "OPEN", "#43d2ec"),
            ("S1 / Trustworthy mission state", "Measurement and action lifecycle", "Close attachment ambiguity, cancellation, evaluator coverage and command ownership gaps.", "IN PROGRESS", "#43d2ec"),
            ("S2 / Reliable navigation", "Onboard sensing and avoidance", "Pass the full existing matrix, then held-out encounters without weakening the strict gates.", "OPEN", "#f4c978"),
            ("S3 / Physical finger grasp", "Dex3 contact-only integration", "Both-hand bench and ROS/DDS grasp tests pass; mobile carry and placement remain unqualified.", "IN PROGRESS", "#d3a5ff"),
            ("S4 / Recoverable missions", "Wait, reobserve, redock and resume", "Recover within bounded retries; preserve known object and command-authority state on failure.", "OPEN", "#d3a5ff"),
            ("S5 / Smooth loaded transport", "Loaded geometry and response", "Repeat empty-room deliveries across objects and layouts with declared motion tolerances.", "OPEN", "#d3a5ff"),
            ("S6 / Social transport", "Integrate S2 through S5", "Complete held-out deliveries around people with valid evaluation and no prohibited contacts.", "GATED", "#f4c978"),
            ("S7 / Physical deployment", "Calibrated G1 hardware adapters", "Validate the same contracts on the physical robot within a declared operating envelope.", "FUTURE", "#a8c1d6"),
        ],
        "S3 isolated work may run alongside S2. Promotion gates and deliverables: docs/PROJECT_PLAN.md.",
    ),
}


def board(title, subtitle, cards, footer):
    """Two-column cards remain readable as a standalone GitHub image."""
    rows = (len(cards) + 1) // 2
    height = 142 + rows * 194 + 38
    parts = [f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 {height}" role="img" aria-labelledby="title desc">
  <title id="title">{escape(title)}</title>
  <desc id="desc">{escape(subtitle)}. {escape(footer)}</desc>
  <style>
    text {{font-family:Arial,Helvetica,sans-serif}}
    .marker {{animation:glow 3s ease-in-out infinite}}
    @keyframes glow {{0%,100% {{opacity:.6}} 50% {{opacity:1}}}}
    @media (prefers-reduced-motion:reduce) {{.marker {{animation:none}}}}
  </style>
  <rect width="1200" height="{height}" rx="20" fill="#0b1727"/>
  <rect x="34" y="32" width="5" height="56" rx="2" fill="#43d2ec"/>
  <text x="54" y="56" fill="#edf6ff" font-size="29" font-weight="bold">{escape(title)}</text>
  <text x="54" y="87" fill="#a8c1d6" font-size="16">{escape(subtitle)}</text>
''']
    for index, (heading, path, body, label, color) in enumerate(cards):
        x, y = 34 + (index % 2) * 580, 120 + (index // 2) * 194
        parts.append(f'''  <rect x="{x}" y="{y}" width="552" height="174" rx="12" fill="#14273b" stroke="#29425b"/>
  <rect x="{x}" y="{y+17}" width="4" height="40" rx="2" fill="{color}"/>
  <text x="{x+20}" y="{y+32}" fill="{color}" font-size="21" font-weight="bold">{escape(heading)}</text>
  <text x="{x+20}" y="{y+58}" fill="#edf6ff" font-size="17">{escape(path)}</text>
''')
        lines = wrap(body, width=62)
        if len(lines) > 3:
            raise ValueError(f"Too much text in graphic card: {heading}")
        for line, text in enumerate(lines):
            parts.append(f'  <text x="{x+20}" y="{y+86+line*20}" fill="#b5c9dc" font-size="15">{escape(text)}</text>\n')
        parts.append(f'''  <circle class="marker" cx="{x+24}" cy="{y+151}" r="4" fill="{color}"/>
  <text x="{x+36}" y="{y+155}" fill="{color}" font-size="11" font-weight="bold" letter-spacing="1">{escape(label)}</text>
''')
    parts.append(f'  <text x="34" y="{height-26}" fill="#a8c1d6" font-size="13">{escape(footer)}</text>\n</svg>\n')
    return "".join(parts)


def graphic(title, subtitle, panels):
    parts = [f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 330" role="img" aria-labelledby="title desc">
  <title id="title">{escape(title)}</title>
  <desc id="desc">{escape(subtitle)}. Schematic illustration, not recorded benchmark footage.</desc>
  <style>
    text {{font-family:Arial,Helvetica,sans-serif}}
    .flow {{stroke-dasharray:8 9;animation:flow 1.4s linear infinite}}
    .pulse {{animation:pulse 2.4s ease-in-out infinite}}
    @keyframes flow {{to {{stroke-dashoffset:-34}}}}
    @keyframes pulse {{0%,100% {{opacity:.45}} 50% {{opacity:1}}}}
    @media (prefers-reduced-motion:reduce) {{.flow,.pulse {{animation:none}}}}
  </style>
  <rect width="1200" height="330" rx="18" fill="#0b1727"/>
  <text x="34" y="53" fill="#edf6ff" font-size="32" font-weight="bold">{escape(title)}</text>
  <text x="35" y="85" fill="#a8c1d6" font-size="17">{escape(subtitle)}</text>
''']
    colors = ("#43d2ec", "#7de39a", "#d3a5ff")
    for i, (heading, first, second) in enumerate(panels):
        x = 35+i*395
        parts.append(f'''  <rect x="{x}" y="119" width="340" height="144" rx="12" fill="#14273b" stroke="{colors[i]}"/>
  <circle class="pulse" cx="{x+22}" cy="146" r="5" fill="{colors[i]}"/>
  <text x="{x+38}" y="153" fill="{colors[i]}" font-size="21" font-weight="bold">{escape(heading)}</text>
  <text x="{x+18}" y="192" fill="#edf6ff" font-size="16">{escape(first)}</text>
  <text x="{x+18}" y="229" fill="#acc2d6" font-size="14">{escape(second)}</text>
''')
        if i < 2:
            parts.append(f'  <path class="flow" d="M{x+346} 192h40" stroke="#43d2ec" stroke-width="3"/><path d="M{x+379} 187l8 5-8 5" fill="none" stroke="#43d2ec" stroke-width="2"/>\n')
    parts.append('  <text x="35" y="302" fill="#8ca8c0" font-size="13">Simulation research prototype | Illustrative data flow | See DEMO_RESULTS.md for measured outcomes</text>\n</svg>\n')
    return "".join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    graphics = {name: graphic(*content) for name, content in PANELS.items()}
    graphics.update({name: board(*content) for name, content in BOARDS.items()})
    graphics = {str(Path('docs/media') / name): content for name, content in graphics.items()}
    graphics.update(navigation_graphics())
    graphics.update(resource_graphics())
    graphics['docs/resources/RESEARCH.md'] = paper_catalogue(ROOT)
    for name, expected in graphics.items():
        path = ROOT/name
        if args.check:
            if not path.exists() or path.read_text() != expected:
                raise SystemExit(f"Outdated generated asset or catalogue: {path}")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(expected)
    navigation_count = 0
    for path in maintained_documents(ROOT):
        text = path.read_text()
        expected = update_navigation(text, path, ROOT)
        if '<!-- geoff-nav:start -->' in text:
            navigation_count += 1
        if text != expected:
            if args.check:
                raise SystemExit(f"Outdated navigation: {path}")
            path.write_text(expected)
    print(f"README graphics, resource catalogue and navigation are current ({navigation_count} pages)")


if __name__ == "__main__":
    main()
