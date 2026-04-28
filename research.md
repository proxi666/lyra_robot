# Lyra Research Direction

This document captures the forward-looking research direction for extending Lyra beyond its current ROS 2 navigation baseline.

Lyra today is a classical mobile robot stack built around:

- Gazebo home simulation
- iRobot Create 3 simulation support
- laser-based SLAM
- Nav2 planning and control

The research direction discussed for this repository is to extend that stack toward semantic navigation and, later, vision-language-action style navigation.

## Core Idea

The first target is not end-to-end voice control and not direct language-to-wheel velocity prediction.

The architecture we discussed is:

```text
camera + semantic perception -> semantic map layer
language query -> semantic target
semantic target -> Nav2 goal pose
Nav2 -> path planning + local control + cmd_vel
```

The key principle is:

- keep the geometric map and Nav2 costmaps for safe navigation
- add semantic understanding on top of the existing navigation stack

This means the robot still uses standard ROS 2 navigation for motion, while a semantic layer tells it what parts of the environment correspond to words like `sofa`, `fridge`, `kitchen`, or `door`.

## First Research Direction

The first clean research direction is:

### Semantic Map Plus Nav2 Integration

The idea is to build a semantic map layer on top of the normal geometric map.

Geometric map:

- free
- occupied
- unknown

Semantic layer:

- sofa here
- fridge here
- kitchen-like region here
- charging dock here

With that structure, language grounding becomes possible:

```text
"go to the sofa"
-> resolve "sofa" in semantic map
-> produce a goal pose near the sofa
-> send goal to Nav2
```

The first version should use the semantic map for goal grounding only.

## Meaning Of Semantic Grounding

Semantic grounding means translating a word or phrase into something the robot can act on.

Examples:

- `sofa` -> object location in the map
- `fridge` -> object location in the map
- `kitchen` -> semantic region in the map
- `charging dock` -> known docking location

Without this step, language remains text only. With grounding, language becomes a navigation target.

## Why This Direction Comes First

This direction is the most practical first step because:

- the current repo already has a strong navigation base
- Nav2 can remain responsible for safety and motion execution
- the new research contribution is well-defined
- it is easier to validate than a full end-to-end VLA controller

It also matches the current gaps in the repo:

- there is no RGB camera integration yet
- there is no semantic mapping layer yet
- there is no language grounding pipeline yet

## Stronger Second Step

After semantic goal grounding, the next stronger step is:

### Semantic Planning

In this stage, the semantic map affects path planning, not only goal selection.

Examples:

- `go to the sofa but avoid the kitchen`
- `go to the fridge through the hallway`
- `prefer open regions`
- `avoid human-occupied areas`

This can be implemented by modifying planning cost or path ranking using semantic information.

## Longer-Term Research Possibilities

We discussed three main parts of the navigation stack where future VLA or VLM ideas can enter.

### 1. Mapping / Localization

Possible direction:

- language-indexed semantic map
- open-vocabulary map labels
- object-anchored localization cues

Example contribution:

- a semantic map where the robot can query `sofa`, `bedroom`, `kitchen`, or `desk`

### 2. Planning

Possible direction:

- semantic costmap layer
- path re-ranking using semantic relevance
- instruction-aware global planning

Example contribution:

- planner prefers paths that better satisfy a language instruction rather than only shortest-path geometry

### 3. Control

Possible direction:

- learned local action critic
- VLA-inspired local subgoal generation
- short-horizon semantic control assistance on top of Nav2

This is the highest-risk path and should come after semantic mapping and planning.

## Practical Roadmap

The phased roadmap we discussed is:

1. Add camera perception to the current robot stack.
2. Build a semantic map aligned with the geometric map.
3. Ground language targets into goal poses.
4. Use Nav2 to execute those goals safely.
5. Extend the planner with semantic costs or path ranking.
6. Explore VLA-style local decision layers only after the above is working.

## Project Framing

The strongest way to frame Lyra as a long-term research project is:

`ROS 2 semantic indoor navigation with language grounding, built on top of a reliable Nav2 baseline`

Later, that can evolve into:

`vision-language-action inspired semantic navigation for mobile robots`
