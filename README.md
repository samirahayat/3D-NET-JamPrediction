# 3D-NET-JamPrediction
Repository to evaluate the value of jam prediction using drones against jam detection alone

## Overview

This project is an **agent-based traffic simulation** built on the [MESA](https://mesa.readthedocs.io/) framework. It models a grid of city streets where vehicles spawn at the edges, navigate through signal-controlled junctions, and exit at the opposite side. **Monitoring drones** fly above the network to detect and predict traffic jams, reporting in real time to a ground station and to junction controllers that adapt their signal timing accordingly.

The simulation runs in the browser at **http://localhost:8521** with an interactive canvas grid and tunable parameter sliders.

---

## How to Run

```
python main.py
```

Then open **http://localhost:8521**. Use the sidebar sliders to adjust parameters and press **Reset** to restart with new values. The **seed** is printed to the console on every run so you can reproduce the exact same simulation by passing it back.

---

## Architecture

The project consists of **9 Python modules**, each with a clear responsibility:

```
main.py                 Entry point — launches the MESA server
model.py                TrafficModel — owns the grid, scheduler, simulation loop
car.py                  CarAgent — moving vehicles with NaSch velocity model
cells.py                Passive grid cells (RoadCell, JunctionCell, ApproachLight)
controllers.py          JunctionController — adaptive traffic-light logic
drone.py                DroneAgent — flying monitors with jam detection/prediction
ground_station.py       GroundStation — aggregates drone reports
grid_builder.py         Procedural road-network generator
visualization.py        Browser-based rendering and UI sliders
```

### Dependency flow

```
main.py
  └─ visualization.py  (UI, sliders, server)
       └─ model.py     (creates everything)
            ├─ grid_builder.py (road layout, junctions, lights)
            │    ├─ cells.py
            │    └─ controllers.py
            ├─ car.py
            ├─ drone.py
            └─ ground_station.py
```

---

## Module-by-Module Description

### `main.py`

The entry point. Calls `run_server()` from `visualization.py` to start the MESA `ModularServer`.

---

### `model.py` — TrafficModel

The central simulation model. Responsibilities:

- **Grid setup**: Creates a `MultiGrid` of configurable width × height and delegates road/junction construction to `grid_builder.py`.
- **Agent creation**: Spawns the ground station, monitoring drones, and an initial batch of 30 cars.
- **Simulation loop** (`step()`): Each tick runs a **3-phase schedule** plus bookkeeping:
  0. **Road event** — at tick 10, spawn a `RoadEvent` obstacle on a random lane near the grid edge (once only).
  1. **Spawn** — attempt to spawn new cars at grid edges (×3 attempts).
  2. **Reserve** — create a fresh `_reserved_targets` set.
  3a. **All agents `step()`** — each car computes its intended `next_pos` (reading the grid but not writing).
  3b. **Deadlock resolution** — `_resolve_junction_deadlocks()` detects cyclic blockages inside junctions and executes coordinated rotation (see below).
  3c. **All agents `advance()`** — each car applies its move to the grid.
  4. **Ground-truth jam scan** — `_scan_junctions_for_jams()` computes objective jam state for every junction (independent of drones), used for monitoring-efficiency metrics.
  5. **Remove exited cars** — cars that have left the grid edge are removed.
- **Junction deadlock resolution** (`_resolve_junction_deadlocks()`): After all cars have called `step()`, the model inspects every junction for cyclic deadlocks.  It gathers all stuck cars (those with a recorded `_desired_junction_move`), builds a directed "blocked-by" graph, and walks chains to find closed loops.  When a cycle is found, `_execute_cycle_rotation()` simultaneously moves every car in the cycle one position forward along the ring — like a coordinated swap.  Phase 1 clears all `holder.car` for cycle cells, Phase 2 places each car at its desired position and sets `next_pos = pos` (making `advance()` a no-op for those cars).
- **Route planning** (`compute_route()`): Computes a list of `(junction_id, turn)` waypoints that guide a car from entry to exit. Uses knowledge of the corridor grid structure (horizontal and vertical centres) to create at most 2-turn routes.
- **Exit-road check** (`junction_exit_has_free_road()`): Scans a junction's exit edge in a given direction and returns `True` if at least one road cell is free.  Used by approaching cars to avoid entering when the exit is blocked.
- **BFS pathfinding** (`find_junction_exit_path()`): BFS through junction cells toward an exit road.  Supports a `require_free_exit` parameter to prefer free exit lanes when multiple lanes exist in the same direction.
- **Road event spawning** (`_spawn_road_event()`): At tick 10, picks a random exit lane and places a `RoadEvent` obstacle 3 cells before the grid edge.  Validates that the chosen cell is a road cell in the correct direction and is currently unoccupied.  If all candidates are blocked, retries on the next tick.
- **Seed management**: If no seed is provided, one is generated from `time.time()`. The seed is printed to the console and displayed in the browser metrics panel for reproducibility.

**Key parameters** (all adjustable via UI sliders):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `width` / `height` | 40 | Grid dimensions in cells |
| `spawn_prob` | 0.08 | Probability of spawning a car at each entry cell per tick |
| `lanes_choice_prob` | 0.6 | Probability of 2 lanes per direction (vs 1) |
| `green_duration` | 12 | Ticks a GREEN phase lasts |
| `yellow_duration` | 4 | Minimum YELLOW phase length |
| `yellow_max_wait` | 25 | Maximum YELLOW before forced switch |
| `num_horizontal_roads` | 2 | East/West corridors |
| `num_vertical_roads` | 2 | North/South corridors |
| `num_drones` | 2 | Monitoring drones deployed |
| `drone_speed` | 2 | Cells a drone moves per tick (= 20 m/s at 10 m/cell) |
| `drone_comm_range` | 25 | Drone-to-drone communication range (cells) |
| `hover_ticks_min` | 10 | Shortest hover duration per junction visit (ticks = seconds) |
| `hover_ticks_max` | 40 | Longest hover duration per junction visit (ticks = seconds) |
| `cell_size_m` | 10.0 | Physical size of one grid cell (metres) |
| `tick_duration_s` | 1.0 | Physical duration of one simulation tick (seconds) |
| `jam_density_threshold` | 0.30 | Minimum fraction of approach cells occupied for a jam to be declared |
| `jam_speed_threshold` | 0.40 | Jam when mean speed < threshold × v\_max (default: speeds below 1.6 cells/tick) |
| `jam_queue_min` | 1 | Minimum contiguous queue length at stop line required to declare a jam |
| `jam_sustain_ticks` | 3 | Ticks trigger must persist (pure green) before a jam event opens |
| `v_max` | 4 | Max vehicle velocity (NaSch model, cells/tick) |
| `random_slow_prob` | 0.3 | Random deceleration probability |

**Junction conflict avoidance:** a drone picking its next destination skips any junction currently hovered over or targeted by a peer drone, spreading the fleet across the network.  If a late-arriving drone finds a peer already hovering at the same junction, it immediately re-routes to the next free junction.  Fallback to plain next-in-sequence ensures drones are never stranded when every junction is occupied.
| `road_priority` | NONE | H / V / NONE — used when both directions are congested |
| `seed` | auto | Random seed for reproducibility |

---

### `cells.py` — Passive Grid Cells

Four agent types that represent physical infrastructure (they don't act, only exist for rendering and querying):

- **`RoadCell`**: One cell of a one-way lane. Has a `direction` (N/S/E/W) and a `car` slot for quick occupancy checks.
- **`JunctionCell`**: One cell inside a junction block. Knows its `junction_id` and which `valid_directions` originally overlapped there.
- **`ApproachLight`**: Visual traffic-light indicator placed one cell before a junction entrance. Its colour is derived at render time from the linked controller's phase and the approach type (H or V).
- **`RoadEvent`**: A road obstacle (accident, roadwork) placed on a single road cell.  Blocks that cell so cars cannot pass through it.  If a parallel lane exists, cars merge into it; otherwise they queue behind the obstruction.

---

### `grid_builder.py` — Road Network Generator

The `build_road_network()` function populates the model grid with a **grid-of-streets** layout:

- **Horizontal corridors** run East/West at configurable y-positions.
- **Vertical corridors** run North/South at configurable x-positions.
- **Junctions** form where corridors cross — rectangular blocks of `JunctionCell` agents.
- Lane counts are randomised per corridor (1 or 2 lanes per direction, controlled by `lanes_choice_prob`).
- **Traffic lights** (`ApproachLight`) are placed around each junction entrance.
- **Entry/exit cells** are identified at grid edges where roads begin and end.
- Exit cells are grouped into **exit groups** (adjacent lanes on the same corridor edge), and each group is assigned a distinct colour for visual identification.

Returns a network dictionary containing roads, junctions, controllers, entry/exit cells, corridor centres, and exit groups.

---

### `car.py` — CarAgent

Vehicles that traverse the road network. The model splits each tick into three phases to prevent move-order bias:

1. **`step()`** — all agents compute their intended `next_pos` in parallel (reading the grid but not yet writing).
2. **Deadlock resolution** — the model detects cyclic blockages inside junctions and executes coordinated rotation so all cycle members advance simultaneously.
3. **`advance()`** — all agents apply their move (writing to the grid).

**Three movement modes** (computed during `step()`):

1. **On a normal road** — Follow the Nagel-Schreckenberg cellular automaton rules (see below).
2. **Approaching a junction** — Obey the traffic light. Only enter on GREEN for the matching approach (H or V) **and** only if the exit road in the car's desired direction has at least one free cell. If the exit is fully blocked, the car waits outside, letting the jam propagate backward along the approach road rather than gridlocking the junction (see **Exit-Road Gate** below).
3. **Inside a junction** — Navigate toward the correct exit using BFS pathfinding through junction cells, based on the car's `intended_turn` (straight / left / right / uturn). When there are multiple lanes in the same exit direction, BFS prefers lanes with a free exit road so cars don't wait behind a jammed lane when a parallel lane is open (see **Free-Lane BFS** below).

**Nagel-Schreckenberg (NaSch) microscopic model:**

Each car has a `velocity` attribute (cells/step). On normal road cells, the following four rules are applied each tick:

1. **Acceleration**: If `v < v_max`, increase `v` by 1.
2. **Braking**: If there is a vehicle or obstacle within `v` cells ahead, reduce `v` to `gap - 1`.
3. **Random slowing**: With probability `random_slow_prob`, reduce `v` by 1 (models driver hesitation).
4. **Movement**: Advance `v` cells forward.

This produces realistic stop-and-go traffic waves without explicit scripting.

**The NaSch `_scan_ahead_distance()` method treats a junction cell as an obstacle**, so a car approaching a junction will naturally brake to `v = gap` where `gap` is the number of free road cells before the junction. This is how the NaSch model interacts with junction entry (CASE B).

**Junction approach and velocity behaviour:**

Cars do slow down when approaching and passing through junctions. The velocity transitions work as follows:

| Situation | Velocity behaviour |
|-----------|-------------------|
| **On open road** | NaSch rules: accelerate up to `v_max`, brake for obstacles, random slowing |
| **Approaching junction (NaSch scan)** | `_scan_ahead_distance()` counts a junction cell as an obstacle — the car brakes to `gap` cells before the junction and enters CASE B |
| **At red light (CASE B)** | `velocity = 0` — car waits |
| **Exit road blocked (CASE B)** | `velocity = 0` — light is green but exit road full; car stays put (see Exit-Road Gate below) |
| **Entering junction on green (CASE B)** | `velocity = 1` — car enters at low speed |
| **Inside junction (CASE A)** | Moves 1 cell per tick via BFS pathfinding (no NaSch rules inside the junction) |
| **Exiting junction onto road** | `velocity = 1` — re-enters normal road from near-stop, then NaSch rules resume and the car gradually accelerates back toward `v_max` |

So **cars naturally slow down** as they approach a junction: the NaSch braking rule reduces their speed to match the shrinking gap, and upon entering/exiting the junction they are capped at velocity 1.

**Exit-Road Gate — preventing junction gridlock:**

Before entering a junction, a car checks whether the exit road on the other side of the junction is free in its desired exit direction. If every exit road cell in that direction is occupied, the car does **not** enter — it stays on the approach road with `velocity = 0`, even though the light is green.

**Worked example:** Car A is heading West, approaching a junction. Its intended turn is `straight`, so it wants to exit West. The junction has two West-bound exit road cells: `(10, 15)` and `(10, 16)`. Both have cars on them.

```
← ← ← [A]  │ junction │ [car][car] ← ← ←
             │  block   │
```

Car A checks `junction_exit_has_free_road(jid, "W")` → `False`. So although the light is GREEN and the entry cell in the junction is empty, A stays put. The result:

- Cars behind A queue up on the eastbound approach road — the jam propagates backward organically.
- The junction stays clear for cross-traffic (North/South) to flow through.
- As soon as one of the West exit cells clears, `junction_exit_has_free_road` returns `True` and A enters normally.

Without this gate, A would enter the junction, discover it can't exit, and spend many ticks sidestepping internally — wasting junction capacity that cross-traffic could use.

**Free-Lane BFS — choosing the open lane:**

When there are two (or more) lanes running in the same direction, one lane's exit road may be jammed while the other is free. The BFS pathfinding uses a two-pass strategy to handle this:

1. **First pass** (`require_free_exit=True`): Find the shortest geometric path through the junction to an exit road cell that is **free** — even if that means routing to the farther lane. Junction cells along the way may be occupied; the car will navigate around them tick-by-tick.
2. **Fallback pass** (`require_free_exit=False`): If no free exit exists in any lane, fall back to the nearest geometric exit (even if occupied), and use sidestep/wait/yield logic to work toward it.

**Worked example:** A car is inside a junction, heading West. There are two W-bound lanes:

```
                         ┌──────────────────────┐
    W lane 1 (jammed):   │ [car] ← junction     │ ← [car][car][car]...
    W lane 2 (free):     │ [   ] ← junction     │ ← [   ][   ][   ]...
                         └──────────────────────┘
```

- **Old behaviour**: BFS finds the nearest W exit (lane 1) first, returns that path. Car waits behind the jammed lane, ignoring the free lane one row over.
- **New behaviour**: BFS pass 1 skips lane 1's occupied exit, continues searching, and finds lane 2's free exit. Returns a path leading the car to the free lane. The car moves across the junction toward lane 2 and exits promptly.

**Route planning**: Each car is assigned an exit group at spawn time. The model computes a route — a sequence of `(junction_id, turn_type)` waypoints. At each junction, the car checks whether it matches the next waypoint and takes the prescribed turn, or goes straight to reach the waypoint junction.

**Junction navigation — inside the junction (detailed):**

Once inside a junction, a car follows an escalating strategy each tick. The steps below are tried in order; the first one that succeeds determines the car's `next_pos`.

| Step | Trigger | Action |
|------|---------|--------|
| **0a. Hard exit cap** | `junction_time > 20` | Exit-focused only: try direct exit → BFS paths requiring free exit roads (len=1 handled as immediate exit) → `_try_exit_any_road` → stay put. No sidestepping. |
| **0b. Force straight** | `junction_time > 15` | Override `intended_turn` to `"straight"` regardless of route. |
| **1. Direct exit** | Always | Step onto the adjacent exit road cell if it is free. |
| **2. BFS path** | Always | Two-pass BFS: first with `require_free_exit=True` (prefer a free lane), then fallback with `require_free_exit=False`. Follow the path's next step if the cell is free. |
| **2a. Yield-or-proceed** | BFS target blocked by a junction mate | The car with the **lower `unique_id`** *yields* (stays put, no sidestep), giving the higher-id car a stable obstacle to navigate around. This breaks mutual-sidestep oscillation. The cycle detector (Phase 2 of the tick) handles genuine circular deadlocks where yielding alone is insufficient. |
| **2b. Sidestep** | BFS target blocked, car has priority | `_try_sidestep()` — move to a free adjacent junction cell that still has a valid BFS path to the same exit. |
| **3. Fallback** | `wait_counter > 8` | Force `intended_turn` to `"straight"`. |
| **4. Emergency** | `wait_counter > 4` | `_try_emergency_move()` — move to any free junction neighbour, or exit onto any adjacent road regardless of direction. |
| **5. Stay put** | All above failed | Reserve current cell, set `next_pos = pos`. |

The `_desired_junction_move` attribute records the blocked BFS target cell each tick.  This is the input data that the model's `_resolve_junction_deadlocks()` reads between `step()` and `advance()` to detect and break cyclic deadlocks via coordinated rotation.

The ordering follows a principle of escalating desperation — the cheapest, most correct action is tried first, and progressively more aggressive fallbacks kick in only after the car has been stuck longer:

Steps 0a/0b (time caps) come first because they're safety overrides. A car stuck for 15+ ticks is clearly in trouble — no point trying a complex turn when going straight is almost certainly the only viable exit. At 20+ ticks, the car is in crisis mode: it strips away all internal manoeuvring (no sidesteps) and focuses exclusively on getting out. These fire early to prevent the later steps from wasting more ticks on a doomed plan.

Step 1 (direct exit) is next because it's the ideal outcome — the car is already adjacent to its exit road and can just leave. No pathfinding needed, no conflict resolution. Checking this first avoids unnecessary BFS computation.

Step 2 (BFS path) is the main navigation engine. It only runs if the car can't exit immediately. The two-pass free-lane strategy goes here because it's the car's best-informed decision — it uses global junction topology to find the optimal route.

Steps 2a/2b (yield-or-proceed + sidestep) are sub-cases of Step 2 that only matter when the BFS target is blocked by another car. The yield rule (lower ID stays put) goes before sidestepping because staying still is cheaper and more predictable than moving to a different cell — it gives the other car a stable grid to plan around. Sidestepping is only allowed for the car with priority (higher ID), and only if a valid alternative path exists.

Step 3 (fallback to straight at 8 ticks) is a soft reset. If the car has been waiting 8 ticks for a turn that isn't working, the turn itself is probably the problem (e.g. the left-turn exit is permanently blocked). Forcing straight is a route compromise — the car will miss its intended turn but can try again at the next junction. This is placed after yield/sidestep because those should resolve transient blockages within a few ticks; 8 ticks of failure suggests a structural problem.

Step 4 (emergency at 4 ticks of wait_counter) allows any move — even exiting onto a wrong-direction road. This seems like it should come later than Step 3, and it does in the sense that it's a more extreme action. But note the different counters: Step 3 triggers on wait_counter > 8 while Step 4 triggers on wait_counter > 4. In practice, Step 3 fires first (at tick 9) and resets wait_counter to 0. Step 4 only fires if even going straight doesn't work for another 5 ticks — meaning the car is truly boxed in.

Step 5 (stay put) is the last resort. The car reserves its current cell so no other car tries to move into it, and waits for the deadlock resolver (Phase 2 of the tick) to potentially rotate it out of a cycle.

In short: exit > planned path > yield > sidestep > change plan > panic > freeze. Each escalation trades route optimality for a higher chance of actually moving.

---

**Road Events — obstacle avoidance and lane merging:**

A `RoadEvent` is a single-cell obstacle (accident / roadwork) placed on a lane near the grid edge.  Cars must never occupy the event cell.  The feature adds four mechanisms:

1. **NaSch obstacle detection** — `_scan_ahead_distance()` treats a `RoadEvent` exactly like a stopped car: the gap calculation breaks when it sees one, so the NaSch velocity model automatically decelerates toward the obstruction.

2. **Diagonal lane-change** (`_try_event_lane_change()`) — Before the NaSch rules execute, each car checks whether an event lies within `EVENT_DETECT_DIST = 6` cells ahead.  If so, it looks for a **parallel lane** in the same direction (one cell forward + one cell lateral = diagonal move).  If the diagonal target is a same-direction `RoadCell`, is free, and has no event, the car moves there (velocity reset to 1) and continues on the open lane.

3. **Conditional merge-zone yielding** (`_in_event_merge_zone()` + `_cars_blocked_behind_event()`) — Cars on the *free* parallel lane that are within `MERGE_ZONE_DEPTH = 8` cells of the obstruction periodically yield (velocity = 0, stay put) **only if at least one car is actually queued behind the event** on the blocked lane.  If no car is stuck there, the free lane flows at full speed.  When yielding is active it occurs every `YIELD_INTERVAL = 3` ticks (`tick_count % 3 == 0`), creating gaps so blocked cars can merge.

4. **Junction exit prevention** — The four junction-exit code paths (`_try_direct_exit`, `_try_emergency_move`, `_try_exit_any_road`, and the hard-cap BFS len=1 road step) all check `_cell_has_event()` before committing, so a car cannot exit a junction directly onto a blocked cell.

**Key constants:**

| Constant | Value | Meaning |
|----------|-------|---------|
| `EVENT_DETECT_DIST` | 6 | How far ahead a car detects an event (cells) |
| `MERGE_ZONE_DEPTH` | 8 | How many cells before the event trigger yield behaviour on the free lane |
| `YIELD_INTERVAL` | 3 | Cars on the free lane yield every N ticks |
| `EVENT_OFFSET` | 3 | How many cells before the grid edge the event is placed (in model.py) |

**Worked example — dual-lane Eastbound road with event:**

```
Grid edge (exit) is on the right.  Event placed 3 cells from edge on lane 1.

Tick 10 (event spawns):
  Lane 1 (y=5): [___][car A v=3][___][___][___][✖ EVENT][___][___][exit →]
  Lane 2 (y=6): [___][___][___][car B v=2][___][___][___][___][exit →]

Tick 11:
  Car A (lane 1) scans ahead: gap = 3 cells (event at distance 4).
  Event within EVENT_DETECT_DIST (6) → tries diagonal lane change.
  Diagonal target (1 forward, 1 lateral) on lane 2: free, same direction, no event → moves diagonally.
  Lane 1 (y=5): [___][___][___][___][___][✖ EVENT][___][___][exit →]
  Lane 2 (y=6): [___][___][car A v=1][car B v=2][___][___][___][___][exit →]

Tick 12 (YIELD_INTERVAL tick, 12 % 3 == 0):
  No car is blocked behind the event on lane 1 → car B does NOT yield.
  Both cars advance using normal NaSch rules.

Alternate scenario — car C is stuck behind event on lane 1:
  Lane 1 (y=5): [___][___][___][car C v=0][___][✖ EVENT][___][___][exit →]
  Lane 2 (y=6): [___][___][car A v=2][car B v=2][___][___][___][___][exit →]
  Tick 12: car B detects car C blocked behind event → yields (v=0).
  This creates a gap for car C to merge diagonally into lane 2.
```

If only **one lane** exists in that direction, no lane change is possible.  Cars simply queue behind the event (NaSch gap = 0 when adjacent) and wait.

---

### `controllers.py` — JunctionController

Controls traffic flow through one junction block. Cycles through four phases:

```
H_GREEN → H_YELLOW → V_GREEN → V_YELLOW → (repeat)
```

- **GREEN**: Cars from the matching approach (H = East/West, V = North/South) may enter.
- **YELLOW**: No new entries. The controller waits for the junction to clear (or until `yellow_max_wait` is exceeded) before switching.

**Adaptive signal control** — via `apply_congestion_response()`:

When a drone reports congestion data (from the ground station or directly via 5G), the controller adjusts timing:

- **One direction congested**: Extend green for the congested approach; shorten for the other.
- **Both directions congested**: Use `road_priority` to decide which direction gets extended. If priority is `NONE`, neither is favoured (extension stays at 0).
- **Prediction-based**: If predicted inflow for the opposite direction is high, the current green is shortened pre-emptively to switch sooner.

The green extension is capped at ±50% of the base `green_duration`.

---

### `drone.py` — DroneAgent

Monitoring drones that fly continuously from junction to junction, performing inspection and reporting via a 5G network.

**Behaviour cycle** (each tick):

1. **Share information** with nearby peer drones (jam weights, visit timestamps).
2. **If hovering** (`_hover_remaining > 0`): Stay at the current junction, inspect and report each tick, decrement the hover counter.  When the counter reaches 0, pick the next target.
3. **If flying**: Move toward the target junction (Manhattan path, at `drone_speed` cells/tick).  **On arrival**: begin a **4-tick hover** (`HOVER_TICKS = 4`) — inspect the junction, transmit the first report, and set `_hover_remaining = 3`.  The subsequent 3 ticks continue inspecting at the same junction, giving multiple observations for reliable detection.
4. **Deliver reports** instantly to the ground station AND directly to the junction controller (5G — no range limitation).

**Next-junction selection** uses a priority score:

```
score = jam_weight × staleness
```

Where `jam_weight` increases when jams are found (learned over time) and `staleness` is ticks since the last visit. Drones coordinate to avoid targeting the same junction.

**Jam detection** uses a **majority-vote multi-criteria AND trigger** applied only during the pure-green phase (`H_GREEN` for E/W, `V_GREEN` for N/S — yellow excluded):

| Criterion | Threshold | Purpose |
|---|---|---|
| **Density** | ≥ `jam_density_threshold` (default 0.30) | Rejects empty-road false positives |
| **Mean speed** | < `jam_speed_threshold × v_max` (default 1.6 cells/tick) | Confirms traffic is genuinely slow |
| **Queue length** | ≥ `jam_queue_min` (default 1) | Requires at least one car backed up to the stop line |

All three criteria must hold simultaneously for a direction to be flagged as jammed. This avoids the false-positive problem of low-flow metrics.

**Majority-vote confirmation (new):** a direction is only confirmed jammed after `drone_min_detect_votes` (default 2) **consecutive** pure-green scan ticks all meeting the three-criterion trigger.  Any criteria failure (including non-green phase ticks between scans) resets the vote streak to zero.  This models camera-confirmation latency and eliminates single-tick transients.  The vote counter (`_hover_detect_votes`) is reset to zero on every new junction arrival so votes from a previous visit cannot carry over.

**Congestion prediction (updated — temporal Δk):** fires during green or yellow when density is moderate and either speed is degrading **or** the density is **worsening** since the last reported observation stored at the ground station (`temporal_dk = current_density − prev_gs_density > 0.02`).  This replaces the previous spatial inner/outer Δk split with a true temporal gradient.  The GS preserves the previous observation per junction; the drone reads that snapshot on each scan tick to compute the gradient, honouring the architecture in which **drones inspect-and-forget while the ground station retains history for decision support**.

**Per-drone prediction cooldown (new):** each drone independently tracks the last tick it issued a prediction per `(junction_id, direction)` in `_pred_cooldown`.  Other drones visiting the same junction/direction during drone A's cooldown window are **not** blocked — they each contribute their own independent prediction record.  The model-level guard only prevents predicting a direction that is already an active confirmed jam.

**Macroscopic metrics computed per approach direction:**

- **Density (k)**: vehicles / segment length
- **Velocity (v)**: average speed of cars in the segment
- **Flow (q)**: k × v
- **Net accumulation (Δk)**: q_in − q_out (positive = traffic building up)

**How Δk (net accumulation) is calculated — worked example:**

The drone scans up to `MACRO_SCAN_DEPTH = 15` cells outward from the junction edge for each approach direction. The segment is split into two halves:

- **Inner half** (cells 1–7, closest to the junction) — represents the **outflow** side (cars about to enter/leaving).
- **Outer half** (cells 8–15, far from the junction) — represents the **inflow** side (cars arriving).

For each half, the drone computes a **local flow** = (local density) × (local avg velocity).

```
q_in  = (outer_cars / outer_length) × (outer_velocity_sum / outer_cars)
q_out = (inner_cars / inner_length) × (inner_velocity_sum / inner_cars)
Δk    = q_in − q_out
```

**Example**: A drone inspects junction J_15_6 from the East approach (cars heading West toward the junction). It scans 15 cells along the Eastbound road:

```
Cells scanned (East approach, 2 edge lanes):
  Lane 1: [car v=0][car v=0][car v=1][car v=2][___][___][___] | [car v=3][car v=4][___][car v=3][___][___][___][___]
  Lane 2: [car v=0][car v=1][___][___][___][___][___]         | [car v=2][___][___][car v=4][___][___][___][___]

  Inner half (cells 1–7):  5 cars, total velocity = 0+0+1+2+0+1 = 4, length = 14 cells (7 per lane × 2 lanes)
  Outer half (cells 8–15): 4 cars, total velocity = 3+4+3+2+4 = 16, length = 16 cells (8 per lane × 2 lanes)

  q_out = (5/14) × (4/5)   = 0.357 × 0.8   = 0.286
  q_in  = (4/16) × (16/4)  = 0.250 × 4.0   = 1.000
  Δk    = 1.000 − 0.286     = 0.714  (positive → traffic building up)
```

A positive Δk means cars are arriving faster than they leave — congestion is building. A negative Δk means traffic is draining. The drone flags a direction for predicted congestion if `Δk > 0.15` or `estimated_inflow > 0.3`.

**Key constants** (in `drone.py`):

| Constant / Parameter | Value | Meaning |
|----------|-------|--------|
| `SCAN_DEPTH` | 6 | Queue-counting scan range (cells from junction edge) |
| `MACRO_SCAN_DEPTH` | 15 | Macroscopic segment scan range (density / velocity / flow) |
| `JAM_WEIGHT_INCREMENT` | 0.5 | Patrol-weight bonus added when a jam is found at a junction |
| `BASE_WEIGHT` | 1.0 | Default patrol weight for a junction |
| `drone_min_detect_votes` | 2 (model param) | Consecutive passing scan ticks required to confirm a jam |
| `_hover_detect_votes` | `{dir: int}` per drone | Per-direction vote counter; reset on every new junction arrival |
| `_pred_cooldown` | `{(jid,dir): tick}` per drone | Per-drone prediction cooldown; does not block other drones |

---

## Jam Detection vs Jam Prediction — Explained with Examples

These are two fundamentally different questions a drone asks when it hovers over a junction:

| | **Jam Detection** | **Jam Prediction** |
|---|---|---|
| **Question** | "Is there a jam **right now**?" | "Is a jam **about to form**?" |
| **Time horizon** | Present (this tick) | Near future (next few ticks) |
| **Phase gate** | Pure green only (`H_GREEN`/`V_GREEN`) | Green + yellow (`_approach_has_green`) |
| **What it measures** | Density, mean speed, stop-line queue | Density (softer threshold), speed, temporal Δk from GS history |
| **Confirmation rule** | Majority vote: ≥ 2 consecutive passing scan ticks | Single passing tick, per-drone cooldown |
| **Trigger thresholds** | density ≥ 0.30 **AND** mean\_speed < 1.6 **AND** queue ≥ 1 | density ≥ 0.18 AND (mean\_speed < 2.56 OR **temporal\_Δk > 0.02**) AND queue ≥ 1 |
| **Cooldown** | Vote streak reset on any failure or new junction arrival | Per-drone `_pred_cooldown` per (jid, direction); other drones unaffected |
| **Action taken** | Ground station marks junction jammed; controller extends green for congested direction | Controller pre-emptively shortens green for the *opposite* direction so congested traffic gets a head start |

### Concrete scenario

Imagine a junction with four approach roads: North, South, East, West. A drone arrives and hovers for 4 ticks, inspecting every tick.

#### Tick 1 — Jam **detected** on East approach

```
                 N approach (2 cars, v=2, v=3)
                     ↓  ↓
           ┌─────────────────────┐
  W road   │      junction       │  E approach: [car v=0][car v=0][car v=0][car v=1][car v=0][___]
  (clear)  │      block          │               ←──── 5 cars in 6 cells ────→
           └─────────────────────┘
                     ↑  ↑
                 S approach (1 car, v=3)
```

The drone scans the East approach (15 cells back from junction edge):

**Multi-criteria check** (pure green required):
- Density = 5 cars / 6 cells = **0.83 ≥ 0.30** ✓
- Mean speed = (0+0+0+1+0) / 5 = **0.2 < 1.6** ✓
- Queue = 5 consecutive cars at stop line = **5 ≥ 1** ✓

All three criteria pass → direction **E** flagged as `jammed`.
The junction controller extends **H\_GREEN** so the East queue gets more time to drain.

#### Tick 1 — Congestion **predicted** on North approach

At the same time, the drone scans the North approach:

**Multi-criteria jam check**: Only 2 cars in the near segment, density = 2/15 ≈ 0.13 < 0.30 → density criterion fails → **no jam**.

**Prediction check** (softer thresholds, green + yellow allowed):
- Density = 0.13 ≥ 0.18 (60% of 0.30)? No — in this example density is marginal.
- However, outer-half scan shows fast-arriving cars: q\_in = 1.50, q\_out = 0.36, **Δk = 1.14 > 0.05** → **PREDICTED CONGESTION**

The drone flags direction **N** as `predicted_congestion`. The controller shortens the opposite green pre-emptively so the North approach gets a head start.

#### Summary of the two mechanisms

```
   JAM DETECTION (present)              JAM PREDICTION (future)
   ─────────────────────                ──────────────────────
   "5 cars queued at the                "4 fast cars approaching
    East entrance, all                   from far North — they'll
    stopped"                             arrive in 3-4 ticks and
                                         pile up if we don't
   → Extend East/West green              switch soon"
     to drain the queue
                                        → Shorten the current
                                          phase so North gets
                                          green before they arrive
```

### How this connects to the exit-road gate

The exit-road gate (described in the car.py section above) works **independently** of drone-based jam detection — it is a per-car local decision:

```
  Car A heading West → junction → West exit road FULL

  WITHOUT exit-road gate:          WITH exit-road gate:
  ────────────────────             ───────────────────
  A enters junction despite        A STAYS on approach road.
  blocked exit. Gets stuck         Cars behind A queue up
  inside, wastes junction          naturally on the East
  capacity for 5-20 ticks.         approach road.
  Cross-traffic (N/S) is           Junction stays clear for
  blocked by A sitting             N/S cross-traffic.
  in the junction.                 When the W exit clears,
                                   A enters and exits in
                                   2-3 ticks.
```

The drone may later fly over and **detect** the backward-propagating jam on the East approach road (queue ≥ 4). The controller then extends the E/W green phase to let that queue drain faster. This is how the two systems complement each other: the exit-road gate prevents the symptom (junction gridlock), and the drone + controller address the cause (imbalanced signal timing).

---

### `ground_station.py` — GroundStation

A fixed agent placed at the lower-left corner of the grid. Responsibilities:

- **Receive reports** from drones — stores the latest snapshot per junction (`junction_status`), including queue lengths, macroscopic segment metrics, and predictions.
- **Track jam history** — counts jam events, maintains active-jam set, records detection delays.
- **Learn monitoring weights** — junctions with more jams get higher priority weights (slowly decayed over time).
- **Notify controllers** — pushes congestion and prediction data to junction controllers so they can adapt signal timing. (Note: drones also push directly via 5G, so this serves as a redundant channel.)
- **Expose metrics** — update frequency, junction staleness, monitoring efficiency.

**Data structure — `JunctionReport`:**

A lightweight data object filed by drones on each inspection, containing:
- `junction_id`, `tick`, `is_jammed`
- `queue_lengths` — per-direction queue counts
- `segment_metrics` — per-direction density, velocity, flow, Δk
- `predictions` — per-direction estimated inflow, car count, avg velocity

**Jam persistence at the ground station (important):**

The ground station keeps a junction marked as "jammed" in `active_jams` **indefinitely** until a drone explicitly reports `is_jammed = False` for that junction. There is **no automatic expiry timer** at the ground station level. This means:

- If a drone reports junction J_15_6 as jammed at tick 20, and no drone visits J_15_6 again until tick 100, the ground station **still considers it jammed** for all 80 intervening ticks.
- The `active_jams` dict is only cleared for a junction when `receive_report()` is called with `is_jammed = False`.
- The ground station's `_notify_controllers()` continues to push the stale "jammed" status to the junction controller every tick (thus making it urgent to have up-to-date info)

This is deliberately different from the **visualization**, where junction colours revert to normal after 10 ticks without a fresh drone report. The colour reset is purely visual — it tells the user "this data is stale, a drone hasn't been here recently." The ground station, by contrast, takes the conservative approach: **assume the jam persists until proven otherwise**.

| Layer | Jam expiry behaviour |
|-------|---------------------|
| **Visualization** (junction colour) | Reverts to yellow after 10 ticks without a fresh drone report |
| **Ground station** (`active_jams`) | Persists until a drone reports `is_jammed = False` |
| **Controller** (signal timing) | Keeps adapting based on GS data, which does NOT expire |

---

### `visualization.py` — Browser UI

Defines how each agent type is drawn on the MESA `CanvasGrid` and launches the `ModularServer`.

**Agent rendering layers** (lowest to highest):

| Layer | Agent | Appearance |
|-------|-------|------------|
| 0 | `RoadCell` | Grey cell with direction arrow. Exit cells coloured by group. |
| 1 | `JunctionCell` | Colour reflects drone-reported status (see below). |
| 2 | `ApproachLight` | Small circle: green / yellow / red. |
| 3 | `CarAgent` | Coloured circle matching its exit group. |
| 3 | `RoadEvent` | Red square with ✖ (obstacle marker). |
| 4 | `DroneAgent` | Small square with 3 states: orange ◎ (scanning/hovering), red ⚠ (jam detected within 3 ticks), blue ✈ (flying). |
| 4 | `GroundStation` | Green square labelled "GS". |

**Junction colour coding** (based on drone reports within the last 10 ticks):

| Colour | Meaning | Text |
|--------|---------|------|
| Pale yellow `#FFF0B3` | Normal — no issues | — |
| Pale pink `#FFB3B3` | Real jam present (ground truth) **not yet seen by any drone** | ⚠ + direction arrows |
| Amber `#FFD480` | Congestion **predicted** by drone (no confirmed jam) | P + directional arrows |
| Coral red `#FF8888` | Jam **detected** by drone | Directional arrows |
| Bright red `#FF6666` | Both detected AND predicted | Arrows + P-arrows |

The pale-pink state makes undetected real jams visible at a glance, showing the gap between ground truth and drone knowledge.  Junction colours for drone-based states revert to normal after 10 ticks without a fresh drone report.

**Metrics panel** (right side of the browser) shows:
- Tick count, drone count, ground station update frequency
- Actual jams (ground truth) and monitoring efficiency
- Junction staleness
- Per-drone status: flight target, junctions visited, jams detected
- Seed, v_max, random slowing probability, road priority
- Car count and average velocity
- Per-junction macroscopic summary (average density k̄ and flow q̄)
- Per-junction drone detection / prediction indicators with directional arrows

**Sidebar sliders** allow real-time tuning of all model parameters listed in the model section above.

---

## Communication Architecture

```
                    ┌─────────────┐
                    │   Drone 1   │──────┐
                    └──────┬──────┘      │ 5G (instant)
                           │             │
    peer-to-peer    ┌──────┴──────┐      │
    weight sharing  │   Drone 2   │──┐   │
                    └─────────────┘  │   │
                                     │   │
              ┌──────────────────────┼───┘
              │                      │
              ▼                      ▼
    ┌──────────────────┐   ┌─────────────────────┐
    │  Ground Station  │   │ Junction Controller  │
    │  (aggregation,   │   │ (adaptive signal     │
    │   metrics,       │   │  timing adjustment)  │
    │   redundant      │   │                      │
    │   push to ctrls) │   │                      │
    └──────────────────┘   └─────────────────────┘
```

Drones deliver reports **simultaneously** to the ground station and directly to the junction controller via 5G. There is no range limitation — the 5G network provides full coverage. The ground station also pushes data to controllers as a redundant channel.

---

## Traffic Model Summary

### Microscopic Level (Nagel-Schreckenberg)

Each individual vehicle has a velocity that evolves according to four deterministic + stochastic rules each tick: accelerate, brake for obstacles, randomly slow down, then move. This produces realistic emergent traffic phenomena (shockwaves, stop-and-go) from simple local rules without scripted scenarios.

### Macroscopic Level (Density-Velocity-Flow)

Drones compute aggregate metrics over road segments approaching each junction:
- **Density (k)** = cars / segment length
- **Average velocity (v)** = mean speed of cars in segment
- **Flow (q)** = k × v
- **Net accumulation (Δk)** = q_in − q_out

The segment (up to 15 cells from the junction edge) is split into an inner half (near-junction, outflow side) and an outer half (far-from-junction, inflow side). Each half gets its own local flow computed as `(local_density) × (local_avg_velocity)`. The difference `q_in − q_out` indicates whether traffic is building up (positive) or draining (negative). See the detailed worked example in the drone.py section above.

These are used both for jam detection (high density + low velocity = jam) and for prediction (high inflow density approaching from far away = future congestion).

### Adaptive Control

Junction controllers receive real-time congestion data and predictions from drones and adapt signal timing every tick.  The full pipeline has four stages:

#### Stage 1 — Data collection (drone → `JunctionReport`)

Each time a drone finishes its camera-processing window and enters the active-scan phase it assembles a `JunctionReport` containing:
- `queue_lengths` — per-direction vehicle queues (cells from stop line)
- `segment_metrics` — macroscopic density, average velocity, flow, temporal Δk per direction
- `predictions` — per-direction `estimated_inflow` from the prediction rule

The report is pushed **directly to the junction controller** via `_push_to_controller()` (instant 5G) and also forwarded through the **Ground Station** as a redundant channel.

#### Stage 2 — GS congestion classification (`_notify_controllers()`)

Every tick `GroundStation._notify_controllers()` iterates over all `junction_status` entries and builds a `congested_approaches` set (`{'H'}`, `{'V'}`, or `{'H', 'V'}`) using **two independent criteria**:

| Criterion | Rule |
|---|---|
| **Queue-based** | `queue ≥ JAM_THRESHOLD` for any approach direction |
| **Macroscopic** | segment `density ≥ jam_density_threshold` **AND** `avg_velocity ≤ jam_velocity_threshold` |

Any direction satisfying either criterion contributes its axis (`'H'` for E/W, `'V'` for N/S) to the set.  The GS then calls `ctrl.apply_congestion_response(congested_approaches, predictions, road_priority)` for the junction's controller.

**Important:** the GS never expires its jam state autonomously — it conservatively holds the last reported status until a drone explicitly clears it with `is_jammed = False`.  The controller therefore keeps adapting even between drone visits.

#### Stage 3 — Detection-based extension (`apply_congestion_response()`)

The controller computes `max_ext = green_duration // 2` (±50 % of the base green, e.g. ±6 ticks with `green_duration = 12`), then sets `_green_extension` according to which directions are congested:

| Situation | `_green_extension` |
|---|---|
| Only **current** direction congested | `+max_ext` — extend to drain the queue |
| Only **opposite** direction congested | `−max_ext` — shorten to switch sooner |
| **Both** congested, current = priority road | `+max_ext` — favour the priority direction |
| **Both** congested, current ≠ priority road | `−max_ext` — accelerate hand-off to priority |
| **Both** congested, priority = `'NONE'` | `0` — neutral, no change |
| Yellow phase at call time | No change — extension applied at next GREEN |
| No congestion reported | `_green_extension = 0` — immediate reset |

The effective green duration enforces a minimum of 4 ticks:
```
effective_green = max(4, green_duration + _green_extension)
```

#### Stage 4 — Prediction-based pre-emptive shortening (`_apply_prediction_adjustment()`)

After the detection-based extension is set, if the report included inflow predictions, the controller checks the **opposite** direction's `estimated_inflow`.  If `max_inflow > 0.3` **and** that direction is **not already confirmed congested**:
- The current green is shortened by an additional `green_duration // 4` ticks.
- `_prediction_switch = True` is flagged.

This is the **proactive** layer: the signal switches early to give the about-to-be-congested direction a head start, *before* a jam is confirmed.

#### Automatic reset at phase boundary

`_green_extension` and `_prediction_switch` are both zeroed the **exact tick the GREEN phase timer expires** inside `step()`.  Each cycle therefore starts fresh from the base `green_duration`; drone-reported adaptations must be re-applied in the new phase if congestion persists.  Yellow phase durations are never adapted — they always use `yellow_duration` (minimum) and `yellow_max_wait` (maximum force-switch).

#### Summary flow

```
Drone active-scan window
    │
    ├─ temporal Δk / density / queue → estimated_inflow in report
    │
    ├─ _push_to_controller()  (direct 5G, instant)
    │
    └─ pending_reports → GroundStation.receive_report()
              │
              ▼
    GS._notify_controllers()  (every tick)
        │  queue ≥ threshold  OR  density+velocity threshold
        ├─ congested_approaches = {'H'} / {'V'} / {'H','V'}
        │
        ▼
    JunctionController.apply_congestion_response()
        │
        ├─ Detection: ±(green_duration//2)  based on which side is congested
        │
        └─ Prediction: if opposite inflow > 0.3 and not yet confirmed congested
                       → shorten further by green_duration//4  (proactive switch)
        │
        ▼
    phase timer expires → _green_extension = 0  (automatic reset)
```

When both directions are congested, a configurable **road priority** setting (`'H'`, `'V'`, or `'NONE'`) determines which direction receives the extension.

---

## Reproducibility

Every simulation run prints its seed to the console:

```
[TrafficModel] seed = 452344087
```

To reproduce the exact same run with different parameter values, pass the seed back. The seed is also displayed in the browser metrics panel.

---

## File Size Reference

| File | Lines | Primary class/function |
|------|-------|----------------------|
| `main.py` | ~18 | Entry point |
| `model.py` | ~1556 | `TrafficModel` |
| `car.py` | ~1066 | `CarAgent` |
| `cells.py` | ~119 | `RoadCell`, `JunctionCell`, `ApproachLight`, `RoadEvent` |
| `controllers.py` | ~244 | `JunctionController` |
| `drone.py` | ~882 | `DroneAgent` |
| `ground_station.py` | ~231 | `GroundStation`, `JunctionReport` |
| `grid_builder.py` | ~431 | `build_road_network()` |
| `visualization.py` | ~900+ | `agent_portrayal()`, `run_server()` |
| `results_logger.py` | ~281 | `SimulationLogger` |

