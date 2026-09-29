"""
visualization.py — MESA CanvasGrid portrayal and server launcher.

Defines how each agent type is drawn on the browser-based grid and
provides ``run_server()`` to start the interactive visualization.

The server sidebar exposes tunable parameters so you can experiment
with traffic density, light timing, lane counts, etc. and see how
they affect congestion in real time.
"""

from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.UserParam import Slider

from cells import RoadCell, JunctionCell, ApproachLight, RoadEvent
from controllers import JunctionController
from car import CarAgent
from model import TrafficModel
from drone import DroneAgent
from ground_station import GroundStation


# ═════════════════════════════════════════════════════════════════════
#  Agent portrayal — returns a dict that CanvasGrid uses to draw each
#  agent on the HTML <canvas>.
# ═════════════════════════════════════════════════════════════════════

# Direction → unicode arrow for road labels
_ARROWS = {"N": "↑", "S": "↓", "E": "→", "W": "←"}

# Colour applied to cells within the drone's communication range
_COMM_RANGE_ROAD_BG = "#C5DBE8"       # light blue-grey (replaces road grey)
_COMM_RANGE_JUNCTION_BG = "#D0E0F5"   # light blue (replaces junction yellow)

# Approach-lane jam highlights
_JAM_TRUTH_ROAD_BG     = "#FFCCCC"    # pale red  — real jam, not yet drone-detected
_JAM_DETECTED_ROAD_BG  = "#FF5555"    # bright red — drone confirmed jam on this lane
_JAM_TRUTH_JUNC_BG     = "#FFB3B3"    # pale pink  — real jam on junction, not detected

# How many road cells back from the junction edge to highlight as approach lanes
_APPROACH_VIS_DEPTH = 10

# Model attribute used to cache the approach-cell map across portrayal calls
_APPROACH_MAP_ATTR = "_vis_approach_cell_map"


def _build_approach_cell_map(model):
    """
    Build (and cache) a mapping of grid position → list of (jid, direction)
    for every road cell that lies within *_APPROACH_VIS_DEPTH* cells of a
    junction along each of its four approach directions.

    The result is stored on the model as ``_vis_approach_cell_map`` so it is
    computed only once per model instance (the road layout is static).
    """
    if hasattr(model, _APPROACH_MAP_ATTR):
        return getattr(model, _APPROACH_MAP_ATTR)

    _OPP  = {"N": "S", "S": "N", "E": "W", "W": "E"}
    _DIRS = ("N", "S", "E", "W")
    _DXY  = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}

    cell_map = {}  # (x, y) -> [(jid, approach_dir), ...]

    for jid, ctrl in model.junction_controllers.items():
        block_cells = list(ctrl.block_cells)
        if not block_cells:
            continue
        for approach_dir in _DIRS:
            opposite = _OPP[approach_dir]
            dx, dy   = _DXY[opposite]
            # Find the edge cells on the approach side of the junction block
            if opposite == "W":
                v = min(c[0] for c in block_cells)
                edge = [(x, y) for x, y in block_cells if x == v]
            elif opposite == "E":
                v = max(c[0] for c in block_cells)
                edge = [(x, y) for x, y in block_cells if x == v]
            elif opposite == "S":
                v = min(c[1] for c in block_cells)
                edge = [(x, y) for x, y in block_cells if y == v]
            else:  # "N"
                v = max(c[1] for c in block_cells)
                edge = [(x, y) for x, y in block_cells if y == v]

            for ex, ey in edge:
                for step in range(1, _APPROACH_VIS_DEPTH + 1):
                    rx, ry = ex + dx * step, ey + dy * step
                    if not (0 <= rx < model.width and 0 <= ry < model.height):
                        break
                    # Only continue while a matching RoadCell exists
                    has_road = any(
                        isinstance(a, RoadCell) and a.direction == approach_dir
                        for a in model.grid.get_cell_list_contents([(rx, ry)])
                    )
                    if not has_road:
                        break
                    cell_map.setdefault((rx, ry), []).append((jid, approach_dir))

    setattr(model, _APPROACH_MAP_ATTR, cell_map)
    return cell_map


def _in_comm_range(pos, model):
    """Return True if *pos* is within ANY drone's communication range."""
    drones = getattr(model, "drones", [])
    if not drones or pos is None:
        return False
    comm_range = getattr(model, "drone_comm_range", 0)
    if comm_range <= 0:
        return False
    comm_sq = comm_range * comm_range
    for drone in drones:
        if drone.pos is None:
            continue
        dx = pos[0] - drone.pos[0]
        dy = pos[1] - drone.pos[1]
        if dx * dx + dy * dy <= comm_sq:
            return True
    return False


def _drone_junction_status(jid, model):
    """
    Aggregate the latest drone inspection data for junction *jid*.

    Returns ``(detected_dirs, predicted_dirs, is_recent)``:

    * **detected_dirs** — set of approach directions where a jam was detected.
    * **predicted_dirs** — set of directions where congestion is predicted.
    * **is_recent** — True if the report was filed within the last 10 ticks.
    """
    tick = getattr(model, "_tick_count", 0)
    drones = getattr(model, "drones", [])

    detected = set()
    predicted = set()
    is_recent = False

    for drone in drones:
        info = drone.last_inspection.get(jid)
        if info is None:
            continue
        age = tick - info.get("tick", -1)
        if age <= 10:
            is_recent = True
            detected |= info.get("jam_directions", set())
            predicted |= info.get("predicted_congestion_dirs", set())

    return detected, predicted, is_recent


def agent_portrayal(agent):
    """
    Return a portrayal dictionary for *agent*.

    Layer order (higher = drawn on top):
      0 — RoadCell  (grey background with direction arrow)
      1 — JunctionCell  (pale yellow square)
      2 — ApproachLight  (small circle: green / yellow / red)
      3 — CarAgent  (coloured circle indicating intended turn)
    """

    # ── Road surface ─────────────────────────────────────────────
    if isinstance(agent, RoadCell):
        # Highlight exit cells with their assigned colour so you can
        # visually match a car's colour to its destination cell.
        bg = "#D0D0D0"
        txt_color = "black"
        pos = agent.pos
        model = agent.model
        if hasattr(model, "_exit_color_map") and pos in model._exit_color_map:
            bg = model._exit_color_map[pos]
            txt_color = "white"
        else:
            # Check whether this cell is an approach lane for a jammed junction.
            # Priority (highest first):
            #   1. Drone confirmed jam on this approach direction → bright red
            #   2. Ground-truth jam on this approach direction    → pale red
            #   3. Inside drone comm range                        → blue-grey
            #   4. Normal road grey
            approach_map = _build_approach_cell_map(model)
            entries = approach_map.get(pos, [])
            if entries:
                is_detected_jam = False
                is_truth_jam    = False
                for jid, approach_dir in entries:
                    jam_state = model.junction_jam_state.get(jid, {})
                    if approach_dir in jam_state.get("jam_directions", []):
                        is_truth_jam = True
                    detected_dirs, _, _ = _drone_junction_status(jid, model)
                    if approach_dir in detected_dirs:
                        is_detected_jam = True
                if is_detected_jam:
                    bg        = _JAM_DETECTED_ROAD_BG
                    txt_color = "white"
                elif is_truth_jam:
                    bg        = _JAM_TRUTH_ROAD_BG
                    txt_color = "#990000"
                elif _in_comm_range(pos, model):
                    bg = _COMM_RANGE_ROAD_BG
            elif _in_comm_range(pos, model):
                bg = _COMM_RANGE_ROAD_BG
        return {
            "Shape": "rect", "Filled": True, "Layer": 0,
            "w": 1, "h": 1, "Color": bg,
            "text": _ARROWS.get(agent.direction, ""),
            "text_color": txt_color,
        }

    # ── Junction interior ────────────────────────────────────────
    if isinstance(agent, JunctionCell):
        # Colour priority (highest first):
        #   Bright red  — jam DETECTED by drone (+ predicted)
        #   Coral red   — jam DETECTED by drone only
        #   Pale pink   — real jam NOT YET detected by any drone
        #   Amber       — congestion PREDICTED by drone (no confirmed jam)
        #   Blue-grey   — inside drone comm range
        #   Pale yellow — normal
        color = "#FFF0B3"  # default pale yellow
        text = ""
        text_color = "#CC0000"

        jid   = agent.junction_id
        model = agent.model
        detected, predicted, is_recent = _drone_junction_status(jid, model)

        # Ground-truth jam state (independent of drone detection)
        jam_state      = model.junction_jam_state.get(jid, {})
        truth_jam_dirs = set(jam_state.get("jam_directions", []))
        is_truth_jammed = bool(truth_jam_dirs)

        if detected and predicted:
            color = "#FF6666"  # bright red — detected + predicted
        elif detected:
            color = "#FF8888"  # coral red — drone detected
        elif is_truth_jammed:
            color = _JAM_TRUTH_JUNC_BG  # pale pink — real jam, drone hasn't seen it
        elif predicted:
            color = "#FFD480"  # amber — congestion predicted
        elif _in_comm_range(agent.pos, model):
            color = _COMM_RANGE_JUNCTION_BG

        # Build label text
        # • Drone-detected dirs          → plain arrows  (e.g. "↑→")
        # • Drone-predicted-only dirs    → "P" + arrows  (e.g. "P↓")
        # • Ground-truth, not detected   → "⚠" + arrows (e.g. "⚠←")
        parts = []
        if detected:
            parts.append(
                "".join(_ARROWS[d] for d in ("N", "S", "E", "W")
                        if d in detected)
            )
        if predicted:
            pred_only = "".join(
                _ARROWS[d] for d in ("N", "S", "E", "W")
                if d in predicted and d not in detected
            )
            if pred_only:
                parts.append("P" + pred_only)
        if is_truth_jammed and not detected:
            undetected_dirs = "".join(
                _ARROWS[d] for d in ("N", "S", "E", "W")
                if d in truth_jam_dirs
            )
            parts.append("\u26A0" + undetected_dirs)  # ⚠ + direction arrows
        text = " ".join(parts)

        if predicted and not detected and not is_truth_jammed:
            text_color = "#CC6600"  # orange for prediction only
        elif is_truth_jammed and not detected:
            text_color = "#880000"  # dark red for undetected real jam

        return {
            "Shape": "rect", "Filled": True, "Layer": 1,
            "w": 1, "h": 1, "Color": color,
            "text": text, "text_color": text_color,
        }

    # ── Traffic-light indicator ──────────────────────────────────
    if isinstance(agent, ApproachLight):
        color = _light_color(agent)
        return {
            "Shape": "circle", "Filled": True, "Layer": 2,
            "r": 0.18, "Color": color,
        }

    # ── Car ──────────────────────────────────────────────────────
    if isinstance(agent, CarAgent):
        color = _car_color(agent)
        return {
            "Shape": "circle", "Filled": True, "Layer": 3,
            "r": 0.35, "Color": color,
        }

    # u2500─ Drone ───────────────────────────────────────────────
    if isinstance(agent, DroneAgent):
        # Scanning (hovering) = orange with radar symbol
        # Red flash if a recent inspection found a jam
        # Blue when flying
        tick = getattr(agent.model, "_tick_count", 0)
        is_scanning = agent._hover_remaining > 0
        recent_jam = False
        for _jid, info in agent.last_inspection.items():
            if info.get("is_jammed") and (tick - info.get("tick", 0)) <= 3:
                recent_jam = True
                break
        if recent_jam:
            color = "#FF4444"      # red — jam detected
            label = "\u26A0"       # ⚠
        elif is_scanning:
            color = "#FFA500"      # orange — scanning junction
            label = "\u25CE"       # ◎ (radar/bullseye)
        else:
            color = "#00BFFF"      # blue — flying
            label = "\u2708"       # ✈
        return {
            "Shape": "rect", "Filled": True, "Layer": 4,
            "w": 0.6, "h": 0.6, "Color": color,
            "text": label, "text_color": "white",
        }

    # ── Road Event (obstacle) ────────────────────────────────────
    if isinstance(agent, RoadEvent):
        return {
            "Shape": "rect", "Filled": True, "Layer": 3,
            "w": 0.9, "h": 0.9, "Color": "#FF0000",
            "text": "\u2716", "text_color": "white",
        }

    # ── Ground Station ───────────────────────────────────
    if isinstance(agent, GroundStation):
        return {
            "Shape": "rect", "Filled": True, "Layer": 4,
            "w": 0.9, "h": 0.9, "Color": "#228B22",
            "text": "GS", "text_color": "white",
        }

    # ── JunctionController has no visual representation ──────────
    if isinstance(agent, JunctionController):
        return {}

    return {}


# ── Colour helpers ───────────────────────────────────────────────

def _light_color(light):
    """
    Derive the traffic-light colour from the controller's phase and
    the approach type (H / V).

    GREEN  — this approach has a green light right now.
    YELLOW — transition period, no new entries allowed.
    RED    — the opposite approach has the green light.
    """
    phase = light.controller.phase
    is_green = (
        (light.approach == "H" and phase == "H_GREEN") or
        (light.approach == "V" and phase == "V_GREEN")
    )
    is_yellow = (
        (light.approach == "H" and phase == "H_YELLOW") or
        (light.approach == "V" and phase == "V_YELLOW")
    )
    if is_green:
        return "green"
    if is_yellow:
        return "yellow"
    return "red"


def _car_color(car):
    """
    Colour a car by its exit group.

    Adjacent lanes on the same corridor share one colour, so a 2-lane
    northbound exit reads as a single destination.  The matching
    colour is also painted on the exit cells themselves on the grid
    border.
    """
    gidx = getattr(car, "exit_group", None)
    if gidx is None:
        return "#888888"

    group_colors = getattr(car.model, "_group_color", {})
    return group_colors.get(gidx, "#888888")


# ═════════════════════════════════════════════════════════════════════
#  Tunable parameters (displayed in the browser sidebar)
# ═════════════════════════════════════════════════════════════════════
#
#  These are passed to ModularServer as ``model_params``.  When the
#  user clicks **Reset**, the server creates a brand-new TrafficModel
#  using the current slider values.

MODEL_PARAMS = {
    # ── Grid size ────────────────────────────────────────────────
    "width": Slider(
        name="Grid Width",
        value=20,
        min_value=20,
        max_value=60,
        step=5,
        description="Width of the simulation grid (cells).",
    ),
    "height": Slider(
        name="Grid Height",
        value=20,
        min_value=20,
        max_value=60,
        step=5,
        description="Height of the simulation grid (cells).",
    ),

    # ── Road network layout ──────────────────────────────────────
    "num_horizontal_roads": Slider(
        name="Horizontal Roads",
        value=2,
        min_value=1,
        max_value=5,
        step=1,
        description=(
            "Number of horizontal (East/West) corridors.  "
            "Junctions = horizontal × vertical roads."
        ),
    ),
    "num_vertical_roads": Slider(
        name="Vertical Roads",
        value=2,
        min_value=1,
        max_value=5,
        step=1,
        description=(
            "Number of vertical (North/South) corridors.  "
            "Junctions = horizontal × vertical roads."
        ),
    ),

    # ── Traffic density ──────────────────────────────────────────
    "spawn_prob": Slider(
        name="Spawn Probability",
        value=0.08,
        min_value=0.0,
        max_value=1.0,
        step=0.02,
        description=(
            "Probability of spawning a car at each entry cell per "
            "attempt.  Higher = more traffic.  Try 0.20+ to see jams."
        ),
    ),

    # ── Road layout ──────────────────────────────────────────────
    "lanes_choice_prob": Slider(
        name="2-Lane Probability",
        value=0.6,
        min_value=0.0,
        max_value=1.0,
        step=0.1,
        description=(
            "Probability that a corridor has 2 lanes per direction "
            "(otherwise 1 lane).  More lanes = higher junction capacity."
        ),
    ),

    # ── Traffic-light timing ─────────────────────────────────────
    "green_duration": Slider(
        name="Green Duration (ticks)",
        value=12,
        min_value=4,
        max_value=40,
        step=2,
        description="How long a GREEN phase lasts before switching to YELLOW.",
    ),
    "yellow_duration": Slider(
        name="Yellow Duration (ticks)",
        value=4,
        min_value=1,
        max_value=15,
        step=1,
        description="Minimum YELLOW phase length before the light can turn.",
    ),
    "yellow_max_wait": Slider(
        name="Yellow Max Wait (ticks)",
        value=25,
        min_value=5,
        max_value=60,
        step=5,
        description=(
            "Maximum YELLOW phase length — forces a switch even if "
            "cars are still inside the junction."
        ),
    ),

    # ── Drone ────────────────────────────────────────────────────
    # ── Drones ─────────────────────────────────────────────────────
    "num_drones": Slider(
        name="Number of Drones",
        value=2,
        min_value=1,
        max_value=6,
        step=1,
        description=(
            "How many monitoring drones to deploy.  More drones = "
            "better junction coverage but more coordination overhead."
        ),
    ),
    "drone_speed": Slider(
        name="Drone Speed (cells/tick)",
        value=2,
        min_value=1,
        max_value=4,
        step=1,
        description=(
            "How many grid cells a drone moves per tick. "
            "At 10 m/cell and 1 s/tick: 1=10 m/s, 2=20 m/s, 4=40 m/s."
        ),
    ),
    "drone_comm_range": Slider(
        name="Drone Comm Range (cells)",
        value=25,
        min_value=5,
        max_value=50,
        step=1,
        description=(
            "Communication radius of each drone (Euclidean distance "
            "in cells).  Drones relay reports to the ground station "
            "and share information with peers within this range.  "
            "Visible as a blue tint on the grid."
        ),
    ),

    # ── Microscopic NaSch parameters ─────────────────────────────
    "v_max": Slider(
        name="Max Velocity (cells/tick)",
        value=4,
        min_value=1,
        max_value=8,
        step=1,
        description=(
            "Maximum vehicle speed (cells per tick).  "
            "Higher = faster free-flow traffic on open roads."
        ),
    ),
    "random_slow_prob": Slider(
        name="Random Slowing Prob",
        value=0.3,
        min_value=0.0,
        max_value=1.0,
        step=0.05,
        description=(
            "Nagel-Schreckenberg random deceleration probability.  "
            "Models driver hesitation.  Higher = more stop-and-go."
        ),
    ),

    # ── Jam / prediction thresholds ─────────────────────────────
    "jam_density_threshold": Slider(
        name="Jam Density Threshold",
        value=0.30,
        min_value=0.05,
        max_value=1.0,
        step=0.05,
        description=(
            "Minimum fraction of approach cells that must be occupied "
            "for a jam to be declared.  Rejects empty-road false positives. "
            "Default 0.30 = 30% of cells occupied."
        ),
    ),
    "jam_speed_threshold": Slider(
        name="Jam Speed Threshold",
        value=0.40,
        min_value=0.05,
        max_value=1.0,
        step=0.05,
        description=(
            "Jam declared when mean speed < threshold x v_max. "
            "Default 0.40 with v_max=4 = speeds below 1.6 cells/tick. "
            "Rejects free-flow traffic that happens to have low density."
        ),
    ),
    "jam_queue_min": Slider(
        name="Min Queue Length",
        value=1,
        min_value=1,
        max_value=10,
        step=1,
        description=(
            "Minimum contiguous queue length (cells) at the stop line "
            "required to declare a jam.  Filters scattered slow vehicles."
        ),
    ),
    "hover_ticks_min": Slider(
        name="Hover Min (ticks)",
        value=10,
        min_value=1,
        max_value=60,
        step=1,
        description=(
            "Shortest hover duration per junction visit (ticks = seconds "
            "at 1 s/tick). Detection occurs only while hovering."
        ),
    ),
    "hover_ticks_max": Slider(
        name="Hover Max (ticks)",
        value=60,
        min_value=1,
        max_value=120,
        step=1,
        description=(
            "Longest hover duration per junction visit. Actual hover time "
            "is drawn uniformly from [hover_min, hover_max] on each visit."
        ),
    ),
    "jam_sustain_ticks": Slider(
        name="Jam Sustain Ticks",
        value=3,
        min_value=1,
        max_value=20,
        step=1,
        description="How many consecutive ticks are required before a jam is confirmed.",
    ),
    "prediction_horizon": Slider(
        name="Prediction Horizon",
        value=12,
        min_value=1,
        max_value=40,
        step=1,
        description="Ticks ahead used to score whether a prediction was correct.",
    ),

    # ── Random obstacle process ─────────────────────────────────
    "event_spawn_prob": Slider(
        name="Obstacle Spawn Prob",
        value=0.04,
        min_value=0.0,
        max_value=1.0,
        step=0.01,
        description="Per-tick probability of spawning a random obstacle after warmup.",
    ),
    "event_duration_min": Slider(
        name="Obstacle Duration Min",
        value=8,
        min_value=1,
        max_value=100,
        step=1,
        description="Minimum obstacle lifetime (uniform distribution lower bound).",
    ),
    "event_duration_max": Slider(
        name="Obstacle Duration Max",
        value=24,
        min_value=1,
        max_value=150,
        step=1,
        description="Maximum obstacle lifetime (uniform distribution upper bound).",
    ),
    "event_max_active": Slider(
        name="Max Active Obstacles",
        value=2,
        min_value=1,
        max_value=10,
        step=1,
        description="Upper bound on simultaneous obstacle events.",
    ),
    "event_warmup_ticks": Slider(
        name="Obstacle Warmup Ticks",
        value=10,
        min_value=0,
        max_value=100,
        step=1,
        description="No obstacle can spawn before this tick.",
    ),

    # ── Road priority ────────────────────────────────────────────
    "road_priority": Slider(
        name="Road Priority (0=H,1=V,2=None)",
        value=2,
        min_value=0,
        max_value=2,
        step=1,
        description=(
            "0 = horizontal roads have higher priority, "
            "1 = vertical roads have higher priority, "
            "2 = no priority (equal treatment).  "
            "Used when both directions are congested simultaneously."
        ),
    ),
}


# ═════════════════════════════════════════════════════════════════════
#  Server launch
# ═════════════════════════════════════════════════════════════════════

def run_server():
    """
    Build a MESA ModularServer and open it in the browser.

    The server is given the **TrafficModel class** (not an instance)
    plus ``MODEL_PARAMS`` so that pressing Reset creates a fresh model
    with the current slider values.
    """
    # Use the largest possible canvas; individual runs may be smaller
    # but CanvasGrid scales automatically.
    grid = CanvasGrid(agent_portrayal, 30, 30, 800, 800)

    # Metrics text element that updates every tick
    from mesa.visualization.modules import TextElement

    class MonitoringMetrics(TextElement):
        """Live metrics panel showing monitoring performance."""

        def render(self, model):
            gs = getattr(model, "ground_station", None)
            drones = getattr(model, "drones", [])
            tick = getattr(model, "_tick_count", 0)

            lines = []
            lines.append(f"<b>Tick:</b> {tick}")
            lines.append(f"<b>Drones:</b> {len(drones)}")

            if gs:
                avg_freq = gs.get_avg_update_frequency()
                lines.append(
                    f"<b>GS Updates (total):</b> {gs.total_updates}"
                )
                lines.append(
                    f"<b>GS Avg Updates/tick:</b> {avg_freq:.2f}"
                )
                active_jams = len(gs.active_jams)
                lines.append(
                    f"<b>Active Jams (GS view):</b> {active_jams}"
                )

            # Objective jam stats (model-level ground truth)
            jam_state = getattr(model, "junction_jam_state", {})
            actual_jams = sum(
                1 for info in jam_state.values() if info["is_jammed"]
            )
            lines.append(
                f"<b>Actual Jams (ground truth):</b> {actual_jams}"
            )
            eff = model.get_monitoring_efficiency()
            if eff is not None:
                lines.append(
                    f"<b>Jam Monitoring Efficiency:</b> "
                    f"{eff:.1%}"
                )
            else:
                lines.append(
                    "<b>Jam Monitoring Efficiency:</b> N/A "
                    "(no jams yet)"
                )

            logger = getattr(model, "logger", None)
            if logger is not None:
                lines.append(f"<b>Run ID:</b> {logger.run_id}")
            completed = getattr(model, "_completed_jam_events", [])
            if completed:
                detected = sum(1 for e in completed if e.get("detected"))
                missed = len(completed) - detected
                lines.append(
                    f"<b>Jam Events:</b> {len(completed)} "
                    f"(detected {detected}, missed {missed})"
                )
            pred_total = getattr(model, "prediction_total", 0)
            pred_correct = getattr(model, "prediction_correct", 0)
            if pred_total > 0:
                lines.append(
                    f"<b>Prediction Accuracy:</b> "
                    f"{(pred_correct / pred_total):.1%} "
                    f"({pred_correct}/{pred_total})"
                )

            if gs:
                pass  # GS section continues below

                # Per-junction coverage staleness
                coverage = gs.get_junction_coverage()
                if coverage:
                    avg_stale = sum(coverage.values()) / len(coverage)
                    max_stale = max(coverage.values())
                    lines.append(
                        f"<b>Avg Junction Staleness:</b> "
                        f"{avg_stale:.1f} ticks"
                    )
                    lines.append(
                        f"<b>Max Junction Staleness:</b> "
                        f"{max_stale} ticks"
                    )

            # Per-drone summary
            for d in drones:
                if d._hover_remaining > 0:
                    status = (
                        f"\u25CE scanning {d.current_junction_id} "
                        f"({d._hover_remaining} ticks left)"
                    )
                else:
                    target = d._target_jid or "\u2014"
                    status = f"\u2708 \u2192 {target}"
                lines.append(
                    f"<b>Drone {d.drone_index}:</b> {status} "
                    f"| visited={d.junctions_visited} "
                    f"| jams={d.jams_detected}"
                )

            # ── Microscopic / Macroscopic model info ─────────────
            lines.append("")
            seed = getattr(model, '_seed', '?')
            lines.append(f"<b>Seed:</b> {seed}")
            lines.append(f"<b>v_max:</b> {getattr(model, 'v_max', '?')} "
                         f"| <b>Random slow p:</b> "
                         f"{getattr(model, 'random_slow_prob', '?')}")
            lines.append(f"<b>Road priority:</b> "
                         f"{getattr(model, 'road_priority', '?')}")

            # Road events
            events = getattr(model, "road_events", [])
            if events:
                for ev in events:
                    lines.append(
                        f"<b>\u26A0 Road Event:</b> {ev.pos} "
                        f"(dir {ev.direction})"
                    )

            # Average vehicle velocity across all cars
            from car import CarAgent as _CA
            all_cars = [a for a in model.schedule.agents
                        if isinstance(a, _CA)]
            if all_cars:
                avg_v = sum(getattr(c, "velocity", 0) for c in all_cars) / len(all_cars)
                lines.append(
                    f"<b>Cars:</b> {len(all_cars)} "
                    f"| <b>Avg velocity:</b> {avg_v:.2f} cells/tick"
                )
            else:
                lines.append("<b>Cars:</b> 0")

            # Macroscopic summary per junction (from GS latest data)
            if gs:
                for jid in sorted(gs.junction_status.keys()):
                    st = gs.junction_status[jid]
                    seg = st.get("segment_metrics", {})
                    if seg:
                        densities = [m.get("density", 0) for m in seg.values()]
                        flows = [m.get("flow", 0) for m in seg.values()]
                        avg_den = sum(densities) / max(len(densities), 1)
                        avg_flow = sum(flows) / max(len(flows), 1)
                        lines.append(
                            f"<b>{jid}:</b> "
                            f"k\u0305={avg_den:.2f} "
                            f"q\u0305={avg_flow:.2f}"
                        )

            # ── Per-junction detection / prediction (drone view) ─
            if drones:
                lines.append("")
                lines.append(
                    "<b>\u2500\u2500 Drone Detection / "
                    "Prediction \u2500\u2500</b>"
                )
                for jid in sorted(model.junction_controllers.keys()):
                    det, pred, recent = _drone_junction_status(
                        jid, model
                    )
                    if det or pred:
                        parts = []
                        if det:
                            arrows = "".join(
                                _ARROWS[d]
                                for d in ("N", "S", "E", "W")
                                if d in det
                            )
                            parts.append(
                                f"\U0001f534 Jam: {arrows}"
                            )
                        if pred:
                            arrows = "".join(
                                _ARROWS[d]
                                for d in ("N", "S", "E", "W")
                                if d in pred
                            )
                            parts.append(
                                f"\U0001f7e0 Predicted: {arrows}"
                            )
                        lines.append(
                            f"<b>{jid}:</b> "
                            f"{' | '.join(parts)}"
                        )

            return "<br>".join(lines)

    metrics = MonitoringMetrics()

    server = ModularServer(
        TrafficModel,
        [grid, metrics],
        "Drone Monitoring Traffic Simulation",
        MODEL_PARAMS,
    )
    server.port = 8521
    server.launch()
