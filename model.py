"""
model.py — TrafficModel: the MESA simulation model.

Owns the grid, the scheduler, and the simulation loop.  Grid
construction is delegated to :mod:`grid_builder`; car behaviour lives
in :mod:`car`.

Public API used by the visualization layer:
    ``TrafficModel(width, height, …)``  — create a ready-to-run model
    ``model.step()``                     — advance one tick
"""

import random
import time
import atexit
import os
import weakref
from collections import deque

from mesa import Model
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation

from cells import RoadCell, JunctionCell, RoadEvent
from car import CarAgent
from grid_builder import build_road_network
from drone import DroneAgent, SCAN_DEPTH, _OPPOSITE, _APPROACH_DIRS, _approach_has_green, _approach_is_pure_green
from ground_station import GroundStation
from results_logger import SimulationLogger


def _finalize_model_atexit(model_ref):
    """Finalize logging for the current model without retaining it strongly."""
    model = model_ref()
    if model is not None:
        model._finalize_logging()


class TrafficModel(Model):
    """
    Top-level simulation model.

    Parameters
    ----------
    width, height : int
        Grid dimensions (cells).
    spawn_prob : float
        Per-entry-cell probability of spawning a car each attempt.
    lanes_choice_prob : float
        Probability that a corridor has 2 lanes per direction
        (otherwise 1).
    green_duration : int
        Ticks a GREEN phase lasts.
    yellow_duration : int
        Minimum ticks a YELLOW phase lasts.
    yellow_max_wait : int
        Maximum ticks before a YELLOW phase is forcibly ended.
    num_horizontal_roads : int
        Number of horizontal corridors (East/West).  Each corridor
        crosses every vertical corridor, creating junctions.
    num_vertical_roads : int
        Number of vertical corridors (North/South).
    seed : int | None
        Optional random seed for reproducibility.
    """

    def __init__(self, width=40, height=40, spawn_prob=0.08,
                 lanes_choice_prob=0.6, green_duration=12,
                 yellow_duration=4, yellow_max_wait=25,
                 num_horizontal_roads=2, num_vertical_roads=2,
                 drone_speed=2, drone_comm_range=25,
                 num_drones=2,
                 v_max=4, random_slow_prob=0.3,
                 road_priority="V",
                 # ── Multi-criteria jam definition ─────────────────
                 # A direction is jammed when ALL THREE hold (pure green):
                 #   density  >= jam_density_threshold   (not an empty road)
                 #   mean_speed <  jam_speed_threshold * v_max  (cars slow)
                 #   queue_length >= jam_queue_min        (real queue at stop line)
                 jam_density_threshold=0.30,
                 jam_speed_threshold=0.40,
                 jam_queue_min=1,
                 jam_sustain_ticks=3,
                 # ── Physical scale ───────────────────────────────
                 cell_size_m=10.0,       # metres per grid cell
                 tick_duration_s=1.0,    # seconds per simulation tick
                 # ── Drone hover duration per junction visit ──────
                 hover_ticks_min=10,     # shortest hover (10 s at 1 s/tick)
                 hover_ticks_max=60,     # longest  hover (60 s at 1 s/tick)
                 drone_min_detect_votes=2,  # consecutive green ticks required to confirm a jam
                 prediction_horizon=12,
                 car_history_maxlen=0,
                 event_spawn_prob=0.04,
                 event_duration_min=8,
                 event_duration_max=24,
                 event_max_active=2,
                 event_warmup_ticks=10,
                 adaptive_signal_control=True,
                 results_dir=None,
                 seed=None):
        # ── Seed for reproducibility ─────────────────────────────
        if seed is None:
            seed = int(time.time() * 1000) % (2**31)
        self._seed = seed
        random.seed(seed)
        print(f"[TrafficModel] seed = {seed}")

        super().__init__(seed=seed)
        self.width = width
        self.height = height
        self.drone_speed = drone_speed
        self.drone_comm_range = drone_comm_range
        self.lanes_choice_prob = float(lanes_choice_prob)
        self.green_duration = int(green_duration)
        self.yellow_duration = int(yellow_duration)
        self.yellow_max_wait = int(yellow_max_wait)

        # Multi-criteria jam thresholds
        self.jam_density_threshold = max(0.01, min(1.0, float(jam_density_threshold)))
        self.jam_speed_threshold   = max(0.01, min(1.0, float(jam_speed_threshold)))
        self.jam_queue_min         = max(1, int(jam_queue_min))
        self.jam_sustain_ticks     = max(1, int(jam_sustain_ticks))
        self.prediction_horizon = max(1, int(prediction_horizon))
        # Keep per-car history bounded to avoid linear memory growth in long runs.
        self.car_history_maxlen = max(0, int(car_history_maxlen))
        # Physical scale
        self.cell_size_m = float(cell_size_m)
        self.tick_duration_s = float(tick_duration_s)
        # Drone hover bounds
        self.hover_ticks_min = max(1, int(hover_ticks_min))
        self.hover_ticks_max = max(self.hover_ticks_min, int(hover_ticks_max))
        self.drone_min_detect_votes = max(1, int(drone_min_detect_votes))

        self.event_spawn_prob = float(event_spawn_prob)
        self.event_duration_min = max(1, int(event_duration_min))
        self.event_duration_max = max(self.event_duration_min,
                          int(event_duration_max))
        self.event_max_active = max(1, int(event_max_active))
        self.event_warmup_ticks = max(0, int(event_warmup_ticks))
        self.adaptive_signal_control = bool(adaptive_signal_control)
        self.results_dir = results_dir or os.path.join("results", "AllResults")

        # ── Microscopic NaSch parameters ─────────────────────────
        self.v_max = max(1, int(v_max))           # max velocity (cells/step)
        self.random_slow_prob = float(random_slow_prob)  # prob of random deceleration

        # ── Road priority for conflict resolution ────────────────
        # 'V' = vertical roads have higher priority, 'H' = horizontal,
        # 'NONE' = no priority (equal treatment).
        # Accept int (0=H, 1=V, 2=NONE) from slider or string directly.
        if isinstance(road_priority, (int, float)):
            rp = int(road_priority)
            self.road_priority = {0: "H", 1: "V"}.get(rp, "NONE")
        else:
            self.road_priority = road_priority
        self.grid = MultiGrid(width, height, torus=False)
        self.schedule = SimultaneousActivation(self)
        self.spawn_prob = spawn_prob
        self._id = 0
        self._tick_count = 0
        self._to_remove = set()   # cars flagged for removal this tick

        # ── Road events (obstacles) ──────────────────────────────
        self.road_events = []          # active RoadEvent agents
        self._road_event_meta = {}     # event_id -> metadata
        self._next_obstacle_id = 0

        # ── Build the road network (roads, junctions, controllers) ──
        network = build_road_network(
            self, lanes_choice_prob,
            green_duration, yellow_duration, yellow_max_wait,
            num_horizontal_roads=num_horizontal_roads,
            num_vertical_roads=num_vertical_roads,
        )
        self.roads = network["roads"]
        self.junctions = network["junctions"]
        self.junction_controllers = network["junction_controllers"]
        self.entry_cells = network["entry_cells"]
        self.exit_cells = network["exit_cells"]    # {'N': [...], ...}
        self.exit_groups = network["exit_groups"]    # [(edge, [pos, ...]), ...]
        self.h_centers = network["h_centers"]        # sorted list of y-centres
        self.v_centers = network["v_centers"]        # sorted list of x-centres

        # Map each entry direction to candidate exit edges.
        # Cars can now U-turn at a junction, so ALL four edges are
        # reachable from any entry direction.
        self._entry_dir_to_exit_edges = {
            "N": [e for e in ("N", "E", "W", "S") if self.exit_cells.get(e)],
            "S": [e for e in ("S", "E", "W", "N") if self.exit_cells.get(e)],
            "E": [e for e in ("E", "N", "S", "W") if self.exit_cells.get(e)],
            "W": [e for e in ("W", "N", "S", "E") if self.exit_cells.get(e)],
        }

        # Index exit groups by edge for quick lookup during spawning
        self._exit_groups_by_edge = {}
        for idx, (edge, cells) in enumerate(self.exit_groups):
            self._exit_groups_by_edge.setdefault(edge, []).append(idx)

        # Pre-build colour map: one distinct hue per exit GROUP.
        # Every lane in the same group gets the same colour.
        n = max(len(self.exit_groups), 1)
        self._exit_color_map = {}      # pos → hsl string
        self._group_color = {}          # group_index → hsl string
        for i, (_edge, cells) in enumerate(self.exit_groups):
            hue = int(360 * i / n)
            color = f"hsl({hue}, 85%, 45%)"
            self._group_color[i] = color
            for pos in cells:
                self._exit_color_map[pos] = color

        # ── Create ground station at lower-left corner ──────────
        gs_pos = (1, 1)
        self.ground_station = GroundStation(self.next_id(), self, pos=gs_pos)
        self.grid.place_agent(self.ground_station, gs_pos)
        self.schedule.add(self.ground_station)

        # ── Create monitoring drones ───────────────────────────
        self.num_drones = max(1, int(num_drones))
        self.drones = []
        for i in range(self.num_drones):
            d = DroneAgent(self.next_id(), self, drone_index=i)
            self.schedule.add(d)
            self.drones.append(d)

        # ── CSV logging ──────────────────────────────────────────
        run_meta = {
            "seed": self._seed,
            "ticks": "",
            "width": self.width,
            "height": self.height,
            "num_junctions": len(self.junction_controllers),
            "num_drones": self.num_drones,
            "spawn_prob": round(self.spawn_prob, 4),
            "lanes_choice_prob": round(self.lanes_choice_prob, 4),
            "v_max": self.v_max,
            "random_slow_prob": round(self.random_slow_prob, 4),
            "green_duration": self.green_duration,
            "yellow_duration": self.yellow_duration,
            "yellow_max_wait": self.yellow_max_wait,
            "drone_speed": self.drone_speed,
            "drone_comm_range": self.drone_comm_range,
            "jam_density_threshold": self.jam_density_threshold,
            "jam_speed_threshold": self.jam_speed_threshold,
            "jam_queue_min": self.jam_queue_min,
            "jam_sustain_ticks": self.jam_sustain_ticks,
            "hover_ticks_min": self.hover_ticks_min,
            "hover_ticks_max": self.hover_ticks_max,
            "cell_size_m": self.cell_size_m,
            "tick_duration_s": self.tick_duration_s,
            "prediction_horizon": self.prediction_horizon,
            "event_spawn_prob": self.event_spawn_prob,
            "event_duration_min": self.event_duration_min,
            "event_duration_max": self.event_duration_max,
            "event_max_active": self.event_max_active,
            "event_warmup_ticks": self.event_warmup_ticks,
        }
        self.logger = SimulationLogger(run_meta, results_dir=self.results_dir)
        self._logging_finalized = False
        # Use a weakref callback so old models can be garbage-collected after UI resets.
        self._atexit_finalize_cb = (lambda model_ref=weakref.ref(self):
                                    _finalize_model_atexit(model_ref))
        atexit.register(self._atexit_finalize_cb)

        # ── Objective jam tracking (ground truth) ────────────────
        # Updated every tick by _scan_junctions_for_jams().
        # Key: jid → {"is_jammed": bool, "queues": {dir: int}}
        self.junction_jam_state = {}
        self.total_jam_ticks = 0          # sum over all junction-ticks in jam
        self.monitored_jam_ticks = 0      # subset where a drone was present
        self._jam_truth_active = {}       # (jid, dir) -> bool
        self._jam_truth_counter = {}      # (jid, dir) -> sustained counter
        self._active_jam_events = {}      # (jid, dir) -> event dict
        self._completed_jam_events = []
        self._next_jam_event_id = 0

        # ── Prediction evaluation state ──────────────────────────
        self._prediction_records = []
        self._next_prediction_id = 0
        # Note: per-direction cooldown is now managed PER DRONE inside DroneAgent.
        # The model no longer blocks multi-drone predictions on the same direction.
        self.prediction_total = 0
        self.prediction_correct = 0
        self.prediction_missed = 0

        # ── Prediction memory:  (jid, dir) -> list[{tick, drone_id, ...}]
        # Used to check if a jam was predicted *before* it started.
        self._prediction_memory = {}

        # ── Spawn some initial cars ──────────────────────────────
        for _ in range(30):
            self.try_spawn_edge_car()

    def _finalize_logging(self):
        if self._logging_finalized:
            return

        # Close any still-active jam events at the current tick
        for key in list(self._active_jam_events.keys()):
            self._close_jam_event(key, self._tick_count)

        def _median(lst):
            if not lst:
                return None
            s = sorted(lst)
            m = len(s) // 2
            return s[m] if len(s) % 2 else (s[m - 1] + s[m]) / 2

        completed = self._completed_jam_events
        total    = len(completed)
        detected = sum(1 for e in completed if e.get("detected"))
        missed   = total - detected

        delays = [e["detection_delay"] for e in completed
                  if e.get("detected") and e.get("detection_delay") is not None]

        predicted_jams = [e for e in completed if e.get("predicted_before_start")]
        lead_times = [e["lead_time_ticks"] for e in predicted_jams
                      if e.get("lead_time_ticks") is not None]

        visit_ticks = [e.get("drone_visit_ticks", 0) for e in completed]
        durations   = [e.get("duration", 0) for e in completed]
        red_phases  = [e.get("red_phases_spanned", 0) for e in completed]

        mean_delay  = (sum(delays) / len(delays)) if delays else None

        # ── New comparative metrics ───────────────────────────────────
        # Post-detection residual duration (detected jams only)
        post_det_durs = [
            e["post_detect_duration"] for e in completed
            if e.get("detected") and e.get("post_detect_duration") is not None
        ]
        # Duration split: detected vs missed
        det_durs  = [e.get("duration", 0) for e in completed if     e.get("detected")]
        miss_durs = [e.get("duration", 0) for e in completed if not e.get("detected")]
        # Red-phase split: detected vs missed (proxy for controller effectiveness)
        rp_det  = [e.get("red_phases_spanned", 0) for e in completed if     e.get("detected")]
        rp_miss = [e.get("red_phases_spanned", 0) for e in completed if not e.get("detected")]
        # Duration split: had pre-jam prediction vs did not
        pred_durs     = [e.get("duration", 0) for e in completed if     e.get("predicted_before_start")]
        no_pred_durs  = [e.get("duration", 0) for e in completed if not e.get("predicted_before_start")]
        # Prediction precision / false positives tracked online to avoid retaining
        # all historical prediction records in memory.
        false_pos_preds = self.prediction_missed
        pred_precision  = (self.prediction_correct / self.prediction_total
                           if self.prediction_total else None)

        def _safe_mean(lst):
            return round(sum(lst) / len(lst), 4) if lst else None

        summary = {
            "ticks": self._tick_count,
            "total_jam_events": total,
            "detected_jam_events": detected,
            "missed_jam_events": missed,
            "detection_rate": round(detected / total, 4) if total else None,
            "mean_detection_delay":   round(mean_delay, 4)         if mean_delay is not None else None,
            "median_detection_delay": round(_median(delays), 4)    if delays else None,
            "predicted_jam_events": len(predicted_jams),
            "prediction_rate": round(len(predicted_jams) / total, 4) if total else None,
            "mean_lead_time":   round(sum(lead_times) / len(lead_times), 4) if lead_times else None,
            "median_lead_time": round(_median(lead_times), 4)               if lead_times else None,
            "mean_drone_visit_ticks": round(sum(visit_ticks) / max(len(visit_ticks), 1), 4),
            "mean_jam_duration":      round(sum(durations)   / max(len(durations),   1), 4),
            "mean_red_phases_spanned": round(sum(red_phases) / max(len(red_phases), 1), 4),
            # ── Does detection lead to shorter jams? (controller adaptation benefit)
            "mean_post_detect_duration":  _safe_mean(post_det_durs),
            "mean_duration_detected":     _safe_mean(det_durs),
            "mean_duration_missed":       _safe_mean(miss_durs),
            # ── Does controller break the jam faster once a drone detects it?
            "mean_red_phases_detected":   _safe_mean(rp_det),
            "mean_red_phases_missed":     _safe_mean(rp_miss),
            # ── Does pre-jam prediction shorten jam duration?
            "mean_duration_predicted":    _safe_mean(pred_durs),
            "mean_duration_not_predicted": _safe_mean(no_pred_durs),
            # ── Prediction quality
            "false_positive_predictions": false_pos_preds,
            "prediction_precision":       round(pred_precision, 4) if pred_precision is not None else None,
        }
        self.logger.finalize_run(summary)
        self._logging_finalized = True

    # ── ID generator ─────────────────────────────────────────────

    def next_id(self):
        """Return a monotonically increasing unique agent id."""
        self._id += 1
        return self._id

    # ── Car spawning ─────────────────────────────────────────────

    def try_spawn_edge_car(self):
        """
        Attempt to spawn one car at a random grid-edge entry cell.

        Spawning is probabilistic (controlled by ``spawn_prob``) and
        will silently do nothing if the chosen cell is already occupied.
        """
        if not self.entry_cells:
            return
        pos, direction = random.choice(self.entry_cells)
        if random.random() < self.spawn_prob:
            # Only spawn if the cell is empty
            if not any(isinstance(a, CarAgent)
                       for a in self.grid.get_cell_list_contents([pos])):
                # Pick a specific exit group + lane for this car
                exit_target, exit_edge, exit_group = self._pick_exit(direction)
                route = self.compute_route(pos, direction, exit_target, exit_edge)
                car = CarAgent(self.next_id(), self, direction,
                               exit_target=exit_target, exit_edge=exit_edge,
                               exit_group=exit_group, route=route)
                self.grid.place_agent(car, pos)
                self.schedule.add(car)
                # Plan the first intended turn from the route
                car.plan_turn_for_junction()

    # ── Exit assignment ────────────────────────────────────────────

    def _pick_exit(self, entry_direction):
        """
        Choose a random exit group for a car entering in
        ``entry_direction``.  Returns ``(exit_pos, exit_edge, group_idx)``.

        Prefers exits on a different edge than where the car entered.
        Falls back to any available exit if none are reachable.
        """
        # entry_direction is the direction the car is TRAVELLING, so
        # it entered from the opposite edge.
        candidate_edges = self._entry_dir_to_exit_edges.get(entry_direction, [])
        if not candidate_edges:
            # Absolute fallback: any edge with exits
            candidate_edges = [e for e in ("N", "S", "E", "W")
                               if self.exit_cells.get(e)]
        if not candidate_edges:
            return None, None, None

        edge = random.choice(candidate_edges)
        group_indices = self._exit_groups_by_edge.get(edge, [])
        if not group_indices:
            return None, None, None

        gidx = random.choice(group_indices)
        _edge, cells = self.exit_groups[gidx]
        pos = random.choice(cells)
        return pos, edge, gidx

    # ── Route planning ───────────────────────────────────────────

    # Direction helpers
    _OPPOSITE = {"N": "S", "S": "N", "E": "W", "W": "E"}
    _TURN_FOR = {
        # (current_dir, desired_dir) → turn type
        ("E", "N"): "left",  ("E", "S"): "right",
        ("W", "N"): "right", ("W", "S"): "left",
        ("N", "E"): "right", ("N", "W"): "left",
        ("S", "E"): "left",  ("S", "W"): "right",
        # U-turn: reverse direction at the same junction
        ("E", "W"): "uturn",  ("W", "E"): "uturn",
        ("N", "S"): "uturn",  ("S", "N"): "uturn",
    }

    def compute_route(self, entry_pos, entry_dir, exit_target, exit_edge,
                      avoid_junctions=None):
        """
        Compute a list of waypoints ``[(junction_id, turn), ...]`` that
        guides a car from its entry position/direction to its exit target.

        The road network is a simple grid of corridors:
          • Horizontal corridors at y = each h_center  (travel E or W)
          • Vertical   corridors at x = each v_center  (travel N or S)
          • Junctions where they cross: J_{vx}_{hy}

        A route is at most 2 turns:
          1. Turn onto the vertical corridor whose x matches the exit
             (if not already on it).
          2. Turn onto the horizontal corridor whose y matches the exit
             (if not already on it).
        Or the reverse order, depending on the entry direction.

        Parameters
        ----------
        avoid_junctions : set[str] | None
            Junction IDs to avoid when building the route.  If a
            primary waypoint would pass through an avoided junction,
            an alternative corridor is tried instead.

        Returns a list of ``(junction_id_str, turn_type)`` in order.
        """
        if exit_target is None:
            return []

        if avoid_junctions is None:
            avoid_junctions = set()

        TOLERANCE = 3
        tx, ty = exit_target
        ex, ey = entry_pos

        # Find which vertical corridor (x) the exit is on (or nearest to)
        target_vx = min(self.v_centers, key=lambda vx: abs(vx - tx)) if self.v_centers else None
        # Find which horizontal corridor (y) the exit is on (or nearest to)
        target_hy = min(self.h_centers, key=lambda hy: abs(hy - ty)) if self.h_centers else None

        # Which corridor is the car currently on?
        on_h = min(self.h_centers, key=lambda hy: abs(hy - ey)) if self.h_centers else None
        on_v = min(self.v_centers, key=lambda vx: abs(vx - ex)) if self.v_centers else None

        route = []

        if entry_dir in ("E", "W"):
            # Car is on a horizontal corridor.
            # Step 1: Does it need to change vertical corridor?
            need_v = target_vx is not None and (
                exit_edge in ("N", "S") or
                (exit_edge in ("E", "W") and target_hy is not None and abs(on_h - ty) > TOLERANCE)
            )
            if need_v:
                # Find the junction where current h-corridor meets the
                # target v-corridor.  If jammed, try another vx.
                turn_jid = f"J_{target_vx}_{on_h}"
                use_vx = target_vx
                if turn_jid in avoid_junctions:
                    alt_vx = self._pick_alt_corridor(
                        self.v_centers, target_vx, avoid_junctions,
                        lambda vx: f"J_{vx}_{on_h}")
                    if alt_vx is not None:
                        use_vx = alt_vx
                        turn_jid = f"J_{use_vx}_{on_h}"

                # Which direction do we go on the v-corridor?
                if exit_edge == "N" or (exit_edge in ("E", "W") and target_hy is not None and target_hy > on_h):
                    v_dir = "N"
                else:
                    v_dir = "S"
                turn_type = self._TURN_FOR.get((entry_dir, v_dir), "straight")
                if turn_jid in self.junction_controllers:
                    route.append((turn_jid, turn_type))

                # Step 2: After going N/S, do we need to turn E/W?
                if exit_edge in ("E", "W"):
                    turn2_jid = f"J_{use_vx}_{target_hy}"
                    if turn2_jid in avoid_junctions:
                        alt_hy = self._pick_alt_corridor(
                            self.h_centers, target_hy, avoid_junctions,
                            lambda hy: f"J_{use_vx}_{hy}")
                        if alt_hy is not None:
                            turn2_jid = f"J_{use_vx}_{alt_hy}"
                    turn2_type = self._TURN_FOR.get((v_dir, exit_edge), "straight")
                    if turn2_jid in self.junction_controllers and turn2_jid != turn_jid:
                        route.append((turn2_jid, turn2_type))

            elif exit_edge in ("E", "W") and exit_edge != entry_dir:
                # U-turn at the nearest junction on the current corridor.
                best_vx = min(self.v_centers, key=lambda vx: abs(vx - ex))
                turn_jid = f"J_{best_vx}_{on_h}"
                if turn_jid in avoid_junctions:
                    alt_vx = self._pick_alt_corridor(
                        self.v_centers, best_vx, avoid_junctions,
                        lambda vx: f"J_{vx}_{on_h}")
                    if alt_vx is not None:
                        turn_jid = f"J_{alt_vx}_{on_h}"
                if turn_jid in self.junction_controllers:
                    route.append((turn_jid, "uturn"))

        elif entry_dir in ("N", "S"):
            # Car is on a vertical corridor.
            # Step 1: Does it need to change horizontal corridor?
            need_h = target_hy is not None and (
                exit_edge in ("E", "W") or
                (exit_edge in ("N", "S") and target_vx is not None and abs(on_v - tx) > TOLERANCE)
            )
            if need_h:
                turn_jid = f"J_{on_v}_{target_hy}"
                use_hy = target_hy
                if turn_jid in avoid_junctions:
                    alt_hy = self._pick_alt_corridor(
                        self.h_centers, target_hy, avoid_junctions,
                        lambda hy: f"J_{on_v}_{hy}")
                    if alt_hy is not None:
                        use_hy = alt_hy
                        turn_jid = f"J_{on_v}_{use_hy}"

                if exit_edge == "E" or (exit_edge in ("N", "S") and target_vx is not None and target_vx > on_v):
                    h_dir = "E"
                else:
                    h_dir = "W"
                turn_type = self._TURN_FOR.get((entry_dir, h_dir), "straight")
                if turn_jid in self.junction_controllers:
                    route.append((turn_jid, turn_type))

                # Step 2: After going E/W, turn onto the target v-corridor
                if exit_edge in ("N", "S"):
                    turn2_jid = f"J_{target_vx}_{use_hy}"
                    if turn2_jid in avoid_junctions:
                        alt_vx = self._pick_alt_corridor(
                            self.v_centers, target_vx, avoid_junctions,
                            lambda vx: f"J_{vx}_{use_hy}")
                        if alt_vx is not None:
                            turn2_jid = f"J_{alt_vx}_{use_hy}"
                    turn2_type = self._TURN_FOR.get((h_dir, exit_edge), "straight")
                    if turn2_jid in self.junction_controllers and turn2_jid != turn_jid:
                        route.append((turn2_jid, turn2_type))

            elif exit_edge in ("N", "S") and exit_edge != entry_dir:
                # U-turn at the nearest junction on the current corridor.
                best_hy = min(self.h_centers, key=lambda hy: abs(hy - ey))
                turn_jid = f"J_{on_v}_{best_hy}"
                if turn_jid in avoid_junctions:
                    alt_hy = self._pick_alt_corridor(
                        self.h_centers, best_hy, avoid_junctions,
                        lambda hy: f"J_{on_v}_{hy}")
                    if alt_hy is not None:
                        turn_jid = f"J_{on_v}_{alt_hy}"
                if turn_jid in self.junction_controllers:
                    route.append((turn_jid, "uturn"))

        return route

    @staticmethod
    def _pick_alt_corridor(centers, primary, avoid_junctions, jid_fn):
        """
        Choose the nearest corridor centre in ``centers`` that does NOT
        produce a junction ID in ``avoid_junctions``.

        Parameters
        ----------
        centers : list[int]
            All corridor centres (e.g. ``self.v_centers``).
        primary : int
            The preferred corridor centre that turned out to be jammed.
        avoid_junctions : set[str]
            Junction IDs to avoid.
        jid_fn : callable
            Given a corridor centre, return the junction ID string.

        Returns the alternative centre, or None if nothing is available.
        """
        candidates = sorted(centers, key=lambda c: abs(c - primary))
        for c in candidates:
            if c == primary:
                continue
            if jid_fn(c) not in avoid_junctions:
                return c
        return None

    # ── Rerouting (called by cars when a jam is detected) ────────

    # ── Pathfinding ──────────────────────────────────────────────

    def find_junction_exit_path(self, jid, start_pos, desired_out_dir,
                                check_occupancy=True,
                                require_free_exit=None):
        """
        BFS shortest path through a junction toward an exit road.

        Starting from ``start_pos`` (which must be inside the junction),
        search neighbouring junction cells until we find one whose
        adjacent cell in ``desired_out_dir`` is a matching RoadCell.

        Parameters
        ----------
        jid : str
            Junction identifier.
        start_pos : tuple[int, int]
            Current position inside the junction.
        desired_out_dir : str
            Compass direction the car wants to exit toward.
        check_occupancy : bool
            If True, skip junction cells that are currently occupied by
            another car (finds a free path).  If False, find any
            geometric path regardless of traffic (useful for planning).
        require_free_exit : bool | None
            If True, only accept exit roads where ``road.car is None``.
            If False, accept any exit road (even occupied ones).
            If None (default), inherits from *check_occupancy* for
            backward compatibility.

            The typical use for ``check_occupancy=False,
            require_free_exit=True`` is: find the geometrically
            shortest path through any junction cells (occupied or
            not) but only stop at an exit road that is actually
            free.  This lets a car discover a free lane even when
            the nearest lane is jammed.

        Returns
        -------
        (path, reason) where:
            path   — list of (x, y) positions from start to exit cell,
                     or None if no path was found.
            reason — ``'found'``, ``'exit_occupied'``, or ``'no_exit_road'``.
        """
        if require_free_exit is None:
            require_free_exit = check_occupancy

        controller = self.junction_controllers.get(jid)
        if controller is None:
            return None, "no_controller"

        block = set(controller.block_cells)
        deltas = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}

        queue = deque([start_pos])
        prev = {start_pos: None}          # BFS parent map
        found_any_road = False
        found_occupied = False

        while queue:
            pos = queue.popleft()

            # Check if the cell in desired_out_dir is a matching exit road
            dx, dy = deltas[desired_out_dir]
            out_pos = (pos[0] + dx, pos[1] + dy)

            if 0 <= out_pos[0] < self.width and 0 <= out_pos[1] < self.height:
                road = next(
                    (a for a in self.grid.get_cell_list_contents([out_pos])
                     if isinstance(a, RoadCell)
                     and a.direction == desired_out_dir),
                    None,
                )
                if road and road.direction == desired_out_dir:
                    found_any_road = True
                    if not require_free_exit or road.car is None:
                        # ✓ Found a valid (and free) exit — reconstruct path
                        return self._reconstruct_path(prev, pos), "found"
                    else:
                        found_occupied = True
                elif road:
                    found_any_road = True

            # Explore neighbouring junction cells
            for ndx, ndy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbour = (pos[0] + ndx, pos[1] + ndy)
                if neighbour not in block or neighbour in prev:
                    continue
                if check_occupancy:
                    occupied = any(
                        isinstance(a, CarAgent)
                        for a in self.grid.get_cell_list_contents([neighbour])
                    )
                    if occupied and neighbour != start_pos:
                        continue
                prev[neighbour] = pos
                queue.append(neighbour)

        # No path found
        if found_occupied:
            return None, "exit_occupied"
        return None, "no_exit_road"

    @staticmethod
    def _reconstruct_path(prev, end):
        """Walk the BFS parent map backward to build the path list."""
        path = []
        cur = end
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        path.reverse()
        return path

    def junction_exit_has_free_road(self, jid, desired_out_dir):
        """
        Check whether junction *jid* has at least one free exit road
        in *desired_out_dir*.

        Scans every junction block cell; for each, looks one step in
        *desired_out_dir* for a matching ``RoadCell`` with
        ``car is None``.  Returns ``True`` as soon as one free road
        cell is found.

        This is used by approaching cars to decide whether entering
        the junction makes sense — if every exit road in the desired
        direction is occupied, the car should wait outside rather
        than enter and get stuck.
        """
        controller = self.junction_controllers.get(jid)
        if controller is None:
            return True   # no controller → allow entry (can't check)

        dx, dy = {"N": (0, 1), "S": (0, -1),
                  "E": (1, 0), "W": (-1, 0)}[desired_out_dir]

        for bx, by in controller.block_cells:
            out = (bx + dx, by + dy)
            if not (0 <= out[0] < self.width and 0 <= out[1] < self.height):
                continue
            road = next(
                (a for a in self.grid.get_cell_list_contents([out])
                 if isinstance(a, RoadCell)
                 and a.direction == desired_out_dir),
                None,
            )
            if road and road.car is None:
                return True
        return False

    # ── Road event lifecycle (random, uniform spawn + duration) ──

    def _update_road_events(self):
        """Clear obstacle events whose duration has expired."""
        tick = self._tick_count
        survivors = []
        for event in self.road_events:
            meta = self._road_event_meta.get(event.unique_id, {})
            end_tick = meta.get("end_tick")
            if end_tick is not None and tick > end_tick:
                pos = event.pos
                try:
                    if pos and event in self.grid.get_cell_list_contents([pos]):
                        self.grid.remove_agent(event)
                except Exception:
                    pass
                self.logger.log_obstacle_event({
                    "obstacle_id": meta.get("obstacle_id", event.unique_id),
                    "action": "clear",
                    "tick": tick,
                    "x": pos[0] if pos else None,
                    "y": pos[1] if pos else None,
                    "direction": meta.get("direction", getattr(event, "direction", "")),
                    "duration_ticks": meta.get("duration_ticks", ""),
                    "end_tick": end_tick,
                })
                self._road_event_meta.pop(event.unique_id, None)
            else:
                survivors.append(event)
        self.road_events = survivors

    def _spawn_road_event(self):
        """
        Spawn one obstacle on a uniformly random eligible road cell.

        Each obstacle receives a uniformly random lifetime in
        ``[event_duration_min, event_duration_max]`` ticks.
        """
        eligible = []
        for road in self.roads:
            pos = getattr(road, "pos", None)
            if pos is None:
                continue
            if road.car is not None:
                continue
            if any(isinstance(a, RoadEvent)
                   for a in self.grid.get_cell_list_contents([pos])):
                continue
            if any(isinstance(a, CarAgent)
                   for a in self.grid.get_cell_list_contents([pos])):
                continue
            eligible.append(road)

        if not eligible:
            return False

        road = random.choice(eligible)
        direction = road.direction
        event = RoadEvent(self.next_id(), self, direction)
        self.grid.place_agent(event, road.pos)
        self.road_events.append(event)

        duration = random.randint(self.event_duration_min,
                                  self.event_duration_max)
        end_tick = self._tick_count + duration
        self._next_obstacle_id += 1
        self._road_event_meta[event.unique_id] = {
            "obstacle_id": self._next_obstacle_id,
            "start_tick": self._tick_count,
            "end_tick": end_tick,
            "duration_ticks": duration,
            "direction": direction,
        }
        self.logger.log_obstacle_event({
            "obstacle_id": self._next_obstacle_id,
            "action": "spawn",
            "tick": self._tick_count,
            "x": road.pos[0],
            "y": road.pos[1],
            "direction": direction,
            "duration_ticks": duration,
            "end_tick": end_tick,
        })
        return True

    # ── Drone-observation logging hooks ─────────────────────────

    def record_drone_observation(self, jid, tick, drone_id,
                                 queue_lengths,
                                 segment_metrics,
                                 jam_directions,
                                 predicted_congestion_dirs,
                                 predictions):
        """Update jam detection state and register prediction records from a drone visit."""
        for d in _APPROACH_DIRS:
            seg = segment_metrics.get(d, {})
            pred = predictions.get(d, {})
            is_detected = d in jam_directions
            is_predicted = d in predicted_congestion_dirs

            jam_key = (jid, d)
            if is_detected and jam_key in self._active_jam_events:
                evt = self._active_jam_events[jam_key]
                if not evt["detected"]:
                    evt["detected"] = True
                    evt["first_detect_tick"] = tick
                    evt["detection_delay"] = tick - evt["start_tick"]

            if is_predicted:
                self._register_prediction(
                    tick=tick,
                    drone_id=drone_id,
                    junction_id=jid,
                    direction=d,
                    estimated_inflow=pred.get("estimated_inflow", 0),
                    delta_k=pred.get("delta_k", 0),
                    density=seg.get("density", 0),
                    avg_velocity=seg.get("avg_velocity", 0),
                    flow=seg.get("flow", 0),
                )

    def _register_prediction(self, tick, drone_id, junction_id, direction,
                             estimated_inflow, delta_k,
                             density, avg_velocity, flow):
        key = (junction_id, direction)
        # Cooldown is now per-drone (managed in DroneAgent._pred_cooldown).
        # The only model-side guard is: don't register predictions for a
        # direction that already has an active ground-truth jam.
        if self._jam_truth_active.get(key, False):
            return

        self._next_prediction_id += 1
        rec = {
            "prediction_id": self._next_prediction_id,
            "tick": tick,
            "deadline_tick": tick + self.prediction_horizon,
            "drone_id": drone_id,
            "junction_id": junction_id,
            "direction": direction,
            "estimated_inflow": estimated_inflow,
            "delta_k": delta_k,
            "density": density,
            "avg_velocity": avg_velocity,
            "flow": flow,
            "resolved": False,
            "actual_jam_within_horizon": False,
            "first_actual_jam_tick": None,
            "resolved_tick": None,
            "status": "pending",
        }
        self._prediction_records.append(rec)
        self.prediction_total += 1

        # Store in prediction memory so future jams can measure lead time.
        # Keep only horizon-relevant items so this buffer stays bounded.
        if key not in self._prediction_memory:
            self._prediction_memory[key] = []
        self._prune_prediction_memory_for_key(key, tick)
        self._prediction_memory.setdefault(key, []).append({
            "prediction_id": self._next_prediction_id,
            "tick": tick,
            "deadline_tick": tick + self.prediction_horizon,
            "drone_id": drone_id,
            "delta_k": delta_k,
            "estimated_inflow": estimated_inflow,
        })

    def _prune_prediction_memory_for_key(self, key, tick):
        """Drop expired prediction-memory rows for one (junction, direction)."""
        memory = self._prediction_memory.get(key)
        if not memory:
            return
        kept = [
            m for m in memory
            if m.get("deadline_tick", m.get("tick", tick) + self.prediction_horizon) >= tick
        ]
        if kept:
            self._prediction_memory[key] = kept
        else:
            self._prediction_memory.pop(key, None)

    def _prune_prediction_memory_global(self, tick):
        """Bound all per-direction prediction-memory lists to horizon-relevant rows."""
        for key in list(self._prediction_memory.keys()):
            self._prune_prediction_memory_for_key(key, tick)

    def _resolve_prediction_records(self):
        tick = self._tick_count
        unresolved = []
        for rec in self._prediction_records:
            key = (rec["junction_id"], rec["direction"])
            if self._jam_truth_active.get(key, False):
                rec["resolved"] = True
                rec["actual_jam_within_horizon"] = True
                rec["first_actual_jam_tick"] = tick
                rec["resolved_tick"] = tick
                rec["status"] = "correct"
                self.prediction_correct += 1
                continue

            if tick >= rec["deadline_tick"]:
                rec["resolved"] = True
                rec["actual_jam_within_horizon"] = False
                rec["resolved_tick"] = tick
                rec["status"] = "miss"
                self.prediction_missed += 1
                continue

            unresolved.append(rec)

        self._prediction_records = unresolved

        # Periodic global prune keeps memory bounded while avoiding per-tick full scans.
        if tick % 25 == 0:
            self._prune_prediction_memory_global(tick)

    # ── Main simulation tick ─────────────────────────────────────

    def step(self):
        """
        Advance the simulation by one tick.

        Order of operations:
          1. Update/clear obstacle events and maybe spawn new ones.
          2. Try to spawn new cars at grid edges.
          3. Prepare per-tick reservation set (prevents two cars from
             claiming the same junction cell in the same tick).
          4a. All agents compute ``step()`` (decide ``next_pos``).
          4b. Detect & resolve cyclic junction deadlocks (coordinated
              rotation for cars forming closed \"who-blocks-whom\" loops).
          4c. All agents execute ``advance()`` (apply moves).
          5. Evaluate objective jams and prediction outcomes.
          6. Remove cars that left the grid.
        """
        self._tick_count += 1

        # 0 — Obstacle lifecycle (uniform random spawn + uniform duration)
        self._update_road_events()
        if (self._tick_count >= self.event_warmup_ticks
            and len(self.road_events) < self.event_max_active
            and random.random() < self.event_spawn_prob):
            self._spawn_road_event()

        # 1 — Spawn attempts
        for _ in range(3):
            self.try_spawn_edge_car()

        # 2 — Fresh reservation set for this tick
        self._reserved_targets = set()

        # 3 — All agents act  (split into step / resolve / advance
        #     so we can detect and break cyclic junction deadlocks
        #     between the decision and execution phases)
        agents = list(self.schedule.agents)
        for agent in agents:
            agent.step()

        self._resolve_junction_deadlocks()

        for agent in agents:
            agent.advance()
        self.schedule.steps += 1
        self.schedule.time += 1

        # 4 — Objective jam scan (ground truth, independent of drones)
        self._scan_junctions_for_jams()
        self._resolve_prediction_records()

        # 5 — Clean up removed cars
        for car in list(self._to_remove):
            try:
                if car.pos and car in self.grid.get_cell_list_contents([car.pos]):
                    self.grid.remove_agent(car)
            except Exception:
                pass
            try:
                self.schedule.remove(car)
            except Exception:
                pass
            self._to_remove.discard(car)

    # ── Cyclic junction-deadlock resolution ─────────────────────

    def _resolve_junction_deadlocks(self):
        """
        Detect and resolve cyclic deadlocks inside junctions.

        After all agents have computed ``next_pos`` in the step phase,
        some junction cars may be stuck because each car's desired next
        cell is occupied by another stuck car, forming a closed cycle::

            A wants B's cell → B wants C's cell → C wants A's cell

        Since every cell in a cycle is simultaneously vacated **and**
        filled, a coordinated rotation is safe: all cars advance one
        position along the cycle in the same tick.

        The rotation is executed directly (grid moves + holder updates)
        so that ``advance()`` becomes a no-op for the rotated cars,
        avoiding holder.car desynchronisation.
        """
        for jid, ctrl in self.junction_controllers.items():
            block_cells = set(ctrl.block_cells)

            # ── Gather stuck cars with a recorded desire ─────────
            stuck = []
            pos_to_car = {}
            for cell in block_cells:
                for agent in self.grid.get_cell_list_contents([cell]):
                    if (isinstance(agent, CarAgent)
                            and agent._junction_entry_id == jid
                            and agent.next_pos is not None
                            and agent.next_pos == agent.pos      # stuck
                            and agent._desired_junction_move is not None
                            and agent._desired_junction_move in block_cells):
                        stuck.append(agent)
                        pos_to_car[agent.pos] = agent

            if len(stuck) < 2:
                continue

            # ── Build directed graph: car → car it's blocked by ──
            blocked_by = {}
            for car in stuck:
                blocker = pos_to_car.get(car._desired_junction_move)
                if blocker is not None and blocker is not car:
                    blocked_by[car] = blocker

            # ── Find and resolve cycles ──────────────────────────
            resolved = set()
            for start_car in stuck:
                if start_car in resolved or start_car not in blocked_by:
                    continue

                # Walk the chain
                chain = []
                visited = {}
                current = start_car
                while (current
                       and current not in visited
                       and current in blocked_by):
                    visited[current] = len(chain)
                    chain.append(current)
                    current = blocked_by[current]

                if current and current in visited:
                    cycle_start = visited[current]
                    cycle = chain[cycle_start:]

                    if len(cycle) >= 2:
                        self._execute_cycle_rotation(cycle)
                        resolved.update(cycle)

    def _execute_cycle_rotation(self, cycle):
        """
        Perform a coordinated rotation for *cycle* (list of CarAgents).

        Each car moves to its ``_desired_junction_move`` position.
        The moves are executed directly on the grid so that the
        subsequent ``advance()`` call is effectively a no-op.
        """
        # Phase 1: clear holder.car for every cell being vacated
        for car in cycle:
            holder = self._holder_at(car.pos)
            if holder:
                holder.car = None

        # Phase 2: move each car and set holder.car at new position
        for car in cycle:
            new_pos = car._desired_junction_move
            try:
                self.grid.move_agent(car, new_pos)
            except Exception:
                pass
            holder = self._holder_at(car.pos)
            if holder:
                holder.car = car
            # Make advance() a no-op: next_pos == current pos
            car.next_pos = car.pos
            car._desired_junction_move = None
            car._turn_wait_counter = 0

    def _holder_at(self, pos):
        """Return the RoadCell / JunctionCell at *pos*, or None."""
        holders = [
            a for a in self.grid.get_cell_list_contents([pos])
            if isinstance(a, (RoadCell, JunctionCell))
        ]
        return holders[0] if holders else None

    # ── Objective jam detection ───────────────────────────────────

    def _scan_junctions_for_jams(self):
        """
        Scan every junction and compute the *ground-truth* jam state.

        A direction is treated as jammed when **all three** criteria hold
        simultaneously during a pure-green phase:

        1. ``density >= jam_density_threshold``   — segment is genuinely
           loaded (rejects empty-road false positives).
        2. ``mean_speed < jam_speed_threshold * v_max``  — traffic is slow
           (rejects free-flow traffic at any density).
        3. ``queue_length >= jam_queue_min``  — a real contiguous queue
           has formed at the stop line (rejects scattered slow cars).

        All three conditions must persist for ``jam_sustain_ticks``
        consecutive pure-green ticks before a jam event is opened.
        Once open, the jam closes immediately when any criterion fails.

        Yellow phase: state is frozen (no evaluation, no counter change).
        Red phase: state is preserved, counter reset.

        Also updates ``total_jam_ticks`` and ``monitored_jam_ticks``.
        """
        for jid, ctrl in self.junction_controllers.items():
            queues = {}
            direction_metrics = {}
            jam_dirs = set()
            for d in _APPROACH_DIRS:
                queues[d] = self._count_approach_queue(ctrl, d)
                metrics = self._analyze_approach_segment(ctrl, d)
                direction_metrics[d] = metrics

                # ── Multi-criteria jam trigger, signal-phase aware ─────────
                # Pure green: evaluate trigger.  Once open a jam stays open as
                # long as all criteria hold; closes immediately when any fails.
                # A new jam still requires jam_sustain_ticks green ticks.
                #
                # Yellow phase: state is frozen — flow/speed readings during
                # yellow are transient; counter left unchanged.
                #
                # Red phase: counter is reset, state is PRESERVED so a jam
                # that started during green is not spuriously closed each cycle.
                # We track how many red phases the jam survived.
                density     = metrics.get("density", 0.0)
                avg_vel     = metrics.get("avg_velocity", 0.0)
                car_count   = metrics.get("car_count", 0)
                queue       = queues[d]          # contiguous cars at stop line
                speed_limit = self.jam_speed_threshold * self.v_max
                key         = (jid, d)
                prev_active = self._jam_truth_active.get(key, False)

                if _approach_is_pure_green(ctrl, d):
                    # ── Pure green phase: evaluate multi-criteria trigger ─
                    is_trigger = (
                        car_count >= 1
                        and density >= self.jam_density_threshold
                        and avg_vel  <  speed_limit
                        and queue   >= self.jam_queue_min
                    )
                    if is_trigger:
                        self._jam_truth_counter[key] = self._jam_truth_counter.get(key, 0) + 1
                    else:
                        self._jam_truth_counter[key] = 0

                    counter = self._jam_truth_counter[key]

                    if prev_active:
                        # Existing jam: keep alive while trigger fires;
                        # close immediately the moment flow recovers.
                        is_active = is_trigger
                    else:
                        # No current jam: require sustain_ticks to open.
                        is_active = counter >= self.jam_sustain_ticks

                    self._jam_truth_active[key] = is_active

                    # Clear the non-green flag now that we are on pure green
                    if key in self._active_jam_events:
                        self._active_jam_events[key]["_was_red_last_tick"] = False

                    if is_active and not prev_active:
                        start_tick = self._tick_count - self.jam_sustain_ticks + 1
                        self._open_jam_event(jid, d, start_tick)
                    elif not is_active and prev_active:
                        self._close_jam_event(key, self._tick_count)

                elif _approach_has_green(ctrl, d):
                    # ── Yellow phase: preserve state, do not evaluate trigger
                    # Flow readings during yellow are transient (cars clearing
                    # the stop line); treating yellow like red would wrongly
                    # reset the sustain counter or close an active jam.
                    # The sustain counter is left unchanged so the green-phase
                    # streak is not broken by a yellow interlude.
                    is_active = prev_active   # carry through unchanged

                else:
                    # ── Red phase: preserve jam state, reset counter ──────
                    self._jam_truth_counter[key] = 0
                    is_active = prev_active   # carry through unchanged

                    # Count each new red-phase entry while a jam is active
                    if is_active and key in self._active_jam_events:
                        evt = self._active_jam_events[key]
                        if not evt.get("_was_red_last_tick", False):
                            evt["red_phases_spanned"] += 1
                        evt["_was_red_last_tick"] = True

                if is_active:
                    jam_dirs.add(d)
                    if key in self._active_jam_events:
                        self._update_jam_event_stats(
                            key,
                            queue=queues[d],
                            density=metrics.get("density", 0),
                            avg_velocity=metrics.get("avg_velocity", 0),
                            delta_k=metrics.get("delta_k", 0),
                            flow=metrics.get("flow", 0),
                        )
                        # Drone coverage: count active-scan ticks for the
                        # full duration of the jam (both green and red phases).
                        evt  = self._active_jam_events[key]
                        tick = self._tick_count
                        drone_actively_scanning = any(
                            dn.current_junction_id == jid
                            and dn._hover_remaining > 0
                            and tick >= dn._detection_ready_tick
                            for dn in self.drones
                        )
                        if drone_actively_scanning:
                            evt["drone_visit_ticks"] += 1
                            if not evt["_drone_was_present_last_tick"]:
                                evt["num_drone_visits"] += 1
                            evt["_drone_was_present_last_tick"] = True
                        else:
                            evt["_drone_was_present_last_tick"] = False

            is_jammed = len(jam_dirs) > 0

            self.junction_jam_state[jid] = {
                "is_jammed": is_jammed,
                "queues": queues,
                "jam_directions": sorted(jam_dirs),
                "segment_metrics": direction_metrics,
            }

            if is_jammed:
                self.total_jam_ticks += 1
                # Count as monitored only when a drone is actively scanning
                # (past its camera-processing window, not just hovering)
                tick = self._tick_count
                if any(
                    dn.current_junction_id == jid
                    and dn._hover_remaining > 0
                    and tick >= dn._detection_ready_tick
                    for dn in self.drones
                ):
                    self.monitored_jam_ticks += 1

    def _open_jam_event(self, jid, direction, start_tick):
        self._next_jam_event_id += 1
        key = (jid, direction)
        evt = {
            "event_id": self._next_jam_event_id,
            "junction_id": jid,
            "direction": direction,
            "start_tick": start_tick,
            "end_tick": None,
            "duration": None,
            "detected": False,
            "first_detect_tick": None,
            "detection_delay": None,
            "peak_queue": 0,
            "peak_density": 0.0,
            "min_velocity": None,
            "max_delta_k": 0.0,
            # ── Jam longevity across signal cycles
            "red_phases_spanned": 0,   # how many red phases the jam survived
            "_was_red_last_tick": False,
            # ── Post-detection residual duration
            # Ticks the jam persisted after the drone first confirmed it.
            # If zero or small, controller adaptation broke the jam quickly.
            "post_detect_duration": None,
            # ── Drone coverage during jam
            "drone_visit_ticks": 0,
            "num_drone_visits": 0,
            "_drone_was_present_last_tick": False,
            # ── Traffic metric accumulators
            "_sum_flow": 0.0,
            "_sum_delta_k": 0.0,
            "_sum_velocity": 0.0,
            "_obs_count": 0,
            "avg_flow": None,
            "avg_delta_k": None,
            "avg_velocity": None,
            # ── Prediction memory
            "predicted_before_start": False,
            "prediction_tick": None,
            "lead_time_ticks": None,
            "prediction_drone_id": None,
        }
        # Scan prediction memory: was a prediction issued before the jam started
        # and still within that prediction's horizon window?
        memory = self._prediction_memory.get(key, [])
        pre_start = [
            m for m in memory
            if m["tick"] < start_tick
            and m.get("deadline_tick", m["tick"] + self.prediction_horizon) >= start_tick
        ]
        if pre_start:
            # Most-recent in-horizon pre-jam prediction = shortest plausible lead time
            best = max(pre_start, key=lambda m: m["tick"])
            evt["predicted_before_start"] = True
            evt["prediction_tick"] = best["tick"]
            evt["lead_time_ticks"] = start_tick - best["tick"]
            evt["prediction_drone_id"] = best["drone_id"]
        self._active_jam_events[key] = evt

    def _update_jam_event_stats(self, key, queue, density, avg_velocity, delta_k, flow=0.0):
        evt = self._active_jam_events.get(key)
        if evt is None:
            return
        evt["peak_queue"] = max(evt["peak_queue"], int(queue))
        evt["peak_density"] = max(evt["peak_density"], float(density))
        evt["max_delta_k"] = max(evt["max_delta_k"], float(delta_k))
        if evt["min_velocity"] is None:
            evt["min_velocity"] = float(avg_velocity)
        else:
            evt["min_velocity"] = min(evt["min_velocity"], float(avg_velocity))
        # Accumulate for per-jam averages
        evt["_sum_flow"]     += float(flow)
        evt["_sum_delta_k"]  += float(delta_k)
        evt["_sum_velocity"] += float(avg_velocity)
        evt["_obs_count"]    += 1

    def _close_jam_event(self, key, end_tick):
        evt = self._active_jam_events.pop(key, None)
        if evt is None:
            return
        evt["end_tick"] = end_tick
        evt["duration"] = max(1, end_tick - evt["start_tick"] + 1)
        # Post-detection residual duration: how many ticks the jam survived
        # after the drone first confirmed it.  A small value means the
        # controller’s signal-timing adaptation resolved the jam quickly.
        if evt.get("detected") and evt.get("first_detect_tick") is not None:
            evt["post_detect_duration"] = max(0, end_tick - evt["first_detect_tick"])
        else:
            evt["post_detect_duration"] = None
        # Compute per-jam traffic averages
        n = max(evt["_obs_count"], 1)
        evt["avg_flow"]     = round(evt["_sum_flow"]     / n, 4)
        evt["avg_delta_k"]  = round(evt["_sum_delta_k"]  / n, 4)
        evt["avg_velocity"] = round(evt["_sum_velocity"] / n, 4)
        self._completed_jam_events.append(evt)
        self.logger.log_jam_event(evt)

    def _analyze_approach_segment(self, controller, approach_dir):
        """Objective macroscopic metrics for one approach direction."""
        block_cells = controller.block_cells
        opposite = _OPPOSITE[approach_dir]
        dx, dy = {"N": (0, 1), "S": (0, -1),
                  "E": (1, 0), "W": (-1, 0)}[opposite]
        edge_cells = self._junction_edge_cells(block_cells, opposite)
        if not edge_cells:
            return {
                "density": 0.0,
                "avg_velocity": 0.0,
                "flow": 0.0,
                "car_count": 0,
                "segment_length": 0,
                "delta_k": 0.0,
            }

        total_cells = 0
        total_cars = 0
        total_velocity = 0.0
        inner_cars = 0
        inner_velocity = 0.0
        outer_cars = 0
        outer_velocity = 0.0
        macro_depth = 15
        half_depth = macro_depth // 2

        for ex, ey in edge_cells:
            for step in range(1, macro_depth + 1):
                rx, ry = ex + dx * step, ey + dy * step
                if not (0 <= rx < self.width and 0 <= ry < self.height):
                    break
                road = next(
                    (a for a in self.grid.get_cell_list_contents([(rx, ry)])
                     if isinstance(a, RoadCell)
                     and a.direction == approach_dir),
                    None,
                )
                if road is None:
                    break
                total_cells += 1
                if road.car is not None:
                    v = getattr(road.car, "velocity", 0)
                    total_cars += 1
                    total_velocity += v
                    if step <= half_depth:
                        inner_cars += 1
                        inner_velocity += v
                    else:
                        outer_cars += 1
                        outer_velocity += v

        seg_len = max(total_cells, 1)
        density = total_cars / seg_len
        avg_velocity = total_velocity / max(total_cars, 1) if total_cars else 0.0
        flow = density * avg_velocity

        inner_len = max(1, min(half_depth, seg_len))
        outer_len = max(1, seg_len - inner_len)
        q_in = (outer_cars / outer_len) * (outer_velocity / max(outer_cars, 1)) if outer_cars else 0.0
        q_out = (inner_cars / inner_len) * (inner_velocity / max(inner_cars, 1)) if inner_cars else 0.0
        delta_k = q_in - q_out

        return {
            "density": round(density, 4),
            "avg_velocity": round(avg_velocity, 4),
            "flow": round(flow, 4),
            "car_count": total_cars,
            "segment_length": total_cells,
            "delta_k": round(delta_k, 4),
        }

    @staticmethod
    def _junction_edge_cells(block_cells, side):
        if not block_cells:
            return []
        cells = list(block_cells)
        if side == "W":
            v = min(c[0] for c in cells)
            return [(x, y) for x, y in cells if x == v]
        if side == "E":
            v = max(c[0] for c in cells)
            return [(x, y) for x, y in cells if x == v]
        if side == "S":
            v = min(c[1] for c in cells)
            return [(x, y) for x, y in cells if y == v]
        if side == "N":
            v = max(c[1] for c in cells)
            return [(x, y) for x, y in cells if y == v]
        return []

    @staticmethod
    def _microscopic_queue_threshold(segment_length):
        """Adaptive queue threshold.

        - Baseline trigger: queue length > 6  (implemented as >= 7)
        - If segment length <= 6, threshold becomes the segment length.
        """
        seg_len = int(segment_length or 0)
        if 0 < seg_len <= 6:
            return seg_len
        return 7

    def _microscopic_queue_trigger(self, queue_length, segment_length):
        threshold = self._microscopic_queue_threshold(segment_length)
        return int(queue_length or 0) >= threshold

    def _count_approach_queue(self, controller, approach_dir):
        """
        Max per-lane queue length on the road approaching this
        junction from *approach_dir*.

        Replicates the drone's counting logic so the model has an
        independent, objective view.
        """
        block_cells = controller.block_cells
        opposite = _OPPOSITE[approach_dir]
        dx, dy = {"N": (0, 1), "S": (0, -1),
                  "E": (1, 0), "W": (-1, 0)}[opposite]

        # Find edge cells on the approach side
        cells = list(block_cells)
        if opposite == "W":
            v = min(c[0] for c in cells)
            edge = [(x, y) for x, y in cells if x == v]
        elif opposite == "E":
            v = max(c[0] for c in cells)
            edge = [(x, y) for x, y in cells if x == v]
        elif opposite == "S":
            v = min(c[1] for c in cells)
            edge = [(x, y) for x, y in cells if y == v]
        elif opposite == "N":
            v = max(c[1] for c in cells)
            edge = [(x, y) for x, y in cells if y == v]
        else:
            edge = []

        if not edge:
            return 0

        max_per_lane = 0
        for ex, ey in edge:
            lane_queued = 0
            for step in range(1, SCAN_DEPTH + 1):
                rx, ry = ex + dx * step, ey + dy * step
                if not (0 <= rx < self.width and 0 <= ry < self.height):
                    break
                road = next(
                    (a for a in self.grid.get_cell_list_contents([(rx, ry)])
                     if isinstance(a, RoadCell)
                     and a.direction == approach_dir),
                    None,
                )
                if road and road.car is not None:
                    lane_queued += 1
                elif road and road.car is None:
                    break
                else:
                    break
            max_per_lane = max(max_per_lane, lane_queued)
        return max_per_lane

    def get_monitoring_efficiency(self):
        """Fraction of jammed-junction-ticks that were monitored."""
        if self.total_jam_ticks == 0:
            return None
        return self.monitored_jam_ticks / self.total_jam_ticks
