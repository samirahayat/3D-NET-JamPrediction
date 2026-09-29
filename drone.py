"""
drone.py - Monitoring drones for traffic-jam detection and prediction.

Drones fly continuously from junction to junction, collecting both
microscopic (queue-length) and macroscopic (density, velocity, flow)
traffic data.  They communicate in real time via a 5G network, so
all reports are delivered instantly to the ground station **and**
directly to the relevant junction controller.

Behaviour
---------
    1. **Fly** from junction to junction in a fixed round-robin order.
    2. **Hover** over the junction for a configurable number of ticks
       (default 4), inspecting and reporting every tick.  This gives
       multiple observations for more reliable detection/prediction.
    3. **Coordinate** passively via report sharing (route remains fixed).
    4. **Detect** jams using combined microscopic + macroscopic criteria.
    5. **Predict** near-term congestion from observed inflow density.
    6. **Report** via 5G to the ground station and the junction
       controller simultaneously (no range limitation).
    7. **Learn**: junctions where jams were observed receive a higher
       monitoring weight.  Weights are shared between drones when
       they are within communication range of each other.

Metrics exposed:
    ``junctions_visited``  — total inspections performed
    ``jams_detected``      — times a jam was found on arrival
"""

from mesa import Agent
import random

from cells import RoadCell
from car import CarAgent, DELTAS
from ground_station import JunctionReport


# ===================================================================
#  Constants
# ===================================================================

JAM_THRESHOLD = 7          # fallback queue threshold (model may adapt by segment length)
SCAN_DEPTH = 6             # road cells to scan back from junction edge
MACRO_SCAN_DEPTH = 15      # deeper scan for macroscopic density / flow
JAM_WEIGHT_INCREMENT = 0.5 # added to a junction weight on each jam tick
BASE_WEIGHT = 1.0          # minimum / default weight

# Macroscopic jam criteria
DENSITY_JAM_THRESHOLD = 0.5   # density (fraction of cells occupied) → jam
VELOCITY_JAM_THRESHOLD = 1.0  # avg speed (cells/step) below which → jam

_OPPOSITE = {"N": "S", "S": "N", "E": "W", "W": "E"}
_APPROACH_DIRS = ("N", "S", "E", "W")


def _approach_has_green(controller, direction):
    """
    Return True when *direction* currently has the green light.

    E/W (horizontal) are active during H_GREEN and H_YELLOW.
    N/S (vertical)   are active during V_GREEN and V_YELLOW.

    We include YELLOW here because cars that entered on GREEN are
    still clearing the junction; the approach was flowing and any
    congestion reading is still meaningful.
    """
    phase = getattr(controller, "phase", "H_GREEN")
    if direction in ("E", "W"):
        return phase.startswith("H")
    return phase.startswith("V")


def _approach_is_pure_green(controller, direction):
    """
    Return True when *direction* has a **pure green** light only.

    Unlike ``_approach_has_green``, this excludes the YELLOW phase so
    that the capacity-based jam trigger is not evaluated while the
    light is transitioning — flow readings during yellow are
    transient and should not open or close a jam event.

    E/W (horizontal) → True only during H_GREEN.
    N/S (vertical)   → True only during V_GREEN.
    """
    phase = getattr(controller, "phase", "H_GREEN")
    if direction in ("E", "W"):
        return phase == "H_GREEN"
    return phase == "V_GREEN"


# ===================================================================
#  DroneAgent
# ===================================================================

class DroneAgent(Agent):
    """
    A monitoring drone that patrols junctions and reports to the
    ground station.

    Parameters
    ----------
    unique_id : int
    model : TrafficModel
    drone_index : int
        Index among all drones (used to spread initial positions).
    """

    def __init__(self, unique_id, model, drone_index=0):
        super().__init__(unique_id, model)
        self.drone_index = drone_index

        # -- Current state -----------------------------------------
        self.current_junction_id = None
        self._hover_remaining = 0      # ticks left at current junction

        # -- Camera processing state --------------------------------
        # Drone cannot confirm detections until _detection_ready_tick.
        # Hover total = random processing time + one full signal cycle.
        self._detection_ready_tick = 0
        self._arrival_tick = 0

        # -- Flight ------------------------------------------------
        self._flight_path = []
        self._flight_step = 0
        self._target_jid = None
        self._patrol_order = []
        self._patrol_index = 0

        # -- Learned knowledge -------------------------------------
        self.jam_weights = {}           # jid -> float
        self.last_visit_tick = {}       # jid -> tick (own or shared)

        # -- Reports waiting for delivery to ground station --------
        self.pending_reports = []

        # -- Latest inspection cache (shared with peers) -----------
        self.last_inspection = {}       # jid -> dict

        # -- Statistics --------------------------------------------
        self.junctions_visited = 0
        self.jams_detected = 0

        # -- Per-drone prediction cooldown --------------------------
        # Each drone maintains its own cooldown per (jid, direction).
        # Other drones are NOT blocked by this drone's cooldown —
        # each drone independently contributes prediction records.
        self._pred_cooldown = {}         # (jid, dir) -> last_pred_tick

        # -- Detection majority-vote counters -----------------------
        # Counts consecutive green-phase scan ticks where all jam
        # criteria were met.  Reset to zero on each new junction
        # arrival (via _calc_hover_ticks) and on any criteria failure.
        # A direction is only confirmed jammed once the vote count
        # reaches drone_min_detect_votes (model param, default 2).
        self._hover_detect_votes = {}    # dir -> int

        # -- Initialise weights and starting position --------------
        for jid in model.junction_controllers:
            self.jam_weights[jid] = BASE_WEIGHT
            self.last_visit_tick[jid] = -1

        self._place_initial(model)

    # -------------------------------------------------------------
    #  Placement
    # -------------------------------------------------------------

    def _place_initial(self, model):
        """Spread drones across junctions based on *drone_index*."""
        jids = list(model.junction_controllers.keys())
        if not jids:
            return
        ordered = self._build_patrol_order(model) or jids
        self._patrol_order = ordered
        idx = self.drone_index % len(ordered)
        self._patrol_index = idx
        start_jid = ordered[idx]
        centre = self._junction_centre(start_jid)
        if centre:
            try:
                model.grid.place_agent(self, centre)
            except Exception:
                pass
            self.current_junction_id = start_jid

    # =============================================================
    #  Core tick
    # =============================================================

    HOVER_TICKS = 4  # ticks a drone stays over a junction to scan

    def step(self):
        """Main per-tick logic — fly, hover & scan, or pick next."""
        if not self.model.junction_controllers:
            return

        tick = getattr(self.model, "_tick_count", 0)

        # 1. Share knowledge with nearby drones
        self._share_info_with_nearby_drones()

        # 2. Flying, hovering, or idle
        if self._flight_path and self._flight_step < len(self._flight_path):
            self._fly(tick)
        elif self._hover_remaining > 0:
            # Hovering over this junction.
            # Only scan after the camera processing window has elapsed.
            if self.current_junction_id and tick >= self._detection_ready_tick:
                self._inspect_and_report(self.current_junction_id, tick)
            self._hover_remaining -= 1
            if self._hover_remaining <= 0:
                self._pick_next_junction(tick)
        else:
            # Idle: only at the very first tick before any hover is set.
            if self.current_junction_id:
                self._hover_remaining = self._calc_hover_ticks(tick)
            else:
                self._pick_next_junction(tick)

        # 3. Deliver any pending reports via 5G (instant)
        self._deliver_reports()

    def advance(self):
        pass

    # =============================================================
    #  Hover duration calculation
    # =============================================================

    def _calc_hover_ticks(self, arrival_tick):
        """
        Compute the total hover duration for one junction visit.

        Total = random camera-processing time  +  one full signal cycle.

        One full signal cycle covers H_GREEN + H_YELLOW + V_GREEN + V_YELLOW
        so the drone is guaranteed to observe both approach directions
        during their respective green phases.

        The drone cannot confirm detections until after the processing
        window; ``_detection_ready_tick`` is set accordingly.
        """
        hover_min       = getattr(self.model, "hover_ticks_min", 10)
        hover_max       = getattr(self.model, "hover_ticks_max", 60)
        green_dur       = getattr(self.model, "green_duration",  12)
        yellow_dur      = getattr(self.model, "yellow_duration",  4)

        processing_ticks    = random.randint(hover_min, hover_max)
        signal_cycle_ticks  = 2 * (green_dur + yellow_dur)  # full H+V cycle

        self._arrival_tick         = arrival_tick
        self._detection_ready_tick = arrival_tick + processing_ticks

        # Reset detection-vote counters for this new junction visit.
        # Votes accumulated at the previous junction must not carry over.
        self._hover_detect_votes = {}

        return processing_ticks + signal_cycle_ticks

    # =============================================================
    #  Flying between junctions
    # =============================================================

    def _fly(self, tick):
        """Advance along the pre-built flight path.

        The drone does **not** detect or report anything during flight.
        Detection only happens while hovering (see step()).
        """
        speed = getattr(self.model, "drone_speed", 2)

        for _ in range(speed):
            if self._flight_step < len(self._flight_path):
                target = self._flight_path[self._flight_step]
                try:
                    self.model.grid.move_agent(self, target)
                except Exception:
                    pass
                self._flight_step += 1

            if self._flight_step >= len(self._flight_path):
                # Arrived at destination junction — begin hover (no detection here).
                jid = self._target_jid
                if jid:
                    self.current_junction_id = jid
                    # If a peer is already hovering here, skip to the next free junction.
                    peer_here = any(
                        dn.unique_id != self.unique_id
                        and dn._hover_remaining > 0
                        and dn.current_junction_id == jid
                        for dn in getattr(self.model, "drones", [])
                    )
                    if peer_here:
                        self._pick_next_junction(tick)
                    else:
                        self._hover_remaining = self._calc_hover_ticks(tick)
                break

    # =============================================================
    #  Next-junction selection (fixed round-robin)
    # =============================================================

    def _pick_next_junction(self, tick):
        """
        Choose the next junction in a fixed cyclic order, skipping any
        junction that is currently being hovered over or actively targeted
        by another drone.  If every junction is occupied by peers, fall
        back to the normal next-in-sequence choice so the drone is never
        stranded.
        """
        if not self._patrol_order:
            self._patrol_order = self._build_patrol_order(self.model)
        if not self._patrol_order:
            return

        if self.current_junction_id in self._patrol_order:
            self._patrol_index = self._patrol_order.index(self.current_junction_id)

        monitored = self._get_monitored_by_others()
        n = len(self._patrol_order)

        # Walk the patrol order from the next position, skip monitored junctions.
        # Try up to n candidates; if all are occupied fall back to next-in-sequence.
        chosen_index = None
        chosen_jid   = None
        for offset in range(1, n + 1):
            candidate_index = (self._patrol_index + offset) % n
            candidate_jid   = self._patrol_order[candidate_index]
            if candidate_jid == self.current_junction_id:
                continue
            if candidate_jid not in monitored:
                chosen_index = candidate_index
                chosen_jid   = candidate_jid
                break

        # Fallback: just take the immediate next slot
        if chosen_jid is None:
            chosen_index = (self._patrol_index + 1) % n
            chosen_jid   = self._patrol_order[chosen_index]

        self._patrol_index = chosen_index
        self._target_jid   = chosen_jid
        self._build_flight_to(chosen_jid)

    def _get_monitored_by_others(self):
        """Junction IDs currently targeted or being scanned by peer drones."""
        monitored = set()
        for drone in getattr(self.model, "drones", []):
            if drone.unique_id == self.unique_id:
                continue
            if drone._target_jid:
                monitored.add(drone._target_jid)
            # Also exclude junctions a peer is currently hovering over
            if drone._hover_remaining > 0 and drone.current_junction_id:
                monitored.add(drone.current_junction_id)
        return monitored

    # =============================================================
    #  Junction inspection (microscopic + macroscopic)
    # =============================================================

    def _inspect_and_report(self, jid, tick):
        """
        Inspect a junction while hovering.

        The drone camera sees all four approach directions simultaneously.

        **Jam detection** (pure-green only, mirrors ground-truth trigger):
        All three criteria must hold simultaneously:
          1. density  >= jam_density_threshold       — segment genuinely loaded
          2. mean_speed < jam_speed_threshold*v_max  — traffic is slow
          3. queue_length >= jam_queue_min           — real queue at stop line

        **Congestion prediction** (green + yellow, fires before jam threshold):
          density >= 60 % of jam_density_threshold
          AND (mean_speed < 1.6 × jam_speed_threshold × v_max
               OR delta_k > 0   — inflow exceeding outflow)
          AND queue >= 1
        """
        queue_lengths   = self._get_queue_lengths(jid)
        segment_metrics = self._get_segment_metrics(jid)
        predictions     = self._predict_inflow(jid)

        # Pull jam thresholds from model (fall back to defaults if absent)
        v_max              = getattr(self.model, "v_max",                4)
        jam_density_thr    = getattr(self.model, "jam_density_threshold", 0.30)
        jam_speed_thr      = getattr(self.model, "jam_speed_threshold",   0.40)
        jam_queue_min      = getattr(self.model, "jam_queue_min",          1)
        jam_speed_limit    = jam_speed_thr * v_max           # e.g. 1.6

        # Warning-zone thresholds (softer than jam thresholds)
        warn_density_thr   = jam_density_thr * 0.60          # e.g. 0.24
        warn_speed_limit   = min(jam_speed_thr * 1.60, 0.80) * v_max  # e.g. 2.56

        ctrl = self.model.junction_controllers.get(jid)

        # ── Multi-criteria jam detection with majority voting (pure green only) ──
        # A direction is only confirmed jammed after MIN_DETECT_VOTES
        # consecutive pure-green-phase scan ticks all meeting the criteria.
        # This models camera-confirmation latency and rejects single-tick
        # transients.  Any criteria failure resets the vote streak.
        MIN_DETECT_VOTES = getattr(self.model, "drone_min_detect_votes", 2)
        jam_directions = set()
        for d in _APPROACH_DIRS:
            seg       = segment_metrics.get(d, {})
            density   = seg.get("density",      0.0)
            avg_vel   = seg.get("avg_velocity", 0.0)
            car_count = seg.get("car_count",    0)
            queue     = queue_lengths.get(d,    0)
            criteria_met = (
                ctrl is not None
                and _approach_is_pure_green(ctrl, d)
                and car_count >= 1
                and density  >= jam_density_thr
                and avg_vel  <  jam_speed_limit
                and queue    >= jam_queue_min
            )
            if criteria_met:
                self._hover_detect_votes[d] = self._hover_detect_votes.get(d, 0) + 1
            else:
                # Any failure resets the streak — non-green phases included
                self._hover_detect_votes[d] = 0

            if self._hover_detect_votes.get(d, 0) >= MIN_DETECT_VOTES:
                jam_directions.add(d)

        is_jammed = len(jam_directions) > 0

        # ── Congestion prediction — temporal delta_k from GS memory (green + yellow) ──
        # Uses a TEMPORAL density gradient (current minus last GS-reported density)
        # instead of the spatial inner/outer split.  The Ground Station preserves
        # the previous observation; the drone fetches it to compute the gradient,
        # honouring the architecture where drones inspect-and-report and the GS
        # retains history for decision support.
        #
        # Per-drone cooldown: each drone independently tracks the last tick it
        # issued a prediction per (jid, direction).  Other drones are NOT blocked
        # by this drone's cooldown — they can predict the same direction freely.
        prediction_horizon = getattr(self.model, "prediction_horizon", 20)
        gs = getattr(self.model, "ground_station", None)
        prev_seg_metrics = {}
        if gs is not None:
            prev_status = gs.junction_status.get(jid, {})
            prev_seg_metrics = prev_status.get("segment_metrics", {})

        predicted_congestion_dirs = set()
        for d in _APPROACH_DIRS:
            if d in jam_directions:
                continue
            seg       = segment_metrics.get(d, {})
            density   = seg.get("density",      0.0)
            avg_vel   = seg.get("avg_velocity", 0.0)
            car_count = seg.get("car_count",    0)
            queue     = queue_lengths.get(d,    0)

            # Temporal delta_k: density worsening since last GS-stored observation
            prev_seg     = prev_seg_metrics.get(d, {})
            prev_density = prev_seg.get("density", density)  # no change if no history
            temporal_dk  = density - prev_density             # positive = worsening

            # Per-drone cooldown: skip if this drone predicted recently
            last_pred_tick = self._pred_cooldown.get((jid, d), -10 ** 9)
            if tick - last_pred_tick < prediction_horizon:
                continue

            if (ctrl is not None
                    and _approach_has_green(ctrl, d)
                    and car_count >= 1
                    and density  >= warn_density_thr
                    and queue    >= 1
                    and (avg_vel < warn_speed_limit or temporal_dk > 0.02)):
                predicted_congestion_dirs.add(d)
                self._pred_cooldown[(jid, d)] = tick

        # Update own records
        self.last_visit_tick[jid] = tick
        self.last_inspection[jid] = {
            "is_jammed": is_jammed,
            "queues": queue_lengths,
            "segment_metrics": segment_metrics,
            "predictions": predictions,
            "jam_directions": jam_directions,
            "predicted_congestion_dirs": predicted_congestion_dirs,
            "tick": tick,
        }
        self.junctions_visited += 1

        # Track jam detection count
        if is_jammed:
            self.jams_detected += 1

        # Push detailed observation rows to model-level logger / evaluator
        if hasattr(self.model, "record_drone_observation"):
            self.model.record_drone_observation(
                jid=jid,
                tick=tick,
                drone_id=self.unique_id,
                queue_lengths=queue_lengths,
                segment_metrics=segment_metrics,
                jam_directions=jam_directions,
                predicted_congestion_dirs=predicted_congestion_dirs,
                predictions=predictions,
            )

        # Adjust jam weight
        if is_jammed:
            self.jam_weights[jid] = (
                self.jam_weights.get(jid, BASE_WEIGHT)
                + JAM_WEIGHT_INCREMENT
            )
        else:
            cur = self.jam_weights.get(jid, BASE_WEIGHT)
            self.jam_weights[jid] = max(BASE_WEIGHT, cur * 0.99)

        # Queue report for delivery
        report = JunctionReport(jid, tick, is_jammed,
                                queue_lengths, self.unique_id,
                                segment_metrics=segment_metrics,
                                predictions=predictions)
        self.pending_reports.append(report)

    def _get_queue_lengths(self, jid):
        """Return {direction: queue_length} for *jid*."""
        ctrl = self.model.junction_controllers.get(jid)
        if ctrl is None:
            return {}
        return {d: self._count_approach_queue(ctrl, d)
                for d in _APPROACH_DIRS}

    # =============================================================
    #  Macroscopic traffic metrics (density, velocity, flow)
    # =============================================================

    def _get_segment_metrics(self, jid):
        """
        Compute density, average velocity, and flow for each approach
        direction of junction *jid*.

        Returns ``{direction: {density, avg_velocity, flow, car_count,
        segment_length, delta_k}}``
        """
        ctrl = self.model.junction_controllers.get(jid)
        if ctrl is None:
            return {}
        return {d: self._analyze_approach_segment(ctrl, d)
                for d in _APPROACH_DIRS}

    def _analyze_approach_segment(self, controller, approach_dir):
        """
        Scan the road segment approaching *controller* from
        *approach_dir* and compute macroscopic metrics.

        Core Metrics
        • Density (k): vehicles / segment length
        • Velocity (v): average speed  (cells/step)
        • Flow (q): k · v
        • Δk: q_in − q_out  (net accumulation indicator)
        """
        block_cells = controller.block_cells
        opposite = _OPPOSITE[approach_dir]
        dx, dy = DELTAS[opposite]
        edge_cells = self._junction_edge_cells(block_cells, opposite)
        if not edge_cells:
            return {"density": 0, "avg_velocity": 0, "flow": 0,
                    "car_count": 0, "segment_length": 0, "delta_k": 0}

        total_cells = 0
        total_cars = 0
        total_velocity = 0.0
        # For Δk: track cars in inner half (near junction) vs outer half
        inner_cars = 0
        inner_velocity = 0.0
        outer_cars = 0
        outer_velocity = 0.0
        half_depth = MACRO_SCAN_DEPTH // 2

        for ex, ey in edge_cells:
            for step in range(1, MACRO_SCAN_DEPTH + 1):
                rx, ry = ex + dx * step, ey + dy * step
                if not (0 <= rx < self.model.width
                        and 0 <= ry < self.model.height):
                    break
                road = next(
                    (a for a in
                     self.model.grid.get_cell_list_contents([(rx, ry)])
                     if isinstance(a, RoadCell)
                     and a.direction == approach_dir),
                    None,
                )
                if road is None:
                    break
                total_cells += 1
                if road.car is not None:
                    v = getattr(road.car, "velocity", 1)
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
        avg_velocity = total_velocity / max(total_cars, 1) if total_cars else 0
        flow = density * avg_velocity

        # Δk approximation: net accumulation = inflow - outflow
        # inner segment ~ outflow side, outer segment ~ inflow side
        inner_len = max(1, sum(1 for _ in range(min(half_depth, seg_len))))
        outer_len = max(1, seg_len - inner_len)
        q_in = (outer_cars / outer_len) * (outer_velocity / max(outer_cars, 1)) if outer_cars else 0
        q_out = (inner_cars / inner_len) * (inner_velocity / max(inner_cars, 1)) if inner_cars else 0
        delta_k = q_in - q_out

        return {
            "density": round(density, 3),
            "avg_velocity": round(avg_velocity, 2),
            "flow": round(flow, 3),
            "car_count": total_cars,
            "segment_length": total_cells,
            "delta_k": round(delta_k, 3),
        }

    # =============================================================
    #  Traffic flow prediction
    # =============================================================

    def _predict_inflow(self, jid):
        """
        Estimate near-term inflow per approach direction by observing
        vehicle density and velocity in the *outer* portion of the
        approach segment (far from the junction).

        Returns ``{direction: {estimated_inflow, car_count, avg_velocity}}``
        """
        ctrl = self.model.junction_controllers.get(jid)
        if ctrl is None:
            return {}

        predictions = {}
        for d in _APPROACH_DIRS:
            seg = self._analyze_approach_segment(ctrl, d)
            # Cars further from the junction represent upcoming demand
            outer_flow = max(seg.get("flow", 0), 0)
            predictions[d] = {
                "estimated_inflow": round(outer_flow, 3),
                "car_count": seg["car_count"],
                "avg_velocity": seg["avg_velocity"],
                "delta_k": seg.get("delta_k", 0),
            }
        return predictions

    # =============================================================
    #  Drone-to-drone information sharing
    # =============================================================

    def _share_info_with_nearby_drones(self):
        """
        Merge jam weights and visit timestamps with every peer
        drone within communication range.
        """
        comm_sq = self._comm_range_sq()
        if comm_sq <= 0:
            return

        for drone in getattr(self.model, "drones", []):
            if drone.unique_id == self.unique_id:
                continue
            if drone.pos is None or self.pos is None:
                continue

            dx = self.pos[0] - drone.pos[0]
            dy = self.pos[1] - drone.pos[1]
            if dx * dx + dy * dy <= comm_sq:
                # Share jam weights -- keep the higher value
                for jid, w in self.jam_weights.items():
                    if jid not in drone.jam_weights or w > drone.jam_weights[jid]:
                        drone.jam_weights[jid] = w
                # Share last-visit -- keep the more recent
                for jid, t in self.last_visit_tick.items():
                    if jid not in drone.last_visit_tick or t > drone.last_visit_tick[jid]:
                        drone.last_visit_tick[jid] = t

    # =============================================================
    #  Report delivery via 5G (instant, no range limitation)
    # =============================================================

    def _deliver_reports(self):
        """
        Push pending reports to the ground station **and** directly
        to the relevant junction controller via 5G.  No range check
        is needed — 5G provides full network coverage.
        """
        gs = getattr(self.model, "ground_station", None)
        if not self.pending_reports:
            return

        for report in self.pending_reports:
            # Ground station
            if gs is not None:
                gs.receive_report(report)

            # Direct to junction controller
            self._push_to_controller(report)

        self.pending_reports.clear()

    def _push_to_controller(self, report):
        """
        Send detection + prediction data directly to the junction
        controller so it can adapt signal timing immediately.
        """
        if not getattr(self.model, "adaptive_signal_control", True):
            return

        from drone import JAM_THRESHOLD  # avoid circular at module level

        jid = report.junction_id
        ctrl = self.model.junction_controllers.get(jid)
        if ctrl is None:
            return

        road_priority = getattr(self.model, "road_priority", "V")

        congested_approaches = set()
        for d, q in report.queue_lengths.items():
            seg_len = 0
            if report.segment_metrics:
                seg_len = report.segment_metrics.get(d, {}).get("segment_length", 0)
            if hasattr(self.model, "_microscopic_queue_trigger"):
                micro_trigger = self.model._microscopic_queue_trigger(q, seg_len)
            else:
                micro_trigger = q >= JAM_THRESHOLD
            if micro_trigger:
                congested_approaches.add("H" if d in ("E", "W") else "V")

        seg = report.segment_metrics or {}
        for d, metrics in seg.items():
            density = metrics.get("density", 0)
            avg_v = metrics.get("avg_velocity", 0)
            if density >= DENSITY_JAM_THRESHOLD and avg_v <= VELOCITY_JAM_THRESHOLD:
                congested_approaches.add("H" if d in ("E", "W") else "V")

        ctrl.apply_congestion_response(
            congested_approaches,
            predictions=report.predictions,
            road_priority=road_priority,
        )

    # =============================================================
    #  Flight-path helpers
    # =============================================================

    def _build_flight_to(self, jid):
        """Manhattan walk from current position to *jid* centre."""
        dest = self._junction_centre(jid)
        if dest is None or self.pos is None:
            self._flight_path = []
            return

        path = []
        cx, cy = self.pos
        tx, ty = dest
        while (cx, cy) != (tx, ty):
            if cx < tx:
                cx += 1
            elif cx > tx:
                cx -= 1
            elif cy < ty:
                cy += 1
            elif cy > ty:
                cy -= 1
            path.append((cx, cy))

        self._flight_path = path if path else [(tx, ty)]
        self._flight_step = 0

    def _junction_centre(self, jid):
        """Centre (x, y) of *jid* block cells."""
        ctrl = self.model.junction_controllers.get(jid)
        if ctrl and ctrl.block_cells:
            cells = list(ctrl.block_cells)
            return (sum(c[0] for c in cells) // len(cells),
                    sum(c[1] for c in cells) // len(cells))
        return None

    def _distance_to_junction(self, jid):
        """Manhattan distance to *jid* from current position."""
        c = self._junction_centre(jid)
        if c is None or self.pos is None:
            return float("inf")
        return abs(self.pos[0] - c[0]) + abs(self.pos[1] - c[1])

    def _comm_range_sq(self):
        r = getattr(self.model, "drone_comm_range", 15)
        return r * r

    # =============================================================
    #  Approach-queue counting
    # =============================================================

    def _count_approach_queue(self, controller, approach_dir):
        """Max per-lane queue length approaching from *approach_dir*."""
        block_cells = controller.block_cells
        opposite = _OPPOSITE[approach_dir]
        dx, dy = DELTAS[opposite]
        edge_cells = self._junction_edge_cells(block_cells, opposite)
        if not edge_cells:
            return 0

        max_per_lane = 0
        for ex, ey in edge_cells:
            lane_queued = 0
            for step in range(1, SCAN_DEPTH + 1):
                rx, ry = ex + dx * step, ey + dy * step
                if not (0 <= rx < self.model.width
                        and 0 <= ry < self.model.height):
                    break
                road = next(
                    (a for a in
                     self.model.grid.get_cell_list_contents([(rx, ry)])
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

    def _junction_edge_cells(self, block_cells, side):
        """Return junction cells on the given *side* edge."""
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

    # =============================================================
    #  Patrol order (for initial placement spread)
    # =============================================================

    @staticmethod
    def _build_patrol_order(model):
        """Geographic snake ordering of junctions for even spread."""
        jids = list(model.junction_controllers.keys())
        if not jids:
            return []

        from collections import defaultdict

        def _centre(jid):
            parts = jid.split("_")
            return (int(parts[1]), int(parts[2]))

        rows = defaultdict(list)
        for jid in jids:
            _x, _y = _centre(jid)
            rows[_y].append(jid)

        ordered = []
        for i, y_key in enumerate(sorted(rows.keys())):
            row = sorted(rows[y_key], key=lambda j: _centre(j)[0])
            if i % 2 == 1:
                row = list(reversed(row))
            ordered.extend(row)
        return ordered
